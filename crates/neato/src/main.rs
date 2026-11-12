//! Runs on the Raspberry Pi and bridges the Neato to the desktop.
//!
//! It opens the Neato's USB serial port and serves the most recent station
//! connection. While one is connected it spins the lidar and streams wheel
//! positions, bumpers, scans and battery state to it, each stamped with the Pi's monotonic
//! clock when it was read. A scan is stamped instead with when its sweep
//! finished, which `neato::SweepClock` works out from when the reply changes.
//!
//! The Neato drops off USB now and then (when it sleeps, or on a marginal
//! cable) and may come back under a different ttyACM number. A serial error
//! closes the port, and pi-node keeps looking for the robot and reopens it,
//! without dropping the station.
//!
//! Driving: the station sends wheel speeds, re-sent while its keys are
//! held. Each tick, the latest one becomes a `SetMotor` that moves each
//! wheel the distance it would cover in `--horizon-ms` at that speed, so the
//! next tick's move replaces it before it runs out. A changed command (a
//! new speed, or a stop) goes out as soon as the serial port is free rather
//! than at the next tick. If no command arrives for `WATCHDOG`, or the
//! station goes away, the wheels are stopped.
//!
//! The station can also switch the lidar off, and take the Neato out of
//! test mode so it charges on its dock. Out of test mode the wheels and
//! lidar stay off, and only the wheel, bumper and battery readings go on.

mod port;

use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use neato::cmd;
use robot::wire::{self, MAX_FRAME_LEN};
use robot::{Command, Msg, NodeInfo, Stamped};

use crate::port::NeatoPort;

const USAGE: &str = "usage: pi-node [--serial auto|/dev/ttyACM0] [--listen 0.0.0.0:7878] [--tick-ms 100] [--horizon-ms 300] [--stop disable|tiny|zero]";

/// Wheels and bumpers are polled every tick and the battery every tenth.
/// Settable with `--tick-ms`. The lidar is asked whenever its next sweep
/// should have finished (see `neato::SweepClock`).
const TICK: Duration = Duration::from_millis(100);
/// How long a drive command lasts, after which the wheels stop.
const WATCHDOG: Duration = Duration::from_millis(300);
/// How far ahead each `SetMotor` reaches. Settable with `--horizon-ms`.
const HORIZON: Duration = Duration::from_millis(300);
/// If the lidar's reply hasn't changed for this long, the scene is so still
/// (or so empty) that sweeps can't be told apart; the reply is sent anyway.
const SCAN_STALE: Duration = Duration::from_millis(500);
const CHARGER_EVERY_TICKS: u32 = 10;
/// Time for the lidar to reach speed after it is switched on.
const LDS_SPIN_UP: Duration = Duration::from_secs(2);
/// How often to look for the Neato while it's missing.
const REOPEN_EVERY: Duration = Duration::from_secs(2);
/// A station that stops reading for this long is treated as gone.
const WRITE_TIMEOUT: Duration = Duration::from_secs(2);

fn main() -> Result<()> {
    let mut serial = String::from("auto");
    let mut listen = String::from("0.0.0.0:7878");
    let mut timing = Timing {
        tick: TICK,
        horizon: HORIZON,
    };
    let ms = |v: Option<String>| -> Result<Duration> {
        Ok(Duration::from_millis(
            v.context(USAGE)?.parse().context(USAGE)?,
        ))
    };
    let mut stop_style = StopStyle::Disable;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--serial" => serial = args.next().context(USAGE)?,
            "--listen" => listen = args.next().context(USAGE)?,
            "--tick-ms" => timing.tick = ms(args.next())?,
            "--horizon-ms" => timing.horizon = ms(args.next())?,
            "--stop" => {
                stop_style = match args.next().as_deref() {
                    Some("zero") => StopStyle::Zero,
                    Some("tiny") => StopStyle::Tiny,
                    Some("disable") => StopStyle::Disable,
                    _ => bail!("{USAGE}"),
                }
            }
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(());
            }
            _ => bail!("unknown argument {arg:?}\n{USAGE}"),
        }
    }

    eprintln!(
        "pi-node {}: tick {:?}, SetMotor horizon {:?}, stop after {WATCHDOG:?} without a command",
        env!("GIT_VERSION"),
        timing.tick,
        timing.horizon
    );
    let mut robot = Robot::new(serial);
    robot.stop_style = stop_style;
    eprintln!("stopping with {stop_style:?}");
    robot.try_open();

    let listener = TcpListener::bind(&listen).with_context(|| format!("listening on {listen}"))?;
    eprintln!("waiting for the station on {listen}");
    // Accept on a separate thread, so a new station can take over even while
    // an old connection looks open but is dead (a half-open TCP link never
    // errors until something is written).
    let (tx, stations) = mpsc::channel();
    thread::spawn(move || {
        for stream in listener.incoming() {
            match stream {
                Ok(s) => {
                    if tx.send(s).is_err() {
                        return;
                    }
                }
                Err(e) => eprintln!("accept failed: {e}"),
            }
        }
    });

    let clock = Instant::now();
    let mut next = stations.recv().ok();
    while let Some(stream) = next.take() {
        let peer = stream
            .peer_addr()
            .map(|a| a.to_string())
            .unwrap_or_default();
        eprintln!("station connected from {peer}");
        match serve(&mut robot, stream, clock, &stations, timing) {
            Ok(Some(newer)) => {
                eprintln!("station {peer} replaced by a newer connection");
                next = Some(newer);
                continue;
            }
            Ok(None) => eprintln!("station {peer} disconnected"),
            Err(e) => eprintln!("station {peer} dropped: {e:#}"),
        }
        // Whenever nobody is watching, stop the wheels and the lidar and
        // leave test mode, so the Neato charges if it's on its dock. The
        // next station turns test mode back on.
        robot.set_test_mode(false);
        next = stations.recv().ok();
    }
    Ok(())
}

/// How a stop is sent. The firmware finishes the move in progress after a
/// zero `SetMotor`, so a stop ran on for the whole horizon (about 45 mm at
/// 150 mm/s). Measured 2026-11-12: zero 30-46 mm, tiny 12-28 mm (and a 1 mm
/// creep while idle), disable 9-16 mm. Disable is the default.
#[derive(Debug, Clone, Copy)]
enum StopStyle {
    /// `SetMotor 0 0 0`.
    Zero,
    /// A 1 mm move at 1 mm/s, which should replace the move in progress.
    Tiny,
    /// Disable both wheel motors, then enable them again.
    Disable,
}

#[derive(Clone, Copy)]
struct Timing {
    tick: Duration,
    horizon: Duration,
}

/// What the station has asked for so far on this connection.
#[derive(Clone, Copy)]
struct Wanted {
    /// The latest drive command (`WheelSpeeds` or `Stop`), and when it came.
    drive: Command,
    at: Instant,
    lidar: bool,
    test_mode: bool,
}

impl Default for Wanted {
    fn default() -> Self {
        Self {
            drive: Command::Stop,
            at: Instant::now(),
            lidar: true,
            test_mode: true,
        }
    }
}

type Latest = Arc<Mutex<Wanted>>;

/// The Neato, which may come and go.
struct Robot {
    serial: String,
    port: Option<NeatoPort>,
    lidar_on: bool,
    /// Whether the last `SetMotor` sent may still be moving the wheels.
    moving: bool,
    /// Whether the Neato is in test mode, which `open_and_greet` turns on.
    test_mode: bool,
    stop_style: StopStyle,
    wheels: neato::ContinuousWheels,
    sweep: neato::SweepClock,
    /// When the last scan went to the station.
    last_scan: Option<Instant>,
    last_attempt: Option<Instant>,
    /// The last open failure printed, so retries every 2 s don't flood the log.
    last_error: String,
}

impl Robot {
    fn new(serial: String) -> Self {
        Self {
            serial,
            port: None,
            lidar_on: false,
            moving: false,
            test_mode: false,
            stop_style: StopStyle::Disable,
            wheels: Default::default(),
            sweep: neato::SweepClock::new(),
            last_scan: None,
            last_attempt: None,
            last_error: String::new(),
        }
    }

    /// Opens the port if it's closed, at most every `REOPEN_EVERY`.
    fn try_open(&mut self) -> bool {
        if self.port.is_some() {
            return true;
        }
        if self
            .last_attempt
            .is_some_and(|t| t.elapsed() < REOPEN_EVERY)
        {
            return false;
        }
        let first = self.last_attempt.is_none();
        self.last_attempt = Some(Instant::now());
        let path = if self.serial == "auto" {
            port::find_port()
        } else {
            Some(self.serial.clone())
        };
        let Some(path) = path else {
            if first {
                eprintln!("no Neato found on USB yet (no /dev/ttyACM*); still looking");
            }
            return false;
        };
        match open_and_greet(&path) {
            Ok(port) => {
                self.port = Some(port);
                self.test_mode = true;
                self.wheels.rebase();
                self.last_error.clear();
                true
            }
            Err(e) => {
                let error = format!("could not open the Neato at {path}: {e:#}");
                if error != self.last_error {
                    eprintln!("{error}; will keep trying");
                    self.last_error = error;
                }
                false
            }
        }
    }

    /// Sends a query, closing the port on any serial error so the next
    /// round looks for the robot again.
    fn query(&mut self, command: &str) -> Option<String> {
        let port = self.port.as_mut()?;
        match port.query(command) {
            Ok(reply) => Some(reply),
            Err(e) => {
                eprintln!("lost the Neato ({e:#}); will keep looking for it");
                self.port = None;
                self.lidar_on = false;
                self.sweep.reset();
                self.moving = false;
                None
            }
        }
    }

    /// Makes sure the lidar is spinning, waiting for it the first time.
    fn lidar_up(&mut self) -> bool {
        if self.lidar_on {
            return true;
        }
        if self.query(cmd::LDS_ON).is_none() {
            return false;
        }
        thread::sleep(LDS_SPIN_UP);
        self.lidar_on = true;
        self.sweep.reset();
        self.last_scan = Some(Instant::now());
        true
    }

    /// Turns the latest command into a `SetMotor`, or stops the wheels if
    /// it is a stop or has gone stale.
    fn drive(&mut self, wanted: Wanted, horizon: Duration) {
        match wanted.drive {
            Command::WheelSpeeds {
                left_mm_s,
                right_mm_s,
            } if wanted.at.elapsed() < WATCHDOG => {
                let motor = cmd::wheel_speeds(left_mm_s, right_mm_s, horizon.as_secs_f32());
                if self.query(&motor).is_some() {
                    self.moving = true;
                }
            }
            _ => self.stop(),
        }
    }

    /// Enters or leaves test mode. Leaving stops the wheels and the lidar
    /// first, since the firmware takes them back.
    fn set_test_mode(&mut self, on: bool) {
        if on == self.test_mode {
            return;
        }
        if !on {
            self.stop();
            self.lidar_off();
        }
        let command = if on {
            cmd::TEST_MODE_ON
        } else {
            cmd::TEST_MODE_OFF
        };
        if self.query(command).is_some() {
            self.test_mode = on;
            self.wheels.rebase();
            eprintln!(
                "test mode {}",
                if on {
                    "on"
                } else {
                    "off: the Neato may charge"
                }
            );
        }
    }

    fn stop(&mut self) {
        if !self.moving {
            return;
        }
        let sent = match self.stop_style {
            StopStyle::Zero => self.query(&cmd::set_motor(0, 0, 0)).is_some(),
            StopStyle::Tiny => self.query(&cmd::set_motor(1, 1, 1)).is_some(),
            StopStyle::Disable => {
                let off = self.query(cmd::WHEELS_DISABLE).is_some();
                off && self.query(cmd::WHEELS_ENABLE).is_some()
            }
        };
        if sent {
            self.moving = false;
        }
    }

    fn lidar_off(&mut self) {
        if self.lidar_on {
            self.query(cmd::LDS_OFF);
            self.lidar_on = false;
        }
    }
}

fn open_and_greet(path: &str) -> Result<NeatoPort> {
    let mut port = NeatoPort::open(path)?;
    let version = port.query(cmd::GET_VERSION)?;
    eprintln!("connected to the Neato on {path}");
    for line in version.lines().map(str::trim).filter(|l| !l.is_empty()) {
        // Connected models report their Wi-Fi setup password here; keep it
        // out of logs.
        if line.to_ascii_lowercase().starts_with("appassword") {
            eprintln!("  APPassword,<hidden>");
        } else {
            eprintln!("  {line}");
        }
    }
    port.query(cmd::TEST_MODE_ON)?;
    Ok(port)
}

/// Streams readings to one station until it disconnects, or until a newer
/// station connects, which is returned so the caller can switch to it.
fn serve(
    robot: &mut Robot,
    mut stream: TcpStream,
    clock: Instant,
    stations: &Receiver<TcpStream>,
    timing: Timing,
) -> Result<Option<TcpStream>> {
    stream.set_nodelay(true)?;
    stream.set_write_timeout(Some(WRITE_TIMEOUT))?;
    // Say what is running here, so the station can show it.
    let hello = Stamped {
        t_ns: stamp(clock),
        seq: 0,
        msg: Msg::Node(NodeInfo {
            version: env!("GIT_VERSION").to_owned(),
            settings: format!(
                "--stop {} --tick-ms {} --horizon-ms {}",
                format!("{:?}", robot.stop_style).to_lowercase(),
                timing.tick.as_millis(),
                timing.horizon.as_millis()
            ),
        }),
    };
    stream.write_all(&wire::encode_frame(&hello)?)?;
    let connected = Arc::new(AtomicBool::new(true));
    let latest: Latest = Arc::default();
    let (wake_tx, wake) = mpsc::channel();
    let reader = {
        let stream = stream.try_clone()?;
        let connected = connected.clone();
        let latest = latest.clone();
        thread::spawn(move || {
            if let Err(e) = read_commands(stream, &latest, &wake_tx) {
                eprintln!("command stream ended: {e}");
            }
            connected.store(false, Ordering::Relaxed);
        })
    };

    let mut seq = Seqs::default();
    let mut tick: u32 = 0;
    let result = 'serve: loop {
        if !connected.load(Ordering::Relaxed) {
            break Ok(None);
        }
        if let Ok(newer) = stations.try_recv() {
            break Ok(Some(newer));
        }
        let started = Instant::now();
        // While the robot is missing, keep the station and keep looking.
        if robot.try_open() {
            let wanted = *latest.lock().unwrap();
            robot.set_test_mode(wanted.test_mode);
            let scanning = if wanted.test_mode && wanted.lidar {
                robot.lidar_up()
            } else {
                robot.lidar_off();
                false
            };
            if wanted.test_mode {
                robot.drive(wanted, timing.horizon);
            }
            let polls = Polls {
                scanning,
                charger: tick % CHARGER_EVERY_TICKS == 0,
            };
            let mut between = |robot: &mut Robot| drive_if_changed(robot, &wake, &latest, timing);
            if let Err(e) = poll_once(robot, &mut stream, clock, &mut seq, polls, &mut between) {
                break Err(e);
            }
            tick = tick.wrapping_add(1);
        }
        // Sleep out the tick, but send a changed command the moment it
        // arrives, and ask for a scan the moment its sweep should be done.
        while let Some(rest) = timing.tick.checked_sub(started.elapsed()) {
            let scanning = robot.lidar_on && robot.port.is_some();
            let until_scan = match robot.sweep.next_request() {
                _ if !scanning => rest,
                // Still finding the sweep timing: ask at every chance.
                None => Duration::ZERO,
                Some(due) => {
                    Duration::from_secs_f64((due - clock.elapsed().as_secs_f64()).max(0.0))
                }
            };
            match wake.recv_timeout(rest.min(until_scan)) {
                Ok(()) => drive_now(robot, &wake, &latest, timing),
                Err(_) if until_scan < rest => {
                    if let Some(scan) = poll_scan(robot, clock, &mut seq)
                        && let Err(e) = send(&mut stream, &scan)
                    {
                        break 'serve Err(e);
                    }
                }
                Err(_) => break,
            }
        }
    };
    let _ = stream.shutdown(std::net::Shutdown::Both);
    let _ = reader.join();
    result
}

/// Sends the newest drive command now, if one arrived since the last check.
fn drive_if_changed(robot: &mut Robot, wake: &Receiver<()>, latest: &Latest, timing: Timing) {
    if wake.try_recv().is_ok() {
        drive_now(robot, wake, latest, timing);
    }
}

fn drive_now(robot: &mut Robot, wake: &Receiver<()>, latest: &Latest, timing: Timing) {
    while wake.try_recv().is_ok() {}
    let wanted = *latest.lock().unwrap();
    if wanted.test_mode && robot.port.is_some() {
        robot.drive(wanted, timing.horizon);
    }
}

#[derive(Default)]
struct Seqs {
    wheels: u32,
    contacts: u32,
    lidar: u32,
    status: u32,
}

/// The slower queries to include in a round.
struct Polls {
    /// Whether the lidar is spinning; it is asked only when a sweep is due.
    scanning: bool,
    charger: bool,
}

fn next(seq: &mut u32) -> u32 {
    let s = *seq;
    *seq = s.wrapping_add(1);
    s
}

/// One round of queries. A reply that fails to parse is logged and skipped,
/// and a serial error closes the port (see `Robot::query`); only a dead
/// socket ends the round with an error.
fn poll_once(
    robot: &mut Robot,
    stream: &mut TcpStream,
    clock: Instant,
    seq: &mut Seqs,
    polls: Polls,
    between: &mut dyn FnMut(&mut Robot),
) -> Result<()> {
    let mut out = Vec::with_capacity(4);

    let t = stamp(clock);
    if let Some(reply) = robot.query(cmd::GET_MOTORS) {
        match neato::parse_motors(&reply) {
            Ok(w) => out.push(Stamped {
                t_ns: t,
                seq: next(&mut seq.wheels),
                msg: Msg::Wheels(robot.wheels.next(w)),
            }),
            Err(e) => log_bad_reply(cmd::GET_MOTORS, &e, &reply),
        }
    }

    between(robot);
    let t = stamp(clock);
    if let Some(reply) = robot.query(cmd::GET_DIGITAL_SENSORS) {
        match neato::parse_digital_sensors(&reply) {
            Ok(c) => out.push(Stamped {
                t_ns: t,
                seq: next(&mut seq.contacts),
                msg: Msg::Contacts(c),
            }),
            Err(e) => log_bad_reply(cmd::GET_DIGITAL_SENSORS, &e, &reply),
        }
    }

    between(robot);
    if polls.scanning
        && let Some(scan) = poll_scan(robot, clock, seq)
    {
        out.push(scan);
    }
    between(robot);

    if polls.charger {
        let t = stamp(clock);
        if let Some(reply) = robot.query(cmd::GET_CHARGER) {
            match neato::parse_charger(&reply) {
                Ok(s) => out.push(Stamped {
                    t_ns: t,
                    seq: next(&mut seq.status),
                    msg: Msg::Status(s),
                }),
                Err(e) => log_bad_reply(cmd::GET_CHARGER, &e, &reply),
            }
        }
    }

    for m in &out {
        send(stream, m)?;
    }
    Ok(())
}

fn send(stream: &mut TcpStream, m: &Stamped<Msg>) -> Result<()> {
    stream
        .write_all(&wire::encode_frame(m)?)
        .context("sending to the station")
}

/// Asks for a scan if a sweep should have finished by now. Returns it
/// stamped with when its sweep finished, or `None` if it isn't due or is
/// the sweep already sent.
fn poll_scan(robot: &mut Robot, clock: Instant, seq: &mut Seqs) -> Option<Stamped<Msg>> {
    let sent = clock.elapsed().as_secs_f64();
    if !robot.sweep.due(sent) {
        return None;
    }
    let reply = robot.query(cmd::GET_LDS_SCAN)?;
    let scan = match neato::parse_lds_scan(&reply) {
        Ok(s) => s,
        Err(e) => {
            log_bad_reply(cmd::GET_LDS_SCAN, &e, &reply);
            return None;
        }
    };
    let hz = if scan.scan_duration > 0.0 {
        1.0 / scan.scan_duration
    } else {
        0.0
    };
    let end = match robot.sweep.observe(sent, &reply, hz) {
        Some(end) => end,
        None if robot.last_scan.is_some_and(|t| t.elapsed() > SCAN_STALE) => sent,
        None => return None,
    };
    robot.last_scan = Some(Instant::now());
    Some(Stamped {
        t_ns: (end.max(0.0) * 1e9) as u64,
        seq: next(&mut seq.lidar),
        msg: Msg::Lidar(scan),
    })
}

fn stamp(clock: Instant) -> u64 {
    clock.elapsed().as_nanos() as u64
}

fn log_bad_reply(command: &str, err: &neato::ParseError, reply: &str) {
    let head: String = reply.chars().take(240).collect();
    eprintln!("could not parse {command} reply ({err}); it began: {head:?}");
}

/// Keeps the station's latest command in `latest` until it disconnects.
fn read_commands(
    mut stream: TcpStream,
    latest: &Latest,
    wake: &mpsc::Sender<()>,
) -> io::Result<()> {
    loop {
        let mut len = [0u8; 4];
        match stream.read_exact(&mut len) {
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(()),
            r => r?,
        }
        let len = u32::from_le_bytes(len) as usize;
        if len > MAX_FRAME_LEN {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("frame of {len} bytes"),
            ));
        }
        let mut body = vec![0; len];
        stream.read_exact(&mut body)?;
        match wire::decode_body::<Command>(&body) {
            Ok(c) => {
                let mut wanted = latest.lock().unwrap();
                match c {
                    Command::Lidar { on } => wanted.lidar = on,
                    Command::TestMode { on } => wanted.test_mode = on,
                    drive => {
                        let changed = wanted.drive != drive;
                        wanted.drive = drive;
                        wanted.at = Instant::now();
                        if changed {
                            // A new speed or a stop goes out between serial
                            // exchanges instead of waiting for the next tick.
                            let _ = wake.send(());
                        }
                    }
                }
            }
            Err(e) => eprintln!("ignoring an undecodable command: {e}"),
        }
    }
}
