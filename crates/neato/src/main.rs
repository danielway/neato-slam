//! Runs on the Raspberry Pi and bridges the Neato to the desktop.
//!
//! It opens the Neato's USB serial port and serves the most recent station
//! connection. While one is connected it spins the lidar and streams wheel
//! positions, bumpers, scans and battery state to it, each stamped with the Pi's monotonic
//! clock when it was read.
//!
//! The Neato drops off USB now and then (when it sleeps, or on a marginal
//! cable) and may come back under a different ttyACM number. A serial error
//! closes the port, and pi-node keeps looking for the robot and reopens it,
//! without dropping the station.
//!
//! Read-only: it never sends the Neato a drive command. Commands that
//! arrive from the desktop are logged and dropped. Driving will need a
//! deliberate change here, with the 300 ms stop watchdog from the design.

mod port;

use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use neato::cmd;
use robot::wire::{self, MAX_FRAME_LEN};
use robot::{Command, Msg, Stamped};

use crate::port::NeatoPort;

const USAGE: &str = "usage: pi-node [--serial auto|/dev/ttyACM0] [--listen 0.0.0.0:7878]";

/// Wheels and bumpers are polled every tick, the lidar every other tick and
/// the battery once a second.
const TICK: Duration = Duration::from_millis(100);
const SCAN_EVERY_TICKS: u32 = 2;
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
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--serial" => serial = args.next().context(USAGE)?,
            "--listen" => listen = args.next().context(USAGE)?,
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(());
            }
            _ => bail!("unknown argument {arg:?}\n{USAGE}"),
        }
    }

    let mut robot = Robot::new(serial);
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
        match serve(&mut robot, stream, clock, &stations) {
            Ok(Some(newer)) => {
                eprintln!("station {peer} replaced by a newer connection");
                next = Some(newer);
                continue;
            }
            Ok(None) => eprintln!("station {peer} disconnected"),
            Err(e) => eprintln!("station {peer} dropped: {e:#}"),
        }
        // Stop the lidar whenever nobody is watching.
        robot.lidar_off();
        next = stations.recv().ok();
    }
    Ok(())
}

/// The Neato, which may come and go.
struct Robot {
    serial: String,
    port: Option<NeatoPort>,
    lidar_on: bool,
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
        true
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
) -> Result<Option<TcpStream>> {
    stream.set_nodelay(true)?;
    stream.set_write_timeout(Some(WRITE_TIMEOUT))?;
    let connected = Arc::new(AtomicBool::new(true));
    let reader = {
        let stream = stream.try_clone()?;
        let connected = connected.clone();
        thread::spawn(move || {
            if let Err(e) = drain_commands(stream) {
                eprintln!("command stream ended: {e}");
            }
            connected.store(false, Ordering::Relaxed);
        })
    };

    let mut seq = Seqs::default();
    let mut tick: u32 = 0;
    let result = loop {
        if !connected.load(Ordering::Relaxed) {
            break Ok(None);
        }
        if let Ok(newer) = stations.try_recv() {
            break Ok(Some(newer));
        }
        let started = Instant::now();
        // While the robot is missing, keep the station and keep looking.
        if robot.try_open() && robot.lidar_up() {
            let polls = Polls {
                scan: tick % SCAN_EVERY_TICKS == 0,
                charger: tick % CHARGER_EVERY_TICKS == 0,
            };
            if let Err(e) = poll_once(robot, &mut stream, clock, &mut seq, polls) {
                break Err(e);
            }
            tick = tick.wrapping_add(1);
        }
        if let Some(rest) = TICK.checked_sub(started.elapsed()) {
            thread::sleep(rest);
        }
    };
    let _ = stream.shutdown(std::net::Shutdown::Both);
    let _ = reader.join();
    result
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
    scan: bool,
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
) -> Result<()> {
    let mut out = Vec::with_capacity(4);

    let t = stamp(clock);
    if let Some(reply) = robot.query(cmd::GET_MOTORS) {
        match neato::parse_motors(&reply) {
            Ok(w) => out.push(Stamped {
                t_ns: t,
                seq: next(&mut seq.wheels),
                msg: Msg::Wheels(w),
            }),
            Err(e) => log_bad_reply(cmd::GET_MOTORS, &e, &reply),
        }
    }

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

    if polls.scan {
        let t = stamp(clock);
        if let Some(reply) = robot.query(cmd::GET_LDS_SCAN) {
            match neato::parse_lds_scan(&reply) {
                Ok(s) => out.push(Stamped {
                    t_ns: t,
                    seq: next(&mut seq.lidar),
                    msg: Msg::Lidar(s),
                }),
                Err(e) => log_bad_reply(cmd::GET_LDS_SCAN, &e, &reply),
            }
        }
    }

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
        stream
            .write_all(&wire::encode_frame(m)?)
            .context("sending to the station")?;
    }
    Ok(())
}

fn stamp(clock: Instant) -> u64 {
    clock.elapsed().as_nanos() as u64
}

fn log_bad_reply(command: &str, err: &neato::ParseError, reply: &str) {
    let head: String = reply.chars().take(240).collect();
    eprintln!("could not parse {command} reply ({err}); it began: {head:?}");
}

/// Reads and drops commands from the station until it disconnects.
fn drain_commands(mut stream: TcpStream) -> io::Result<()> {
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
            Ok(Command::Stop) => {}
            Ok(c) => eprintln!("ignoring {c:?}: driving the real robot is disabled"),
            Err(e) => eprintln!("ignoring an undecodable command: {e}"),
        }
    }
}
