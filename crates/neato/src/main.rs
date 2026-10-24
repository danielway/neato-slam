//! Runs on the Raspberry Pi and bridges the Neato to the desktop.
//!
//! It opens the Neato's USB serial port, waits for one desktop connection at
//! a time, and while one is connected spins the lidar and streams wheel
//! positions, bumpers and scans to it, each stamped with the Pi's monotonic
//! clock when it was read.
//!
//! Read-only: it never sends the Neato a drive command. Commands that
//! arrive from the desktop are logged and dropped. Driving will need a
//! deliberate change here, with the 300 ms stop watchdog from the design.

mod port;

use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use neato::cmd;
use robot::wire::{self, MAX_FRAME_LEN};
use robot::{Command, Msg, Stamped};

use crate::port::NeatoPort;

const USAGE: &str = "usage: pi-node [--serial /dev/ttyACM0] [--listen 0.0.0.0:7878]";

/// Wheels and bumpers are polled every tick, the lidar every other tick.
const TICK: Duration = Duration::from_millis(100);
const SCAN_EVERY_TICKS: u32 = 2;
/// Time for the lidar to reach speed after it is switched on.
const LDS_SPIN_UP: Duration = Duration::from_secs(2);

fn main() -> Result<()> {
    let mut serial = String::from("/dev/ttyACM0");
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

    let mut neato = NeatoPort::open(&serial)?;
    let version = neato.query(cmd::GET_VERSION)?;
    eprintln!("connected to the Neato on {serial}");
    for line in version.lines().filter(|l| !l.trim().is_empty()).take(12) {
        eprintln!("  {}", line.trim());
    }
    neato.query(cmd::TEST_MODE_ON)?;

    let listener = TcpListener::bind(&listen).with_context(|| format!("listening on {listen}"))?;
    eprintln!("waiting for the station on {listen}");
    let clock = Instant::now();
    for stream in listener.incoming() {
        let stream = match stream {
            Ok(s) => s,
            Err(e) => {
                eprintln!("accept failed: {e}");
                continue;
            }
        };
        let peer = stream
            .peer_addr()
            .map(|a| a.to_string())
            .unwrap_or_default();
        eprintln!("station connected from {peer}");
        let result = serve(&mut neato, stream, clock);
        // Stop the lidar whenever nobody is watching.
        if let Err(e) = neato.query(cmd::LDS_OFF) {
            eprintln!("could not stop the lidar: {e}");
        }
        match result {
            Ok(()) => eprintln!("station {peer} disconnected"),
            Err(e) => eprintln!("station {peer} dropped: {e:#}"),
        }
    }
    Ok(())
}

/// Streams readings to one station until it disconnects.
fn serve(neato: &mut NeatoPort, mut stream: TcpStream, clock: Instant) -> Result<()> {
    stream.set_nodelay(true)?;
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

    neato.query(cmd::LDS_ON)?;
    thread::sleep(LDS_SPIN_UP);

    let mut seq = Seqs::default();
    let mut tick: u32 = 0;
    let result = loop {
        if !connected.load(Ordering::Relaxed) {
            break Ok(());
        }
        let started = Instant::now();
        if let Err(e) = poll_once(
            neato,
            &mut stream,
            clock,
            &mut seq,
            tick % SCAN_EVERY_TICKS == 0,
        ) {
            break Err(e);
        }
        tick = tick.wrapping_add(1);
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
}

fn next(seq: &mut u32) -> u32 {
    let s = *seq;
    *seq = s.wrapping_add(1);
    s
}

/// One round of queries. A reply that fails to parse is logged and skipped,
/// so one odd reply doesn't drop the connection; a dead serial port or
/// socket ends it.
fn poll_once(
    neato: &mut NeatoPort,
    stream: &mut TcpStream,
    clock: Instant,
    seq: &mut Seqs,
    with_scan: bool,
) -> Result<()> {
    let mut out = Vec::with_capacity(3);

    let t = stamp(clock);
    let reply = neato.query(cmd::GET_MOTORS)?;
    match neato::parse_motors(&reply) {
        Ok(w) => out.push(Stamped {
            t_ns: t,
            seq: next(&mut seq.wheels),
            msg: Msg::Wheels(w),
        }),
        Err(e) => log_bad_reply(cmd::GET_MOTORS, &e, &reply),
    }

    let t = stamp(clock);
    let reply = neato.query(cmd::GET_DIGITAL_SENSORS)?;
    match neato::parse_digital_sensors(&reply) {
        Ok(c) => out.push(Stamped {
            t_ns: t,
            seq: next(&mut seq.contacts),
            msg: Msg::Contacts(c),
        }),
        Err(e) => log_bad_reply(cmd::GET_DIGITAL_SENSORS, &e, &reply),
    }

    if with_scan {
        let t = stamp(clock);
        let reply = neato.query(cmd::GET_LDS_SCAN)?;
        match neato::parse_lds_scan(&reply) {
            Ok(s) => out.push(Stamped {
                t_ns: t,
                seq: next(&mut seq.lidar),
                msg: Msg::Lidar(s),
            }),
            Err(e) => log_bad_reply(cmd::GET_LDS_SCAN, &e, &reply),
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
