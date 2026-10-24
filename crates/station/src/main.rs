//! The control app. By default it serves a web page (see `http`) that shows
//! the simulator or the real robot and drives the sim from the keyboard, so
//! it can run in a headless sandbox and be viewed from any browser. With
//! `--headless` it runs a scripted drive and prints odometry against ground
//! truth, and with `--replay` it does the same over a recorded log.

mod http;
mod recorder;
mod session;
mod teleop;

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, Result, bail};
use robot::log::{Entry, Recording};
use robot::{Command, Msg, Pose2, ReplayBackend, RobotConfig, RobotIo};
use sim::{Sim, SimBackend, World};
use slam::odometry::WheelOdometry;
use tokio::sync::{mpsc, watch};

const ROBOT_TOML: &str = include_str!("../../../robot.toml");
const USAGE: &str = "usage: station [--listen 127.0.0.1:8080] [--recordings DIR]
       station --headless
       station --replay FILE.mcap";

#[tokio::main]
async fn main() -> Result<()> {
    let config = RobotConfig::from_toml_str(ROBOT_TOML)?;
    let start = Pose2::new(1.0, 1.0, 0.0);
    let mut listen = String::from("127.0.0.1:8080");
    let mut recordings = PathBuf::from("recordings");
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--headless" => return headless(config, start).await,
            "--replay" => return replay(config, args.next().context(USAGE)?.into()).await,
            "--listen" => listen = args.next().context(USAGE)?,
            "--recordings" => recordings = args.next().context(USAGE)?.into(),
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(());
            }
            _ => bail!("unknown argument {arg:?}\n{USAGE}"),
        }
    }

    let listener = tokio::net::TcpListener::bind(&listen)
        .await
        .with_context(|| format!("listening on {listen}"))?;
    println!(
        "station: open http://{} in a browser; recordings go in {}",
        listener.local_addr()?,
        recordings.display()
    );
    let (controls_tx, controls) = mpsc::channel(64);
    let (frames_tx, frames) = watch::channel(Arc::<str>::from("{}"));
    tokio::spawn(session::run(
        session::Session::new(config, start, recordings.clone()),
        controls,
        frames_tx,
    ));
    http::serve(listener, controls_tx, frames, recordings).await
}

/// Drives a gentle left arc for ten simulated seconds and reports how far
/// odometry drifted from the truth.
async fn headless(config: RobotConfig, start: Pose2) -> Result<()> {
    let mut odometry = WheelOdometry::new(start, config.drive.wheel_base_m);
    let mut robot = SimBackend::new(Sim::new(World::rectangle(5.0, 4.0), config, start, 0.01));

    robot
        .send(Command::WheelSpeeds {
            left_mm_s: 150.0,
            right_mm_s: 200.0,
        })
        .await?;
    let mut scans = 0;
    while let Some(m) = robot.recv().await? {
        match m.msg {
            Msg::Wheels(w) => {
                odometry.update(w);
            }
            Msg::Lidar(_) => scans += 1,
            _ => {}
        }
        if m.t_ns >= 10_000_000_000 {
            break;
        }
    }

    let truth = robot.ground_truth().expect("the sim always knows its pose");
    let est = odometry.pose;
    println!("after 10 s and {scans} scans:");
    println!(
        "  truth    x={:.3} y={:.3} theta={:.3}",
        truth.x, truth.y, truth.theta
    );
    println!(
        "  odometry x={:.3} y={:.3} theta={:.3}",
        est.x, est.y, est.theta
    );
    println!(
        "  error    {:.4} m",
        (est.x - truth.x).hypot(est.y - truth.y)
    );
    Ok(())
}

/// Summarises a recorded log and runs wheel odometry over it through the
/// replay backend, against the recorded truth when there is one.
async fn replay(config: RobotConfig, path: PathBuf) -> Result<()> {
    let recording =
        Recording::read_file(&path).with_context(|| format!("reading {}", path.display()))?;
    println!("{}", path.display());
    for (key, value) in &recording.info {
        if key != robot::log::ROBOT_TOML && key != "world" {
            println!("  {key}: {value}");
        }
    }
    if recording.truncated {
        println!("  cut short: the recorder stopped without closing the file");
    }
    let (mut counts, mut gaps) = ([0u64; 6], [0u64; 4]);
    let mut last_seq: [Option<u32>; 4] = [None; 4];
    for entry in &recording.entries {
        let (i, seq) = match entry {
            Entry::Msg(m) => match m.msg {
                Msg::Lidar(_) => (0, Some(m.seq)),
                Msg::Wheels(_) => (1, Some(m.seq)),
                Msg::Contacts(_) => (2, Some(m.seq)),
                Msg::Status(_) => (3, Some(m.seq)),
            },
            Entry::Command { .. } => (4, None),
            Entry::Truth { .. } => (5, None),
        };
        counts[i] += 1;
        if let Some(seq) = seq {
            // A jump in a stream's counter means messages went missing; a
            // restart (pi-node reconnecting) begins again from zero.
            if let Some(last) = last_seq[i]
                && seq != last.wrapping_add(1)
                && seq != 0
            {
                gaps[i] += 1;
            }
            last_seq[i] = Some(seq);
        }
    }
    let span = recording
        .span_ns()
        .map_or(0.0, |(a, b)| b.saturating_sub(a) as f64 * 1e-9);
    println!(
        "  {span:.1} s of source time, {} entries",
        recording.entries.len()
    );
    let names = ["lidar", "wheels", "contacts", "status", "commands", "truth"];
    for (i, name) in names.iter().enumerate() {
        if counts[i] > 0 {
            let gap = match gaps.get(i) {
                Some(&g) if g > 0 => format!(", {g} gaps"),
                _ => String::new(),
            };
            println!("  {name:>9}: {}{gap}", counts[i]);
        }
    }

    let start = recording
        .entries
        .iter()
        .find_map(|e| match e {
            Entry::Truth { pose, .. } => Some(*pose),
            _ => None,
        })
        .unwrap_or_default();
    let mut odometry = WheelOdometry::new(start, config.drive.wheel_base_m);
    let mut robot = ReplayBackend::from_recording(recording);
    while let Some(m) = robot.recv().await? {
        if let Msg::Wheels(w) = m.msg {
            odometry.update(w);
        }
    }
    let est = odometry.pose;
    println!(
        "  odometry ends at x={:.3} y={:.3} theta={:.3}",
        est.x, est.y, est.theta
    );
    if let Some(truth) = robot.ground_truth() {
        println!(
            "  truth    ends at x={:.3} y={:.3} theta={:.3}, error {:.4} m",
            truth.x,
            truth.y,
            truth.theta,
            (est.x - truth.x).hypot(est.y - truth.y)
        );
    }
    Ok(())
}
