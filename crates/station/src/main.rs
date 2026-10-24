//! The control app. By default it serves a web page (see `http`) that shows
//! the simulator or the real robot and drives the sim from the keyboard, so
//! it can run in a headless sandbox and be viewed from any browser. With
//! `--headless` it runs a scripted drive and prints odometry against ground
//! truth.

mod http;
mod session;
mod teleop;

use std::sync::Arc;

use anyhow::{Context, Result, bail};
use robot::{Command, Msg, Pose2, RobotConfig, RobotIo};
use sim::{Sim, SimBackend, World};
use slam::odometry::WheelOdometry;
use tokio::sync::{mpsc, watch};

const ROBOT_TOML: &str = include_str!("../../../robot.toml");
const USAGE: &str = "usage: station [--listen 127.0.0.1:8080] | station --headless";

#[tokio::main]
async fn main() -> Result<()> {
    let config = RobotConfig::from_toml_str(ROBOT_TOML)?;
    let start = Pose2::new(1.0, 1.0, 0.0);
    let mut listen = String::from("127.0.0.1:8080");
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--headless" => return headless(config, start).await,
            "--listen" => listen = args.next().context(USAGE)?,
            _ => bail!("unknown argument {arg:?}\n{USAGE}"),
        }
    }

    let listener = tokio::net::TcpListener::bind(&listen)
        .await
        .with_context(|| format!("listening on {listen}"))?;
    println!(
        "station: open http://{} in a browser",
        listener.local_addr()?
    );
    let (controls_tx, controls) = mpsc::channel(64);
    let (frames_tx, frames) = watch::channel(Arc::<str>::from("{}"));
    tokio::spawn(session::run(
        session::Session::new(config, start),
        controls,
        frames_tx,
    ));
    http::serve(listener, controls_tx, frames).await
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
