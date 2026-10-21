//! The desktop control app. For now it runs a scripted drive in the
//! simulator and compares wheel odometry with ground truth, end to end
//! through `RobotIo`. Teleop, recording, a real-robot mode and a Rerun view
//! come next.

use anyhow::Result;
use robot_io::{RobotIo, SimBackend};
use robot_msgs::{Command, Msg, Pose2, RobotConfig};
use sim::{Sim, World};
use slam::odometry::WheelOdometry;

const ROBOT_TOML: &str = include_str!("../../../robot.toml");

#[tokio::main]
async fn main() -> Result<()> {
    let config = RobotConfig::from_toml_str(ROBOT_TOML)?;
    let start = Pose2::new(1.0, 1.0, 0.0);
    let mut odometry = WheelOdometry::new(start, config.drive.wheel_base_m);
    let mut robot = SimBackend::new(Sim::new(World::rectangle(5.0, 4.0), config, start, 0.01));

    // Drive a gentle left arc for ten simulated seconds.
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
