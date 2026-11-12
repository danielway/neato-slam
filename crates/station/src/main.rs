//! The control app. By default it serves a web page (see `http`) that shows
//! the simulator or the real robot and drives the sim from the keyboard, so
//! it can run in a headless sandbox and be viewed from any browser. With
//! `--headless` it runs a scripted drive and prints odometry against ground
//! truth, and with `--replay` it does the same over a recorded log. With
//! `--camera` it also tracks the real robot with the ceiling camera (see
//! `camera`), which needs the station started from the repo folder.

mod camera;
mod drive_test;
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
const USAGE: &str =
    "usage: station [--listen 127.0.0.1:8080] [--recordings DIR] [--camera /dev/video0]
       station --headless
       station --replay FILE.mcap
       station --csv FILE.mcap > run.csv
       station --headings FILE.mcap > headings.csv";

#[tokio::main]
async fn main() -> Result<()> {
    let config = RobotConfig::from_toml_str(ROBOT_TOML)?;
    let start = Pose2::new(1.0, 1.0, 0.0);
    let mut listen = String::from("127.0.0.1:8080");
    let mut recordings = PathBuf::from("recordings");
    let mut camera = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--headless" => return headless(config, start).await,
            "--replay" => return replay(config, args.next().context(USAGE)?.into()).await,
            "--csv" => return csv(args.next().context(USAGE)?.into()),
            "--headings" => return headings(config, args.next().context(USAGE)?.into()),
            "--listen" => listen = args.next().context(USAGE)?,
            "--recordings" => recordings = args.next().context(USAGE)?.into(),
            "--camera" => camera = Some(args.next().context(USAGE)?),
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
    let mut session = session::Session::new(config, start, recordings.clone());
    let mut topdown = None;
    if let Some(device) = camera {
        let arena = arena_corners("arena.toml")?;
        let path =
            std::env::temp_dir().join(format!("neato-station-{}-floor.jpg", std::process::id()));
        println!("station: tracking the robot with the camera at {device}");
        session = session.with_camera(camera::CameraFeed::spawn(device, path.clone()), arena);
        topdown = Some(path);
    }
    let (controls_tx, controls) = mpsc::channel(64);
    let (frames_tx, frames) = watch::channel(Arc::<str>::from("{}"));
    tokio::spawn(session::run(session, controls, frames_tx));
    http::serve(listener, controls_tx, frames, recordings, topdown).await
}

/// Prints a recording's wheels, commands and camera poses as CSV, one row
/// per entry in source time, for plotting elsewhere: `t_s,kind,a,b,c` where
/// wheels are left/right mm, a command is left/right mm/s (a stop is 0,0),
/// and truth is x/y metres and heading radians.
fn csv(path: PathBuf) -> Result<()> {
    let recording =
        Recording::read_file(&path).with_context(|| format!("reading {}", path.display()))?;
    let t0 = recording.span_ns().map_or(0, |(a, _)| a);
    let mut entries: Vec<&Entry> = recording.entries.iter().collect();
    entries.sort_by_key(|e| e.t_ns());
    println!("t_s,kind,a,b,c");
    for entry in entries {
        let t = entry.t_ns().saturating_sub(t0) as f64 * 1e-9;
        match entry {
            Entry::Msg(robot::Stamped {
                msg: Msg::Wheels(w),
                ..
            }) => println!("{t:.4},wheels,{},{},", w.left_mm, w.right_mm),
            Entry::Command { command, .. } => match command {
                Command::WheelSpeeds {
                    left_mm_s,
                    right_mm_s,
                } => println!("{t:.4},command,{left_mm_s},{right_mm_s},"),
                Command::Stop => println!("{t:.4},command,0,0,"),
                _ => {}
            },
            Entry::Truth { pose, .. } => {
                println!("{t:.4},truth,{:.4},{:.4},{:.4}", pose.x, pose.y, pose.theta)
            }
            _ => {}
        }
    }
    Ok(())
}

/// Heading three ways, for checking lidar timing: from the camera, from
/// wheel odometry, and from the lidar alone, by turning each scan until it
/// best matches the first one. Only meaningful while the robot turns on the
/// spot. Prints CSV `t_s,source,heading_rad` with every heading unwrapped
/// and measured from its own value at the first scan.
fn headings(config: RobotConfig, path: PathBuf) -> Result<()> {
    let recording =
        Recording::read_file(&path).with_context(|| format!("reading {}", path.display()))?;
    let t0 = recording.span_ns().map_or(0, |(a, _)| a);
    let mut entries: Vec<&Entry> = recording.entries.iter().collect();
    entries.sort_by_key(|e| e.t_ns());
    let mut odometry = WheelOdometry::new(Pose2::default(), config.drive.wheel_base_m);
    let mut reference: Option<robot::LidarScan> = None;
    let (mut lidar, mut camera, mut wheels) =
        (Unwrap::default(), Unwrap::default(), Unwrap::default());
    println!("t_s,source,heading_rad");
    for entry in entries {
        let t = entry.t_ns().saturating_sub(t0) as f64 * 1e-9;
        match entry {
            Entry::Msg(robot::Stamped {
                msg: Msg::Wheels(w),
                ..
            }) => {
                let theta = odometry.update(*w).theta;
                println!("{t:.4},wheels,{:.4}", wheels.next(theta));
            }
            Entry::Msg(robot::Stamped {
                msg: Msg::Lidar(scan),
                ..
            }) => match &reference {
                None => {
                    reference = Some(scan.clone());
                    println!("{t:.4},lidar,{:.4}", lidar.next(0.0));
                }
                Some(first) => {
                    if let Some(turn) = best_turn(first, scan, &config.lidar) {
                        println!("{t:.4},lidar,{:.4}", lidar.next(turn));
                    }
                }
            },
            Entry::Truth { pose, .. } => println!("{t:.4},camera,{:.4}", camera.next(pose.theta)),
            _ => {}
        }
    }
    Ok(())
}

/// Makes an angle series continuous and starts it at zero.
#[derive(Default)]
struct Unwrap {
    first: Option<f64>,
    last: f64,
    total: f64,
}

impl Unwrap {
    fn next(&mut self, theta: f64) -> f64 {
        match self.first {
            None => {
                self.first = Some(theta);
                self.last = theta;
            }
            Some(_) => {
                let d = theta - self.last;
                self.total += d.sin().atan2(d.cos());
                self.last = theta;
            }
        }
        self.total
    }
}

/// How far the robot has turned on the spot since `first`: the turn about
/// the robot's centre that best carries the points `first` saw onto what
/// `scan` sees. The lidar sits off the centre, so a turn also moves it and
/// a plain shift of the beams would be biased. Searched in 1 degree steps,
/// then 0.05 degree steps around the best.
fn best_turn(
    first: &robot::LidarScan,
    scan: &robot::LidarScan,
    mount: &robot::LidarConfig,
) -> Option<f64> {
    let lidar = Pose2::new(mount.x_m, mount.y_m, mount.theta_rad);
    let points: Vec<(f64, f64)> = first
        .ranges
        .iter()
        .enumerate()
        .filter(|(_, r)| r.is_finite())
        .map(|(i, &r)| {
            let a = (first.angle_min + i as f32 * first.angle_increment) as f64;
            lidar.transform_point(r as f64 * a.cos(), r as f64 * a.sin())
        })
        .collect();
    let n = scan.ranges.len();
    if points.len() < 30 || n == 0 {
        return None;
    }
    let inc = scan.angle_increment as f64;
    let cost = |turn: f64| -> Option<f64> {
        let (s, c) = (-turn).sin_cos();
        let (mut sum, mut count) = (0.0, 0usize);
        for &(x, y) in &points {
            // Where the point sits after the robot turned, seen from the
            // lidar.
            let (rx, ry) = (c * x - s * y - lidar.x, s * x + c * y - lidar.y);
            let a = ry.atan2(rx) - lidar.theta - scan.angle_min as f64;
            let k = (a / inc).round().rem_euclid(n as f64) as usize % n;
            let r = scan.ranges[k];
            if r.is_finite() {
                sum += (r as f64 - rx.hypot(ry)).abs().min(0.3);
                count += 1;
            }
        }
        (count > points.len() / 3).then(|| sum / count as f64)
    };
    let search = |from: f64, to: f64, step: f64| {
        let steps = ((to - from) / step).round() as usize;
        (0..=steps)
            .map(|i| from + i as f64 * step)
            .filter_map(|t| cost(t).map(|c| (t, c)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
    };
    let deg = std::f64::consts::PI / 180.0;
    let (coarse, _) = search(-180.0 * deg, 179.0 * deg, deg)?;
    search(coarse - deg, coarse + deg, 0.05 * deg).map(|(t, _)| t)
}

/// The taped arena's corners from `arena.toml`, clockwise from top left.
fn arena_corners(path: &str) -> Result<Vec<[f64; 2]>> {
    #[derive(serde::Deserialize)]
    struct Arena {
        corners: std::collections::BTreeMap<String, [f64; 2]>,
    }
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {path}"))?;
    let arena: Arena = toml::from_str(&text).with_context(|| format!("parsing {path}"))?;
    ["top_left", "top_right", "bottom_right", "bottom_left"]
        .iter()
        .map(|k| {
            arena
                .corners
                .get(*k)
                .copied()
                .with_context(|| format!("{path} has no corners.{k}"))
        })
        .collect()
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
            Entry::Msg(m) => match &m.msg {
                Msg::Lidar(_) => (0, Some(m.seq)),
                Msg::Wheels(_) => (1, Some(m.seq)),
                Msg::Contacts(_) => (2, Some(m.seq)),
                Msg::Status(_) => (3, Some(m.seq)),
                Msg::Node(n) => {
                    println!("  pi-node: {} {}", n.version, n.settings);
                    continue;
                }
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

    // Odometry starts at the camera's first sighting, which can come
    // seconds after the robot set off (the marker is hard to read at the
    // edge of the view), so wheel travel before it is left out. It is scored
    // at the last sighting, not at the end, since the camera may have lost
    // the robot by then.
    let mut odometry: Option<WheelOdometry> = None;
    let mut last_truth = None;
    let (mut est_trail, mut truth_trail) = (Vec::new(), Vec::new());
    for entry in &recording.entries {
        match entry {
            Entry::Truth { t_ns, pose } => {
                let odo = odometry.get_or_insert_with(|| {
                    est_trail.push((*t_ns, *pose));
                    WheelOdometry::new(*pose, config.drive.wheel_base_m)
                });
                last_truth = Some((*pose, odo.pose));
                truth_trail.push((*t_ns, *pose));
            }
            Entry::Msg(robot::Stamped {
                t_ns,
                msg: Msg::Wheels(w),
                ..
            }) => {
                if let Some(odo) = &mut odometry {
                    est_trail.push((*t_ns, odo.update(*w)));
                }
            }
            _ => {}
        }
    }
    if odometry.is_none() {
        // No truth: odometry from the origin over the whole log.
        let mut odo = WheelOdometry::new(Pose2::default(), config.drive.wheel_base_m);
        let mut robot = ReplayBackend::from_recording(recording);
        while let Some(m) = robot.recv().await? {
            if let Msg::Wheels(w) = m.msg {
                odo.update(w);
            }
        }
        odometry = Some(odo);
    }
    let est = odometry.expect("set above").pose;
    println!(
        "  odometry ends at x={:.3} y={:.3} theta={:.3}",
        est.x, est.y, est.theta
    );
    if let Some((truth, est)) = last_truth {
        let heading = (est.theta - truth.theta)
            .sin()
            .atan2((est.theta - truth.theta).cos());
        println!(
            "  at the last camera pose x={:.3} y={:.3} theta={:.3}, odometry is off by {:.4} m and {:.1} deg",
            truth.x,
            truth.y,
            truth.theta,
            (est.x - truth.x).hypot(est.y - truth.y),
            heading.to_degrees()
        );
        score(&est_trail, &truth_trail);
    }
    Ok(())
}

/// Grades the odometry trail against the camera's: ATE as recorded (both
/// start at the first sighting), ATE after the best rigid alignment (shape
/// error only), and RPE (drift per stretch of true travel).
fn score(est: &[(u64, Pose2)], truth: &[(u64, Pose2)]) {
    use slam::eval;
    // Wheel stamps come every 100 ms or so; a longer hole means the stream
    // dropped and interpolating across it would invent a path.
    let mut est = est.to_vec();
    est.sort_by_key(|e| e.0);
    let pairs = eval::associate(&est, truth, 300_000_000);
    let Some(raw) = eval::ate(&pairs, Pose2::default()) else {
        println!("  no camera poses overlap the odometry, so nothing to score");
        return;
    };
    let line = |name: &str, a: &eval::Ate| {
        println!(
            "  ATE {name}: rmse {:.3} m, max {:.3} m, heading rmse {:.1} deg over {} poses",
            a.rmse_m,
            a.max_m,
            a.heading_rmse_rad.to_degrees(),
            a.pairs
        )
    };
    line("as recorded", &raw);
    if let Some(aligned) = eval::align(&pairs).and_then(|m| eval::ate(&pairs, m)) {
        line("aligned", &aligned);
    }
    for over in [0.5, 1.0] {
        if let Some(r) = eval::rpe(&pairs, over) {
            println!(
                "  RPE per {:.1} m: rmse {:.3} m ({:.1}%), heading rmse {:.1} deg over {} stretches",
                r.over_m,
                r.rmse_m,
                100.0 * r.rmse_m / r.over_m,
                r.heading_rmse_rad.to_degrees(),
                r.stretches
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A scan of an off-centre box from a robot at `pose`, lidar mounted
    /// behind the centre as on the Neato.
    fn scan_at(world: &World, mount: &robot::LidarConfig, pose: Pose2) -> robot::LidarScan {
        let lidar = pose.compose(&Pose2::new(mount.x_m, mount.y_m, mount.theta_rad));
        let inc = std::f32::consts::TAU / 360.0;
        let ranges = (0..360)
            .map(|i| {
                let a = lidar.theta + (i as f32 * inc) as f64;
                world
                    .raycast((lidar.x, lidar.y), a, 6.0)
                    .map_or(f32::NAN, |r| r as f32)
            })
            .collect();
        robot::LidarScan {
            angle_min: 0.0,
            angle_increment: inc,
            ranges,
            intensities: None,
            scan_duration: 0.2,
        }
    }

    #[test]
    fn best_turn_recovers_a_turn_on_the_spot() {
        let config = RobotConfig::from_toml_str(ROBOT_TOML).unwrap();
        let world = World::rectangle(3.6, 1.8);
        let start = Pose2::new(2.1, 0.7, 0.3);
        let first = scan_at(&world, &config.lidar, start);
        for turn in [0.2_f64, 1.0, 2.4, -1.3] {
            let now = Pose2::new(start.x, start.y, start.theta + turn);
            let scan = scan_at(&world, &config.lidar, now);
            let got = best_turn(&first, &scan, &config.lidar).unwrap();
            let err = (got - turn).sin().atan2((got - turn).cos());
            assert!(err.abs() < 0.01, "turn {turn}: got {got}");
        }
    }
}
