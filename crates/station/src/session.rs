//! The one live robot source the web page is looking at, and the loop that
//! turns its messages into frames for the page.
//!
//! The sim is stepped in wall-clock time and driven by the page's keys. The
//! real robot is view-only: its messages are drawn, but this module never
//! sends it a command, so the page can't move the drive motors.

use std::sync::Arc;
use std::time::{Duration, Instant};

use robot::{
    Command, Contacts, LidarScan, Msg, Pose2, RealBackend, RobotConfig, RobotIo, RobotStatus,
    Stamped,
};
use serde::Deserialize;
use serde_json::{Value, json};
use sim::{Segment, Sim, SimBackend, World};
use slam::odometry::WheelOdometry;
use tokio::sync::{mpsc, watch};
use tokio::task::JoinHandle;

use crate::teleop::{self, Keys};

/// How often frames go out to the page.
const FRAME_PERIOD: Duration = Duration::from_millis(33);
/// Longest stretch the sim catches up on, so a stall doesn't make it jump.
const MAX_STEP: Duration = Duration::from_millis(100);
/// The page re-sends held keys every 200 ms. If it goes quiet for longer
/// than this (a closed tab, a dropped connection), the keys count as
/// released.
const KEYS_TIMEOUT: Duration = Duration::from_millis(500);
/// How long to wait for the Pi to accept a connection.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
/// A connected Pi that sends nothing for this long is reported as stalled.
/// It covers the 2 s lidar spin-up after connecting.
const STALL_AFTER: Duration = Duration::from_secs(4);

/// What the page can ask for.
#[derive(Debug, Deserialize)]
#[serde(tag = "op", rename_all = "lowercase")]
pub enum Control {
    /// The drive keys currently held. Only the sim acts on them.
    Keys(Keys),
    /// Switch to another source.
    Use { backend: Backend },
    /// Start the current source over: a fresh sim, or odometry back at the
    /// origin on the real robot.
    Reset,
    /// Freeze sim time.
    Pause { paused: bool },
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Backend {
    Sim,
    /// `pi-node` at a host:port, view-only.
    Real {
        addr: String,
    },
}

/// A room with a few obstacles, so scans have some structure to show.
pub fn demo_world() -> World {
    let mut world = World::rectangle(6.0, 4.5);
    let mut wall = |a: (f64, f64), b: (f64, f64)| world.segments.push(Segment { a, b });
    // An interior wall with a doorway.
    wall((3.5, 0.0), (3.5, 1.6));
    wall((3.5, 2.4), (3.5, 3.2));
    // A box, like a cabinet.
    let (x0, y0, x1, y1) = (1.2, 3.2, 2.2, 3.8);
    wall((x0, y0), (x1, y0));
    wall((x1, y0), (x1, y1));
    wall((x1, y1), (x0, y1));
    wall((x0, y1), (x0, y0));
    // An angled piece of furniture in the second room.
    wall((4.5, 3.6), (5.4, 2.9));
    world
}

/// What both sources feed: odometry and the latest readings.
struct Readings {
    odometry: WheelOdometry,
    /// Stamp of the latest message.
    t_ns: u64,
    /// Latest scan hits in the robot frame, metres.
    hits: Vec<[f64; 2]>,
    contacts: Contacts,
    status: Option<RobotStatus>,
}

impl Readings {
    fn new(config: &RobotConfig, start: Pose2) -> Self {
        Self {
            odometry: WheelOdometry::new(start, config.drive.wheel_base_m),
            t_ns: 0,
            hits: Vec::new(),
            contacts: Contacts::default(),
            status: None,
        }
    }

    fn update(&mut self, m: Stamped<Msg>, config: &RobotConfig) {
        self.t_ns = m.t_ns;
        match m.msg {
            Msg::Wheels(w) => {
                self.odometry.update(w);
            }
            Msg::Lidar(scan) => self.hits = robot_frame_hits(&scan, config),
            Msg::Contacts(c) => self.contacts = c,
            Msg::Status(s) => self.status = Some(s),
        }
    }
}

/// A scan's valid returns as points in the robot frame.
fn robot_frame_hits(scan: &LidarScan, config: &RobotConfig) -> Vec<[f64; 2]> {
    let lidar = &config.lidar;
    let mount = Pose2::new(lidar.x_m, lidar.y_m, lidar.theta_rad);
    scan.ranges
        .iter()
        .enumerate()
        .filter(|(_, r)| r.is_finite())
        .map(|(i, &r)| {
            let a = (scan.angle_min + i as f32 * scan.angle_increment) as f64;
            let (x, y) = mount.transform_point(r as f64 * a.cos(), r as f64 * a.sin());
            [x, y]
        })
        .collect()
}

struct SimRun {
    robot: SimBackend,
    readings: Readings,
    /// Where sim time should be by now. The sim steps in whole physics
    /// steps, so it runs up to this rather than a frame past its last
    /// stamp, which would round every frame up and run fast.
    clock_ns: u64,
    command: Command,
    paused: bool,
}

impl SimRun {
    fn new(config: &RobotConfig, start: Pose2) -> Self {
        Self {
            robot: SimBackend::new(Sim::new(demo_world(), config.clone(), start, 0.01)),
            readings: Readings::new(config, start),
            clock_ns: 0,
            command: Command::Stop,
            paused: false,
        }
    }

    /// Sends the command if it changed, then runs the sim `step_ns` further.
    async fn advance(
        &mut self,
        command: Command,
        step_ns: u64,
        config: &RobotConfig,
    ) -> std::io::Result<()> {
        if command != self.command {
            self.robot.send(command).await?;
            self.command = command;
        }
        self.clock_ns += step_ns;
        while self.readings.t_ns < self.clock_ns {
            let Some(m) = self.robot.recv().await? else {
                break;
            };
            self.readings.update(m, config);
        }
        Ok(())
    }
}

enum Link {
    Connecting,
    /// TCP is up. The instant is when data last arrived, or when the link
    /// came up if nothing has yet, so a link that carries no data can be told
    /// apart from a live one.
    Connected {
        last_data: Instant,
    },
    Closed(String),
}

/// What the reader task reports.
enum RealEvent {
    Connected,
    Msg(Stamped<Msg>),
    Closed(String),
}

struct RealRun {
    addr: String,
    link: Link,
    readings: Readings,
    events: mpsc::Receiver<RealEvent>,
    reader: JoinHandle<()>,
}

impl RealRun {
    /// Connects in the background. The reader task owns the backend and only
    /// ever calls `recv` on it.
    fn connect(addr: String, config: &RobotConfig) -> Self {
        let (tx, events) = mpsc::channel(256);
        let target = addr.clone();
        let reader = tokio::spawn(async move {
            let connect = tokio::time::timeout(CONNECT_TIMEOUT, RealBackend::connect(target));
            let mut robot = match connect.await {
                Ok(Ok(robot)) => robot,
                Ok(Err(e)) => {
                    return drop(
                        tx.send(RealEvent::Closed(format!("connect failed: {e}")))
                            .await,
                    );
                }
                Err(_) => {
                    return drop(tx.send(RealEvent::Closed("connect timed out".into())).await);
                }
            };
            let _ = tx.send(RealEvent::Connected).await;
            let closed = loop {
                match robot.recv().await {
                    Ok(Some(m)) => {
                        if tx.send(RealEvent::Msg(m)).await.is_err() {
                            return;
                        }
                    }
                    Ok(None) => break "pi-node closed the connection".to_owned(),
                    Err(e) => break format!("link error: {e}"),
                }
            };
            let _ = tx.send(RealEvent::Closed(closed)).await;
        });
        Self {
            addr,
            link: Link::Connecting,
            readings: Readings::new(config, Pose2::default()),
            events,
            reader,
        }
    }
}

impl Drop for RealRun {
    fn drop(&mut self) {
        self.reader.abort();
    }
}

enum Source {
    Sim(SimRun),
    Real(RealRun),
}

pub struct Session {
    config: RobotConfig,
    start: Pose2,
    source: Source,
    /// Bumped on every reset or switch, so the page knows to clear trails.
    run: u64,
    keys: Keys,
    keys_at: Instant,
}

impl Session {
    pub fn new(config: RobotConfig, start: Pose2) -> Self {
        Self {
            source: Source::Sim(SimRun::new(&config, start)),
            config,
            start,
            run: 0,
            keys: Keys::default(),
            keys_at: Instant::now(),
        }
    }

    fn control(&mut self, control: Control) {
        match control {
            Control::Keys(keys) => {
                self.keys = keys;
                self.keys_at = Instant::now();
            }
            Control::Use { backend } => {
                self.source = match backend {
                    Backend::Sim => Source::Sim(SimRun::new(&self.config, self.start)),
                    Backend::Real { addr } => {
                        Source::Real(RealRun::connect(addr.trim().to_owned(), &self.config))
                    }
                };
                self.run += 1;
            }
            Control::Reset => {
                match &mut self.source {
                    Source::Sim(sim) => *sim = SimRun::new(&self.config, self.start),
                    Source::Real(real) => {
                        real.readings = Readings::new(&self.config, Pose2::default())
                    }
                }
                self.run += 1;
            }
            Control::Pause { paused } => {
                if let Source::Sim(sim) = &mut self.source {
                    sim.paused = paused;
                }
            }
        }
    }

    fn held_keys(&self) -> Keys {
        if self.keys_at.elapsed() > KEYS_TIMEOUT {
            Keys::default()
        } else {
            self.keys
        }
    }

    async fn tick(&mut self, elapsed: Duration) -> std::io::Result<()> {
        let keys = self.held_keys();
        let config = &self.config;
        match &mut self.source {
            Source::Sim(sim) => {
                let (command, step) = if sim.paused {
                    (Command::Stop, Duration::ZERO)
                } else {
                    let max = config.drive.max_wheel_speed_mm_s;
                    (teleop::command(keys, max), elapsed.min(MAX_STEP))
                };
                sim.advance(command, step.as_nanos() as u64, config).await
            }
            Source::Real(real) => {
                while let Ok(event) = real.events.try_recv() {
                    match event {
                        RealEvent::Connected => {
                            real.link = Link::Connected {
                                last_data: Instant::now(),
                            }
                        }
                        RealEvent::Msg(m) => {
                            if let Link::Connected { last_data } = &mut real.link {
                                *last_data = Instant::now();
                            }
                            real.readings.update(m, config)
                        }
                        RealEvent::Closed(why) => real.link = Link::Closed(why),
                    }
                }
                Ok(())
            }
        }
    }

    fn frame(&self) -> Value {
        let pose = |p: Pose2| json!([round(p.x), round(p.y), round(p.theta)]);
        let (readings, mut frame) = match &self.source {
            Source::Sim(sim) => {
                let walls: Vec<_> = sim
                    .robot
                    .sim()
                    .world
                    .segments
                    .iter()
                    .map(|s| [s.a.0, s.a.1, s.b.0, s.b.1])
                    .collect();
                let frame = json!({
                    "backend": "sim",
                    "state": if sim.paused { "paused" } else { "running" },
                    "command": describe(sim.command),
                    "truth": pose(sim.robot.ground_truth().expect("the sim always knows its pose")),
                    "walls": walls,
                });
                (&sim.readings, frame)
            }
            Source::Real(real) => {
                let (state, detail) = match &real.link {
                    Link::Connecting => ("connecting", String::new()),
                    Link::Connected { last_data } if last_data.elapsed() >= STALL_AFTER => (
                        "stalled",
                        format!(
                            "connected, but no data for {:.0} s",
                            last_data.elapsed().as_secs_f64()
                        ),
                    ),
                    Link::Connected { .. } => ("connected", String::new()),
                    Link::Closed(why) => ("disconnected", why.clone()),
                };
                let frame = json!({
                    "backend": "real",
                    "addr": real.addr,
                    "state": state,
                    "detail": detail,
                    "command": "view only",
                    "truth": null,
                    "walls": [],
                });
                (&real.readings, frame)
            }
        };
        let hits: Vec<_> = readings
            .hits
            .iter()
            .map(|&[x, y]| [round(x), round(y)])
            .collect();
        let extra = json!({
            "run": self.run,
            "t": readings.t_ns as f64 * 1e-9,
            "odometry": pose(readings.odometry.pose),
            "hits": hits,
            "bumpers": readings.contacts.bumpers,
            "wheel_drop": readings.contacts.wheel_drop,
            "battery": readings.status.map(|s| json!({
                "v": s.battery_v,
                "percent": s.fuel_percent,
                "charging": s.charging,
                "external_power": s.external_power,
            })),
        });
        if let (Value::Object(frame), Value::Object(extra)) = (&mut frame, extra) {
            frame.extend(extra);
        }
        frame
    }
}

/// Millimetre precision is plenty for drawing and keeps frames small.
fn round(v: f64) -> f64 {
    (v * 1000.0).round() / 1000.0
}

fn describe(command: Command) -> String {
    match command {
        Command::WheelSpeeds {
            left_mm_s,
            right_mm_s,
        } => {
            format!("wheels L {left_mm_s:+.0} R {right_mm_s:+.0} mm/s")
        }
        Command::Stop => "stopped".to_owned(),
    }
}

/// Applies controls from the page and publishes a frame every tick, until
/// every sender of controls is gone.
pub async fn run(
    mut session: Session,
    mut controls: mpsc::Receiver<Control>,
    frames: watch::Sender<Arc<str>>,
) {
    let mut ticker = tokio::time::interval(FRAME_PERIOD);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut last = Instant::now();
    loop {
        tokio::select! {
            control = controls.recv() => match control {
                Some(control) => session.control(control),
                None => return,
            },
            _ = ticker.tick() => {
                let now = Instant::now();
                if let Err(e) = session.tick(now - last).await {
                    eprintln!("robot error: {e}");
                }
                last = now;
                frames.send_replace(session.frame().to_string().into());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> RobotConfig {
        RobotConfig::from_toml_str(include_str!("../../../robot.toml")).unwrap()
    }

    #[test]
    fn controls_parse_from_page_json() {
        let keys: Control = serde_json::from_str(
            r#"{"op":"keys","forward":true,"back":false,"left":false,"right":false,"fast":false}"#,
        )
        .unwrap();
        assert!(matches!(keys, Control::Keys(Keys { forward: true, .. })));
        let real: Control = serde_json::from_str(
            r#"{"op":"use","backend":{"kind":"real","addr":"10.0.0.5:7878"}}"#,
        )
        .unwrap();
        assert!(matches!(
            real,
            Control::Use {
                backend: Backend::Real { .. }
            }
        ));
    }

    #[tokio::test]
    async fn sim_drives_from_keys_and_stops_when_they_go_stale() {
        let mut session = Session::new(config(), Pose2::new(1.0, 1.0, 0.0));
        session.control(Control::Keys(Keys {
            forward: true,
            ..Keys::default()
        }));
        session.tick(Duration::from_millis(100)).await.unwrap();
        session.tick(Duration::from_millis(100)).await.unwrap();
        let Source::Sim(sim) = &session.source else {
            unreachable!()
        };
        let moved = sim.robot.sim().ground_truth().x - 1.0;
        assert!(moved > 0.01, "moved {moved}");

        session.keys_at -= KEYS_TIMEOUT * 2;
        session.tick(Duration::from_millis(100)).await.unwrap();
        let Source::Sim(sim) = &session.source else {
            unreachable!()
        };
        assert_eq!(sim.command, Command::Stop);
    }

    #[tokio::test]
    async fn real_robot_never_receives_commands() {
        use robot::wire;
        use tokio::io::{AsyncReadExt, AsyncWriteExt};
        use tokio::net::TcpListener;

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let mut session = Session::new(config(), Pose2::default());
        session.control(Control::Use {
            backend: Backend::Real { addr },
        });
        let (mut pi, _) = listener.accept().await.unwrap();
        pi.write_all(
            &wire::encode_frame(&Stamped {
                t_ns: 5,
                seq: 0,
                msg: Msg::Wheels(robot::WheelPositions {
                    left_mm: 0,
                    right_mm: 0,
                }),
            })
            .unwrap(),
        )
        .await
        .unwrap();

        session.control(Control::Keys(Keys {
            forward: true,
            fast: true,
            ..Keys::default()
        }));
        for _ in 0..10 {
            session.tick(FRAME_PERIOD).await.unwrap();
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let frame = session.frame();
        assert_eq!(frame["state"], "connected");
        assert_eq!(frame["t"], 5e-9);

        // Hanging up must leave the Pi with nothing read.
        session.control(Control::Use {
            backend: Backend::Sim,
        });
        let mut buf = Vec::new();
        let n = tokio::time::timeout(Duration::from_secs(1), pi.read_to_end(&mut buf))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(n, 0, "station sent {buf:?}");
    }
}
