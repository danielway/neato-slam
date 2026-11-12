//! The one live robot source the web page is looking at, and the loop that
//! turns its messages into frames for the page.
//!
//! The sim is stepped in wall-clock time and driven by the page's keys. The
//! real robot is driven by the same keys: held keys are re-sent to `pi-node`
//! every `RESEND_EVERY`, and `pi-node` stops the wheels if they stop coming.
//! A replay plays a recorded log at its original pace, or faster.
//!
//! The sim or the real robot can be recorded while it is shown: everything
//! it reports goes into a log in the recordings folder (see `recorder`), and
//! for the sim, the commands it was given and its true pose too. With the
//! ceiling camera running (see `camera`), the real robot has a true pose as
//! well, and its recordings carry it.

mod frame;
mod readings;
mod real;
mod replay;
mod sim;
mod tape;

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use robot::log::{self, Entry, Recording};
use robot::{Command, Pose2, RobotConfig};
use serde::Deserialize;
use serde_json::json;
use slam::odometry::WheelOdometry;
use tokio::sync::{mpsc, watch};

use crate::camera::CameraFeed;
use crate::drive_test;
use crate::recorder::{self, Recorder};
use crate::teleop::{self, Keys};

use readings::Readings;
use real::{Link, RETRY_AFTER, RealEvent, RealRun};
use replay::ReplayRun;
use sim::SimRun;
pub use sim::demo_world;
use tape::Tape;

/// How often frames go out to the page.
const FRAME_PERIOD: Duration = Duration::from_millis(33);
/// Longest stretch the sim catches up on, so a stall doesn't make it jump.
const MAX_STEP: Duration = Duration::from_millis(100);
/// The page re-sends held keys every 200 ms. If it goes quiet for longer
/// than this (a closed tab, a dropped connection), the keys count as
/// released.
const KEYS_TIMEOUT: Duration = Duration::from_millis(500);
const SPEEDS: std::ops::RangeInclusive<f64> = 0.1..=20.0;

/// What the page can ask for.
#[derive(Debug, Deserialize)]
#[serde(tag = "op", rename_all = "lowercase")]
pub enum Control {
    /// The drive keys currently held. They drive the sim or the real robot.
    Keys(Keys),
    /// Switch to another source.
    Use { backend: Backend },
    /// Start the current source over: a fresh sim, odometry back at the
    /// origin on the real robot, or a replay from the top.
    Reset,
    /// Freeze sim or replay time.
    Pause { paused: bool },
    /// Start or stop recording the sim or the real robot.
    Record { on: bool },
    /// Replay speed, as a multiple of the recorded pace.
    Speed { speed: f64 },
    /// Spin the real robot's lidar up or down.
    Lidar { on: bool },
    /// Start or cancel the scripted drive test (see `drive_test`).
    DriveTest { on: bool },
    /// Put the real robot in or out of test mode (out lets it charge).
    TestMode { on: bool },
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Backend {
    Sim,
    /// `pi-node` at a host:port, driven by the page's keys.
    Real {
        addr: String,
    },
    /// A log in the recordings folder, by file name.
    Replay {
        file: String,
    },
}

enum Source {
    Sim(SimRun),
    Real(RealRun),
    Replay(ReplayRun),
}

pub struct Session {
    config: RobotConfig,
    start: Pose2,
    /// The simulator's walls.
    world: ::sim::World,
    source: Source,
    /// Bumped on every reset or switch, so the page knows to clear trails.
    run: u64,
    keys: Keys,
    keys_at: Instant,
    /// Where recordings are written and replays are read from.
    recordings: PathBuf,
    tape: Tape,
    /// The last thing that went wrong with a switch, for the page.
    notice: Option<String>,
    /// Replay speed, kept across replays.
    speed: f64,
    /// The ceiling camera, when the station was started with one.
    camera: Option<CameraFeed>,
    /// The taped arena's corners, drawn under the real robot with the camera.
    arena: Vec<[f64; 2]>,
    /// The drive test, when one is running.
    test: Option<DriveTestRun>,
}

struct DriveTestRun {
    started: Instant,
    /// Whether the test started the recording, and so stops it.
    recording: bool,
    step: &'static str,
}

impl Session {
    pub fn new(config: RobotConfig, start: Pose2, recordings: PathBuf) -> Self {
        Self {
            source: Source::Sim(SimRun::new(&demo_world(), &config, start)),
            config,
            start,
            world: demo_world(),
            run: 0,
            keys: Keys::default(),
            keys_at: Instant::now(),
            recordings,
            tape: Tape::default(),
            notice: None,
            speed: 1.0,
            camera: None,
            test: None,
            arena: Vec::new(),
        }
    }

    /// Simulates this world instead of the demo room, starting at `start`.
    pub fn with_world(mut self, world: ::sim::World, start: Pose2) -> Self {
        self.source = Source::Sim(SimRun::new(&world, &self.config, start));
        self.world = world;
        self.start = start;
        self
    }

    /// Takes the real robot's true pose from the ceiling camera, over the
    /// arena with these corners.
    pub fn with_camera(mut self, camera: CameraFeed, arena: Vec<[f64; 2]>) -> Self {
        self.camera = Some(camera);
        self.arena = arena;
        self
    }

    fn open_replay(&self, file: &str) -> std::io::Result<ReplayRun> {
        let path = recorder::path_of(&self.recordings, file)?;
        let recording = Recording::read_file(&path)
            .map_err(|e| std::io::Error::new(e.kind(), format!("reading {file}: {e}")))?;
        Ok(ReplayRun::new(
            file.to_owned(),
            Arc::new(recording),
            &self.config,
            self.speed,
        ))
    }

    fn start_recording(&mut self) {
        let mut info = BTreeMap::new();
        match self.config.to_toml_string() {
            Ok(toml) => info.insert(log::ROBOT_TOML.to_owned(), toml),
            Err(e) => return self.tape.error = Some(format!("couldn't save robot.toml: {e}")),
        };
        let source = match &self.source {
            Source::Sim(sim) => {
                info.insert("world".into(), json!(sim.walls()).to_string());
                "sim"
            }
            Source::Real(real) => {
                info.insert("addr".into(), real.addr.clone());
                "real"
            }
            Source::Replay(_) => {
                return self.tape.error = Some("a replay can't be recorded".into());
            }
        };
        match Recorder::start(&self.recordings, source, info) {
            Ok(rec) => {
                println!(
                    "station: recording to {}",
                    self.recordings.join(&rec.name).display()
                );
                self.tape = Tape {
                    recorder: Some(rec),
                    error: None,
                };
                // The sim keeps its command until it changes, so say what it
                // is now.
                if let Source::Sim(sim) = &self.source {
                    let (t_ns, command) = (sim.robot.sim().time_ns(), sim.command);
                    self.tape.put(|| Entry::Command { t_ns, command });
                }
            }
            Err(e) => {
                self.tape.error = Some(format!(
                    "couldn't record in {}: {e}",
                    self.recordings.display()
                ))
            }
        }
    }

    fn control(&mut self, control: Control) {
        match control {
            Control::Keys(keys) => {
                self.keys = keys;
                self.keys_at = Instant::now();
            }
            Control::Use { backend } => {
                self.end_test();
                let source = match backend {
                    Backend::Sim => Source::Sim(SimRun::new(&self.world, &self.config, self.start)),
                    Backend::Real { addr } => {
                        Source::Real(RealRun::connect(addr.trim().to_owned(), &self.config))
                    }
                    Backend::Replay { file } => match self.open_replay(&file) {
                        Ok(replay) => Source::Replay(replay),
                        Err(e) => return self.notice = Some(e.to_string()),
                    },
                };
                self.tape.stop();
                self.notice = None;
                self.source = source;
                self.run += 1;
            }
            Control::Reset => {
                match &mut self.source {
                    Source::Sim(sim) => {
                        // A fresh sim starts its clock over, so it gets a
                        // fresh recording too.
                        self.tape.stop();
                        *sim = SimRun::new(&self.world, &self.config, self.start)
                    }
                    Source::Real(real) => {
                        let start = self.camera.as_ref().and_then(CameraFeed::fresh);
                        real.readings = Readings::new(&self.config, start.unwrap_or_default());
                        real.anchored = start.is_some();
                    }
                    Source::Replay(replay) => replay.restart(&self.config),
                }
                self.run += 1;
            }
            Control::Pause { paused } => match &mut self.source {
                Source::Sim(sim) => sim.paused = paused,
                Source::Replay(replay) => replay.paused = paused,
                Source::Real(_) => {}
            },
            Control::Record { on: true } if self.tape.recorder.is_none() => self.start_recording(),
            Control::Record { on: false } => self.tape.stop(),
            Control::Record { .. } => {}
            Control::DriveTest { on: true } if self.test.is_none() => {
                if matches!(self.source, Source::Replay(_)) {
                    return self.notice = Some("the drive test needs the sim or the robot".into());
                }
                let recording = self.tape.recorder.is_none();
                if recording {
                    self.start_recording();
                }
                self.test = Some(DriveTestRun {
                    started: Instant::now(),
                    recording,
                    step: "",
                });
            }
            Control::DriveTest { on: true } => {}
            Control::DriveTest { on: false } => self.end_test(),
            Control::Lidar { on } => {
                if let Source::Real(real) = &mut self.source {
                    real.lidar = on;
                    real.settings_sent = false;
                }
            }
            Control::TestMode { on } => {
                if let Source::Real(real) = &mut self.source {
                    real.test_mode = on;
                    real.settings_sent = false;
                }
            }
            Control::Speed { speed } => {
                if speed.is_finite() {
                    self.speed = speed.clamp(*SPEEDS.start(), *SPEEDS.end());
                    if let Source::Replay(replay) = &mut self.source {
                        replay.speed = self.speed;
                    }
                }
            }
        }
    }

    fn end_test(&mut self) {
        if let Some(test) = self.test.take()
            && test.recording
        {
            self.tape.stop();
        }
    }

    /// The drive test's command while one runs, else the keys'. A held
    /// drive key cancels the test.
    fn command(&mut self, keys: Keys) -> Command {
        let max = self.config.drive.max_wheel_speed_mm_s;
        let Some(test) = &mut self.test else {
            return teleop::command(keys, max);
        };
        let keyed = teleop::command(keys, max);
        match drive_test::at(test.started.elapsed()) {
            Some((command, step)) if keyed == Command::Stop => {
                test.step = step;
                command
            }
            _ => {
                self.end_test();
                keyed
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
        let command = self.command(keys);
        let sightings = self
            .camera
            .as_mut()
            .map(CameraFeed::poll)
            .unwrap_or_default();
        let config = &self.config;
        let tape = &mut self.tape;
        let mut anchored = false;
        let result = match &mut self.source {
            Source::Sim(sim) => {
                let (command, step) = if sim.paused {
                    (Command::Stop, Duration::ZERO)
                } else {
                    (command, elapsed.min(MAX_STEP))
                };
                sim.advance(command, step.as_nanos() as u64, config, tape)
                    .await
            }
            Source::Real(real) => {
                while let Ok(event) = real.events.try_recv() {
                    match event {
                        RealEvent::Connected => {
                            real.link = Link::Connected {
                                last_data: Instant::now(),
                            }
                        }
                        RealEvent::Msg(m, at) => {
                            if let Link::Connected { last_data } = &mut real.link {
                                *last_data = at;
                            }
                            real.clock.observe(at, m.t_ns);
                            tape.put(|| Entry::Msg(m.clone()));
                            real.readings.update(m, config)
                        }
                        RealEvent::Closed(why) => {
                            real.link = Link::Closed {
                                why,
                                at: Instant::now(),
                            }
                        }
                    }
                }
                for s in sightings {
                    // Odometry starts where the camera first sees the robot.
                    if !real.anchored {
                        real.readings.odometry =
                            WheelOdometry::new(s.pose, config.drive.wheel_base_m);
                        real.anchored = true;
                        anchored = true;
                    }
                    if let Some(t_ns) = real.clock.to_pi(s.at) {
                        tape.put(|| Entry::Truth { t_ns, pose: s.pose });
                    }
                }
                let mut sent = real.settings();
                sent.extend(real.drive(command));
                if let Some(t_ns) = real.clock.to_pi(Instant::now()) {
                    for command in sent {
                        tape.put(|| Entry::Command { t_ns, command });
                    }
                }
                if let Link::Closed { at, .. } = &real.link
                    && at.elapsed() >= RETRY_AFTER
                {
                    real.reconnect();
                }
                Ok(())
            }
            Source::Replay(replay) => {
                replay.advance(elapsed, config);
                Ok(())
            }
        };
        // Odometry jumped to the camera's pose, so the page starts its
        // trails over.
        if anchored {
            self.run += 1;
        }
        result
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
    use robot::{Msg, Stamped};

    fn config() -> RobotConfig {
        RobotConfig::from_toml_str(include_str!("../../../../robot.toml")).unwrap()
    }

    #[test]
    fn controls_parse_from_page_json() {
        let keys: Control = serde_json::from_str(
            r#"{"op":"keys","forward":true,"back":false,"left":false,"right":false,"fast":false}"#,
        )
        .unwrap();
        assert!(matches!(keys, Control::Keys(Keys { forward: true, .. })));
        let charge: Control = serde_json::from_str(r#"{"op":"testmode","on":false}"#).unwrap();
        assert!(matches!(charge, Control::TestMode { on: false }));
        let lidar: Control = serde_json::from_str(r#"{"op":"lidar","on":false}"#).unwrap();
        assert!(matches!(lidar, Control::Lidar { on: false }));
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
        let mut session = Session::new(
            config(),
            Pose2::new(1.0, 1.0, 0.0),
            PathBuf::from("recordings"),
        );
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

    fn scratch_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("station-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    #[tokio::test]
    async fn a_recorded_sim_drive_replays_with_its_truth() {
        let dir = scratch_dir("sim");
        let mut session = Session::new(config(), Pose2::new(1.0, 1.0, 0.0), dir.clone());
        session.control(Control::Record { on: true });
        assert!(session.tape.recorder.is_some(), "{:?}", session.tape.error);
        for _ in 0..20 {
            session.control(Control::Keys(Keys {
                forward: true,
                left: true,
                ..Keys::default()
            }));
            session.tick(Duration::from_millis(100)).await.unwrap();
        }
        let live = session.frame();
        session.control(Control::Record { on: false });
        assert!(session.tape.recorder.is_none());

        let logs = recorder::list(&dir);
        assert_eq!(logs.len(), 1);
        assert!(logs[0].name.ends_with("-sim.mcap"), "{}", logs[0].name);
        session.control(Control::Use {
            backend: Backend::Replay {
                file: logs[0].name.clone(),
            },
        });
        session.control(Control::Speed { speed: 20.0 });
        for _ in 0..10 {
            session.tick(Duration::from_millis(100)).await.unwrap();
        }
        let replayed = session.frame();
        assert_eq!(replayed["backend"], "replay", "{replayed}");
        assert_eq!(replayed["state"], "ended");
        assert_eq!(replayed["truth"], live["truth"]);
        // Replayed odometry starts from the first recorded truth, so with
        // the sim's perfect wheels it lands on the truth.
        for i in 0..3 {
            let (odo, truth) = (&replayed["odometry"][i], &replayed["truth"][i]);
            let err = (odo.as_f64().unwrap() - truth.as_f64().unwrap()).abs();
            assert!(err < 0.005, "odometry {odo} vs truth {truth}");
        }
        assert_eq!(replayed["walls"], json!(live["walls"]));
        assert!(replayed["command"].as_str().unwrap().starts_with("wheels"));

        // Recording a replay is refused, and says why.
        session.control(Control::Record { on: true });
        assert!(session.tape.recorder.is_none());
        assert!(session.frame()["notice"].is_string());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[tokio::test]
    async fn drive_test_drives_the_sim_records_and_ends() {
        let dir = scratch_dir("drive-test");
        let mut session = Session::new(config(), Pose2::default(), dir.clone());
        session.control(Control::DriveTest { on: true });
        assert!(session.tape.recorder.is_some(), "the test records itself");
        session.tick(FRAME_PERIOD).await.unwrap();
        let Source::Sim(sim) = &session.source else {
            panic!("expected the sim");
        };
        assert!(matches!(sim.command, Command::WheelSpeeds { left_mm_s, .. } if left_mm_s > 0.0));
        assert_eq!(session.frame()["test"]["step"], "forward");

        // Skip to the reverse step, then past the end.
        session.test.as_mut().unwrap().started -= Duration::from_secs_f32(4.5);
        session.tick(FRAME_PERIOD).await.unwrap();
        let Source::Sim(sim) = &session.source else {
            panic!("expected the sim");
        };
        assert!(matches!(sim.command, Command::WheelSpeeds { left_mm_s, .. } if left_mm_s < 0.0));
        session.test.as_mut().unwrap().started -= drive_test::length();
        session.tick(FRAME_PERIOD).await.unwrap();
        assert!(session.test.is_none());
        assert!(
            session.tape.recorder.is_none(),
            "the test stops its recording"
        );
        assert_eq!(recorder::list(&dir).len(), 1);

        // A drive key cancels a running test.
        session.control(Control::DriveTest { on: true });
        session.control(Control::Keys(Keys {
            left: true,
            ..Keys::default()
        }));
        session.tick(FRAME_PERIOD).await.unwrap();
        assert!(session.test.is_none());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[tokio::test]
    async fn replay_refuses_paths_outside_the_folder() {
        let dir = scratch_dir("paths");
        let mut session = Session::new(config(), Pose2::default(), dir);
        session.control(Control::Use {
            backend: Backend::Replay {
                file: "../robot.toml".into(),
            },
        });
        assert!(matches!(session.source, Source::Sim(_)));
        assert!(session.frame()["notice"].is_string());
    }

    #[tokio::test]
    async fn records_the_real_robot_and_sends_only_settings_while_idle() {
        use robot::wire;
        use tokio::io::{AsyncReadExt, AsyncWriteExt};
        use tokio::net::TcpListener;

        let dir = scratch_dir("real");
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let mut session = Session::new(config(), Pose2::default(), dir.clone());
        session.control(Control::Use {
            backend: Backend::Real { addr },
        });
        session.control(Control::Record { on: true });
        let (mut pi, _) = listener.accept().await.unwrap();
        for seq in 0..3 {
            let m = Stamped {
                t_ns: 1_000 + seq as u64,
                seq,
                msg: Msg::Wheels(robot::WheelPositions {
                    left_mm: seq as i64,
                    right_mm: 0,
                }),
            };
            pi.write_all(&wire::encode_frame(&m).unwrap())
                .await
                .unwrap();
        }
        for _ in 0..10 {
            session.tick(FRAME_PERIOD).await.unwrap();
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        session.control(Control::Record { on: false });

        let logs = recorder::list(&dir);
        let rec = Recording::read_file(dir.join(&logs[0].name)).unwrap();
        assert!(!rec.truncated);
        assert_eq!(rec.info["source"], "real");
        assert!(rec.info.contains_key(log::ROBOT_TOML));
        let msgs = rec.entries.iter().filter(|e| matches!(e, Entry::Msg(_)));
        assert_eq!(msgs.count(), 3);
        assert!(rec.entries.iter().all(|e| matches!(
            e,
            Entry::Msg(_)
                | Entry::Command {
                    command: Command::TestMode { on: true } | Command::Lidar { on: true },
                    ..
                }
        )));

        session.control(Control::Use {
            backend: Backend::Sim,
        });
        // With no keys held, only the settings went out, once.
        let mut sent = Vec::new();
        while let Ok(len) = pi.read_u32_le().await {
            let mut body = vec![0; len as usize];
            pi.read_exact(&mut body).await.unwrap();
            sent.push(wire::decode_body::<Command>(&body).unwrap());
        }
        assert_eq!(
            sent,
            [Command::TestMode { on: true }, Command::Lidar { on: true }]
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[tokio::test]
    async fn records_the_cameras_truth_on_the_pis_clock() {
        use crate::camera::Sighting;
        use robot::wire;
        use tokio::io::AsyncWriteExt;
        use tokio::net::TcpListener;

        let dir = scratch_dir("camera");
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let (camera, see) = CameraFeed::fake();
        let mut session = Session::new(config(), Pose2::default(), dir.clone())
            .with_camera(camera, vec![[0.0, 0.0], [3.0, 0.0], [3.0, 1.5], [0.0, 1.5]]);
        session.control(Control::Use {
            backend: Backend::Real { addr },
        });
        session.control(Control::Record { on: true });
        let (mut pi, _) = listener.accept().await.unwrap();
        // The Pi's clock is far from the station's.
        let pi_t = 7_000_000_000u64;
        let wheels = |seq: u32, t_ns: u64| Stamped {
            t_ns,
            seq,
            msg: Msg::Wheels(robot::WheelPositions {
                left_mm: 0,
                right_mm: 0,
            }),
        };
        pi.write_all(&wire::encode_frame(&wheels(0, pi_t)).unwrap())
            .await
            .unwrap();
        for _ in 0..5 {
            session.tick(FRAME_PERIOD).await.unwrap();
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let seen_at = Instant::now();
        see(Sighting {
            at: seen_at,
            pose: Pose2::new(1.2, 0.7, 0.5),
        });
        pi.write_all(&wire::encode_frame(&wheels(1, pi_t + 100_000_000)).unwrap())
            .await
            .unwrap();
        for _ in 0..5 {
            session.tick(FRAME_PERIOD).await.unwrap();
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let frame = session.frame();
        assert_eq!(frame["truth"], json!([1.2, 0.7, 0.5]));
        // Odometry starts where the camera first saw the robot.
        assert_eq!(frame["odometry"], json!([1.2, 0.7, 0.5]));
        assert_eq!(frame["arena"][2], json!([3.0, 1.5]));
        session.control(Control::Record { on: false });

        let logs = recorder::list(&dir);
        let rec = Recording::read_file(dir.join(&logs[0].name)).unwrap();
        let truths: Vec<_> = rec
            .entries
            .iter()
            .filter_map(|e| match e {
                Entry::Truth { t_ns, pose } => Some((*t_ns, *pose)),
                _ => None,
            })
            .collect();
        assert_eq!(truths.len(), 1);
        // Seen about 50 ms after the first wheels message arrived, which is
        // where the Pi's clock stood then, give or take the local delivery.
        let (t_ns, pose) = truths[0];
        assert!(
            (pi_t..pi_t + 500_000_000).contains(&t_ns),
            "truth at {t_ns}, the Pi at {pi_t}"
        );
        assert_eq!(pose, Pose2::new(1.2, 0.7, 0.5));
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[tokio::test]
    async fn real_robot_drives_from_keys_and_stops_on_release() {
        use robot::wire;
        use tokio::io::{AsyncReadExt, AsyncWriteExt};
        use tokio::net::TcpListener;

        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let mut session = Session::new(config(), Pose2::default(), PathBuf::from("recordings"));
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

        // Connect before pressing keys: nothing is sent while connecting.
        for _ in 0..5 {
            session.tick(FRAME_PERIOD).await.unwrap();
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert_eq!(session.frame()["state"], "connected");
        session.control(Control::Keys(Keys {
            forward: true,
            ..Keys::default()
        }));
        session.tick(FRAME_PERIOD).await.unwrap();
        session.control(Control::Keys(Keys::default()));
        session.tick(FRAME_PERIOD).await.unwrap();

        let mut sent = Vec::new();
        for _ in 0..4 {
            let len = pi.read_u32_le().await.unwrap() as usize;
            let mut body = vec![0; len];
            pi.read_exact(&mut body).await.unwrap();
            sent.push(wire::decode_body::<Command>(&body).unwrap());
        }
        assert_eq!(
            sent[..2],
            [Command::TestMode { on: true }, Command::Lidar { on: true }]
        );
        assert!(
            matches!(sent[2], Command::WheelSpeeds { left_mm_s, right_mm_s } if left_mm_s > 0.0 && left_mm_s == right_mm_s),
            "{sent:?}"
        );
        assert_eq!(sent[3], Command::Stop);
    }
}
