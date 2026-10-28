//! The one live robot source the web page is looking at, and the loop that
//! turns its messages into frames for the page.
//!
//! The sim is stepped in wall-clock time and driven by the page's keys. The
//! real robot is view-only: its messages are drawn, but this module never
//! sends it a command, so the page can't move the drive motors. A replay
//! plays a recorded log at its original pace, or faster.
//!
//! The sim or the real robot can be recorded while it is shown: everything
//! it reports goes into a log in the recordings folder (see `recorder`), and
//! for the sim, the commands it was given and its true pose too. With the
//! ceiling camera running (see `camera`), the real robot has a true pose as
//! well, and its recordings carry it.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use robot::log::{self, Entry, Recording};
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

use crate::camera::{CameraFeed, PiClock};
use crate::recorder::{self, Recorder};
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
/// How long to wait before reconnecting to a Pi that dropped or refused us.
const RETRY_AFTER: Duration = Duration::from_secs(2);
/// A replay skips through quiet stretches longer than this, such as a
/// dropped link or a restarted pi-node, whose stamps start over.
const MAX_REPLAY_GAP_NS: u64 = 2_000_000_000;
const SPEEDS: std::ops::RangeInclusive<f64> = 0.1..=20.0;

/// What the page can ask for.
#[derive(Debug, Deserialize)]
#[serde(tag = "op", rename_all = "lowercase")]
pub enum Control {
    /// The drive keys currently held. Only the sim acts on them.
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
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Backend {
    Sim,
    /// `pi-node` at a host:port, view-only.
    Real {
        addr: String,
    },
    /// A log in the recordings folder, by file name.
    Replay {
        file: String,
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
        tape: &mut Tape,
    ) -> std::io::Result<()> {
        if command != self.command {
            self.robot.send(command).await?;
            self.command = command;
            let t_ns = self.robot.sim().time_ns();
            tape.put(|| Entry::Command { t_ns, command });
        }
        self.clock_ns += step_ns;
        while self.readings.t_ns < self.clock_ns {
            let Some(m) = self.robot.recv().await? else {
                break;
            };
            // Wheels come first from each physics step, when the sim's pose
            // is that step's, so the truth goes in just ahead of them.
            if matches!(m.msg, Msg::Wheels(_)) {
                let pose = self
                    .robot
                    .ground_truth()
                    .expect("the sim always knows its pose");
                tape.put(|| Entry::Truth { t_ns: m.t_ns, pose });
            }
            tape.put(|| Entry::Msg(m.clone()));
            self.readings.update(m, config);
        }
        Ok(())
    }

    fn walls(&self) -> Vec<[f64; 4]> {
        self.robot
            .sim()
            .world
            .segments
            .iter()
            .map(|s| [s.a.0, s.a.1, s.b.0, s.b.1])
            .collect()
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
    /// The link dropped, and a reconnect is due once `RETRY_AFTER` passes.
    Closed {
        why: String,
        at: Instant,
    },
}

/// What the reader task reports.
enum RealEvent {
    Connected,
    /// A message, and when it arrived.
    Msg(Stamped<Msg>, Instant),
    Closed(String),
}

struct RealRun {
    addr: String,
    link: Link,
    readings: Readings,
    events: mpsc::Receiver<RealEvent>,
    reader: JoinHandle<()>,
    /// The Pi's clock, for stamping the camera's sightings.
    clock: PiClock,
    /// Whether odometry has been started from the camera's pose yet.
    anchored: bool,
}

impl RealRun {
    /// Connects in the background. The reader task owns the backend and only
    /// ever calls `recv` on it.
    fn connect(addr: String, config: &RobotConfig) -> Self {
        let (events, reader) = Self::spawn_reader(addr.clone());
        Self {
            addr,
            link: Link::Connecting,
            readings: Readings::new(config, Pose2::default()),
            events,
            reader,
            clock: PiClock::new(),
            anchored: false,
        }
    }

    /// Tries again after a drop, keeping the readings so odometry carries on.
    fn reconnect(&mut self) {
        self.reader.abort();
        (self.events, self.reader) = Self::spawn_reader(self.addr.clone());
        self.link = Link::Connecting;
        // A restarted pi-node starts its clock over.
        self.clock = PiClock::new();
    }

    fn spawn_reader(target: String) -> (mpsc::Receiver<RealEvent>, JoinHandle<()>) {
        let (tx, events) = mpsc::channel(256);
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
                        if tx.send(RealEvent::Msg(m, Instant::now())).await.is_err() {
                            return;
                        }
                    }
                    Ok(None) => break "pi-node closed the connection".to_owned(),
                    Err(e) => break format!("link error: {e}"),
                }
            };
            let _ = tx.send(RealEvent::Closed(closed)).await;
        });
        (events, reader)
    }
}

impl Drop for RealRun {
    fn drop(&mut self) {
        self.reader.abort();
    }
}

struct ReplayRun {
    name: String,
    recording: Arc<Recording>,
    /// Each entry's place on the playback clock: its stamp relative to the
    /// first, with long gaps cut short.
    timeline: Vec<u64>,
    /// The next entry to play.
    next: usize,
    clock_ns: u64,
    speed: f64,
    paused: bool,
    readings: Readings,
    truth: Option<Pose2>,
    command: Option<Command>,
    /// Walls of a sim recording's world.
    walls: Value,
}

impl ReplayRun {
    fn new(name: String, recording: Arc<Recording>, config: &RobotConfig, speed: f64) -> Self {
        let mut timeline = Vec::with_capacity(recording.entries.len());
        let mut last = None;
        let mut at = 0u64;
        for entry in &recording.entries {
            let t = entry.t_ns();
            if let Some(last) = last {
                at += t.saturating_sub(last).min(MAX_REPLAY_GAP_NS);
            }
            last = Some(t);
            timeline.push(at);
        }
        // Odometry starts where the truth does, when the log has it.
        let start = recording
            .entries
            .iter()
            .find_map(|e| match e {
                Entry::Truth { pose, .. } => Some(*pose),
                _ => None,
            })
            .unwrap_or_default();
        let walls = recording
            .info
            .get("world")
            .and_then(|w| serde_json::from_str(w).ok())
            .unwrap_or_else(|| json!([]));
        Self {
            name,
            timeline,
            next: 0,
            clock_ns: 0,
            speed,
            paused: false,
            readings: Readings::new(config, start),
            truth: None,
            command: None,
            walls,
            recording,
        }
    }

    fn restart(&mut self, config: &RobotConfig) {
        let paused = self.paused;
        *self = Self::new(
            self.name.clone(),
            self.recording.clone(),
            config,
            self.speed,
        );
        self.paused = paused;
    }

    fn duration_ns(&self) -> u64 {
        self.timeline.last().copied().unwrap_or(0)
    }

    fn ended(&self) -> bool {
        self.next >= self.timeline.len()
    }

    fn advance(&mut self, elapsed: Duration, config: &RobotConfig) {
        if !self.paused {
            self.clock_ns += (elapsed.min(MAX_STEP).as_nanos() as f64 * self.speed) as u64;
        }
        while self.next < self.timeline.len() && self.timeline[self.next] <= self.clock_ns {
            match &self.recording.entries[self.next] {
                Entry::Msg(m) => self.readings.update(m.clone(), config),
                Entry::Truth { pose, .. } => self.truth = Some(*pose),
                Entry::Command { command, .. } => self.command = Some(*command),
            }
            self.next += 1;
        }
        self.clock_ns = self.clock_ns.min(self.duration_ns());
    }
}

enum Source {
    Sim(SimRun),
    Real(RealRun),
    Replay(ReplayRun),
}

/// The recording in progress, if any.
#[derive(Default)]
struct Tape {
    recorder: Option<Recorder>,
    /// Why the last recording failed, for the page.
    error: Option<String>,
}

impl Tape {
    /// Writes an entry if recording. The entry is only built when needed.
    fn put(&mut self, entry: impl FnOnce() -> Entry) {
        let Some(rec) = &mut self.recorder else {
            return;
        };
        if let Err(e) = rec.write(&entry()) {
            self.error = Some(format!("recording {} stopped: {e}", rec.name));
            self.recorder = None;
        }
    }

    fn stop(&mut self) {
        if let Some(rec) = self.recorder.take() {
            let name = rec.name.clone();
            match rec.finish() {
                Ok(()) => println!("station: saved recording {name}"),
                Err(e) => self.error = Some(format!("saving recording {name} failed: {e}")),
            }
        }
    }
}

pub struct Session {
    config: RobotConfig,
    start: Pose2,
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
}

impl Session {
    pub fn new(config: RobotConfig, start: Pose2, recordings: PathBuf) -> Self {
        Self {
            source: Source::Sim(SimRun::new(&config, start)),
            config,
            start,
            run: 0,
            keys: Keys::default(),
            keys_at: Instant::now(),
            recordings,
            tape: Tape::default(),
            notice: None,
            speed: 1.0,
            camera: None,
            arena: Vec::new(),
        }
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
                let source = match backend {
                    Backend::Sim => Source::Sim(SimRun::new(&self.config, self.start)),
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
                        *sim = SimRun::new(&self.config, self.start)
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

    fn held_keys(&self) -> Keys {
        if self.keys_at.elapsed() > KEYS_TIMEOUT {
            Keys::default()
        } else {
            self.keys
        }
    }

    async fn tick(&mut self, elapsed: Duration) -> std::io::Result<()> {
        let keys = self.held_keys();
        let sightings = self
            .camera
            .as_mut()
            .map(CameraFeed::poll)
            .unwrap_or_default();
        let config = &self.config;
        let tape = &mut self.tape;
        match &mut self.source {
            Source::Sim(sim) => {
                let (command, step) = if sim.paused {
                    (Command::Stop, Duration::ZERO)
                } else {
                    let max = config.drive.max_wheel_speed_mm_s;
                    (teleop::command(keys, max), elapsed.min(MAX_STEP))
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
                    }
                    if let Some(t_ns) = real.clock.to_pi(s.at) {
                        tape.put(|| Entry::Truth { t_ns, pose: s.pose });
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
        }
    }

    fn frame(&self) -> Value {
        let pose = |p: Pose2| json!([round(p.x), round(p.y), round(p.theta)]);
        let (readings, mut frame) = match &self.source {
            Source::Sim(sim) => {
                let frame = json!({
                    "backend": "sim",
                    "state": if sim.paused { "paused" } else { "running" },
                    "command": describe(sim.command),
                    "truth": pose(sim.robot.ground_truth().expect("the sim always knows its pose")),
                    "walls": sim.walls(),
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
                    Link::Closed { why, .. } => ("disconnected", format!("{why}; retrying")),
                };
                let frame = json!({
                    "backend": "real",
                    "addr": real.addr,
                    "state": state,
                    "detail": detail,
                    "command": "view only",
                    "truth": self.camera.as_ref().and_then(CameraFeed::fresh).map(pose),
                    "walls": [],
                });
                (&real.readings, frame)
            }
            Source::Replay(replay) => {
                let state = if replay.ended() {
                    "ended"
                } else if replay.paused {
                    "paused"
                } else {
                    "playing"
                };
                let mut detail = format!(
                    "{}  {:.1} / {:.1} s",
                    replay.name,
                    replay.clock_ns as f64 * 1e-9,
                    replay.duration_ns() as f64 * 1e-9,
                );
                if replay.recording.truncated {
                    detail.push_str(" (cut short)");
                }
                let frame = json!({
                    "backend": "replay",
                    "file": replay.name,
                    "state": state,
                    "detail": detail,
                    "command": replay.command.map_or_else(|| "replay".to_owned(), describe),
                    "truth": replay.truth.map(pose),
                    "walls": replay.walls,
                    "speed": replay.speed,
                });
                (&replay.readings, frame)
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
            "recording": self.tape.recorder.as_ref().map(|r| json!({
                "file": r.name,
                "seconds": r.started.elapsed().as_secs_f64().round(),
                "entries": r.entries,
            })),
            "notice": self.tape.error.as_ref().or(self.notice.as_ref()),
            "camera": self.camera.as_ref().map(|c| json!({
                "state": c.state.as_str(),
                "error": c.error,
            })),
        });
        if let (Value::Object(frame), Value::Object(extra)) = (&mut frame, extra) {
            frame.extend(extra);
            // The arena and the floor under it, everywhere but the sim.
            if let Some(camera) = &self.camera
                && !matches!(self.source, Source::Sim(_))
            {
                frame.insert("arena".into(), json!(self.arena));
                if let Some(t) = &camera.topdown {
                    frame.insert("floor".into(), json!({ "bounds": t.bounds }));
                }
            }
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
    async fn records_the_real_robot_without_sending_it_anything() {
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
        assert_eq!(rec.entries.len(), 3);
        assert!(rec.entries.iter().all(|e| matches!(e, Entry::Msg(_))));

        session.control(Control::Use {
            backend: Backend::Sim,
        });
        let mut buf = Vec::new();
        let n = tokio::time::timeout(Duration::from_secs(1), pi.read_to_end(&mut buf))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(n, 0, "station sent {buf:?}");
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
    async fn real_robot_never_receives_commands() {
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
