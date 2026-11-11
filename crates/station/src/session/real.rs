use std::time::{Duration, Instant};

use robot::{Command, Msg, Pose2, RealBackend, RobotConfig, Stamped};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;

use super::readings::Readings;
use crate::camera::PiClock;

/// How long to wait for the Pi to accept a connection.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
/// A connected Pi that sends nothing for this long is reported as stalled.
/// It covers the 2 s lidar spin-up after connecting.
pub(super) const STALL_AFTER: Duration = Duration::from_secs(4);
/// How long to wait before reconnecting to a Pi that dropped or refused us.
pub(super) const RETRY_AFTER: Duration = Duration::from_secs(2);
/// How often a drive command is re-sent to `pi-node` while keys are held,
/// well inside its stop watchdog.
const RESEND_EVERY: Duration = Duration::from_millis(100);

pub(super) enum Link {
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
pub(super) enum RealEvent {
    Connected,
    /// A message, and when it arrived.
    Msg(Stamped<Msg>, Instant),
    Closed(String),
}

pub(super) struct RealRun {
    pub(super) addr: String,
    pub(super) link: Link,
    /// Commands for the current connection's writer task.
    commands: mpsc::Sender<Command>,
    /// The last command sent, and when.
    pub(super) command: Command,
    sent_at: Instant,
    /// Whether the lidar should spin and the robot be in test mode, and
    /// whether pi-node has been told so on this connection.
    pub(super) lidar: bool,
    pub(super) test_mode: bool,
    pub(super) settings_sent: bool,
    pub(super) readings: Readings,
    pub(super) events: mpsc::Receiver<RealEvent>,
    reader: JoinHandle<()>,
    /// The Pi's clock, for stamping the camera's sightings.
    pub(super) clock: PiClock,
    /// Whether odometry has been started from the camera's pose yet.
    pub(super) anchored: bool,
}

impl RealRun {
    /// Connects in the background. The reader task owns the backend and only
    /// ever calls `recv` on it.
    pub(super) fn connect(addr: String, config: &RobotConfig) -> Self {
        let (events, commands, reader) = Self::spawn_reader(addr.clone());
        Self {
            addr,
            link: Link::Connecting,
            commands,
            command: Command::Stop,
            sent_at: Instant::now(),
            lidar: true,
            test_mode: true,
            settings_sent: false,
            readings: Readings::new(config, Pose2::default()),
            events,
            reader,
            clock: PiClock::new(),
            anchored: false,
        }
    }

    /// Tries again after a drop, keeping the readings so odometry carries on.
    pub(super) fn reconnect(&mut self) {
        self.reader.abort();
        (self.events, self.commands, self.reader) = Self::spawn_reader(self.addr.clone());
        self.settings_sent = false;
        self.link = Link::Connecting;
        // A restarted pi-node starts its clock over.
        self.clock = PiClock::new();
    }

    /// Sends the lidar and test-mode settings after a connect or a change,
    /// and returns them when they went out, for the recording.
    pub(super) fn settings(&mut self) -> Vec<Command> {
        if self.settings_sent || !matches!(self.link, Link::Connected { .. }) {
            return Vec::new();
        }
        let settings = vec![
            Command::TestMode { on: self.test_mode },
            Command::Lidar { on: self.lidar },
        ];
        if settings.iter().all(|&c| self.commands.try_send(c).is_ok()) {
            self.settings_sent = true;
            settings
        } else {
            Vec::new()
        }
    }

    /// Sends `command` if it changed, or again if keys are still held, and
    /// returns it when it went out, for the recording.
    pub(super) fn drive(&mut self, command: Command) -> Option<Command> {
        if !matches!(self.link, Link::Connected { .. }) {
            self.command = Command::Stop;
            return None;
        }
        let due = command != self.command
            || (command != Command::Stop && self.sent_at.elapsed() >= RESEND_EVERY);
        if !due || self.commands.try_send(command).is_err() {
            return None;
        }
        let changed = command != self.command;
        self.command = command;
        self.sent_at = Instant::now();
        changed.then_some(command)
    }

    fn spawn_reader(
        target: String,
    ) -> (
        mpsc::Receiver<RealEvent>,
        mpsc::Sender<Command>,
        JoinHandle<()>,
    ) {
        let (tx, events) = mpsc::channel(256);
        let (commands, mut queued) = mpsc::channel::<Command>(8);
        let reader = tokio::spawn(async move {
            let connect = tokio::time::timeout(CONNECT_TIMEOUT, RealBackend::connect(target));
            let robot = match connect.await {
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
            let (mut robot, mut sender) = robot.split();
            // Ends when the run drops its sender or the link breaks.
            tokio::spawn(async move {
                while let Some(command) = queued.recv().await {
                    if sender.send(command).await.is_err() {
                        return;
                    }
                }
            });
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
        (events, commands, reader)
    }
}

impl Drop for RealRun {
    fn drop(&mut self) {
        self.reader.abort();
    }
}
