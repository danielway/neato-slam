//! The ceiling camera over the garage arena, as a source of ground truth for
//! the real robot.
//!
//! `tools/camera-tracker.py` does the seeing: it finds the marker on the
//! robot's top in each frame and prints the robot's pose in the arena frame
//! (`arena.toml`), one JSON line per frame. This module runs it, restarts it
//! if it stops, and stamps each pose on the station's clock from the frame's
//! reported age. [`PiClock`] then moves those stamps onto the Pi's clock, so
//! the truth lines up with the robot's own messages in a recording.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::process::Stdio;
use std::time::{Duration, Instant};

use robot::Pose2;
use serde::Deserialize;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;

const TRACKER: &str = "tools/camera-tracker.py";
/// How long to wait before restarting a tracker that stopped.
const RESTART_AFTER: Duration = Duration::from_secs(5);
/// A sighting older than this no longer counts as where the robot is.
pub const FRESH_FOR: Duration = Duration::from_millis(500);

/// Where the robot was seen, and when, on the station's clock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sighting {
    pub at: Instant,
    pub pose: Pose2,
}

/// The floor seen straight down, saved by the tracker every second.
#[derive(Debug, Clone, PartialEq)]
pub struct TopDown {
    pub path: PathBuf,
    /// Arena coordinates the image covers: x0, y0, x1, y1.
    pub bounds: [f64; 4],
}

#[derive(Debug, PartialEq)]
enum Event {
    TopDown(TopDown),
    Seen(Sighting),
    Missed,
    Stopped(String),
}

/// One line of the tracker's output.
#[derive(Deserialize)]
struct Line {
    age_ms: Option<f64>,
    pose: Option<[f64; 3]>,
    topdown: Option<PathBuf>,
    bounds: Option<[f64; 4]>,
}

fn parse(line: &str, now: Instant) -> Option<Event> {
    let line: Line = serde_json::from_str(line).ok()?;
    if let (Some(path), Some(bounds)) = (line.topdown, line.bounds) {
        return Some(Event::TopDown(TopDown { path, bounds }));
    }
    let age = Duration::from_secs_f64(line.age_ms?.max(0.0) / 1000.0);
    Some(match line.pose {
        Some([x, y, theta]) => Event::Seen(Sighting {
            at: now.checked_sub(age).unwrap_or(now),
            pose: Pose2::new(x, y, theta),
        }),
        None => Event::Missed,
    })
}

/// What the page is told about the camera.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum State {
    Starting,
    Tracking,
    Lost,
    Stopped,
}

impl State {
    pub fn as_str(self) -> &'static str {
        match self {
            State::Starting => "starting",
            State::Tracking => "tracking",
            State::Lost => "marker not seen",
            State::Stopped => "stopped",
        }
    }
}

pub struct CameraFeed {
    events: mpsc::Receiver<Event>,
    task: JoinHandle<()>,
    pub latest: Option<Sighting>,
    pub topdown: Option<TopDown>,
    pub state: State,
    /// Why the tracker last stopped, for the page.
    pub error: Option<String>,
}

impl CameraFeed {
    /// Runs the tracker on `device`, from the repo folder the station was
    /// started in, saving the top-down floor to `topdown`.
    pub fn spawn(device: String, topdown: PathBuf) -> Self {
        let (tx, events) = mpsc::channel(256);
        let task = tokio::spawn(async move {
            loop {
                let why = track(&device, &topdown, &tx).await;
                if tx.send(Event::Stopped(why)).await.is_err() {
                    return;
                }
                tokio::time::sleep(RESTART_AFTER).await;
            }
        });
        Self {
            events,
            task,
            latest: None,
            topdown: None,
            state: State::Starting,
            error: None,
        }
    }

    /// Takes in what the tracker has said since the last call, and returns
    /// the new sightings, oldest first.
    pub fn poll(&mut self) -> Vec<Sighting> {
        let mut seen = Vec::new();
        while let Ok(event) = self.events.try_recv() {
            match event {
                Event::TopDown(t) => self.topdown = Some(t),
                Event::Seen(s) => {
                    self.latest = Some(s);
                    self.state = State::Tracking;
                    seen.push(s);
                }
                Event::Missed => self.state = State::Lost,
                Event::Stopped(why) => {
                    eprintln!("station: camera tracker stopped: {why}");
                    self.state = State::Stopped;
                    self.error = Some(why);
                    self.topdown = None;
                }
            }
        }
        seen
    }

    /// The latest sighting, if it is recent enough to show.
    pub fn fresh(&self) -> Option<Pose2> {
        self.latest
            .filter(|s| s.at.elapsed() < FRESH_FOR)
            .map(|s| s.pose)
    }
}

#[cfg(test)]
impl CameraFeed {
    /// A feed with no tracker behind it, and a way to report sightings to it.
    pub fn fake() -> (Self, impl Fn(Sighting)) {
        let (tx, events) = mpsc::channel(256);
        let feed = Self {
            events,
            task: tokio::spawn(async {}),
            latest: None,
            topdown: None,
            state: State::Starting,
            error: None,
        };
        (feed, move |s| tx.try_send(Event::Seen(s)).unwrap())
    }
}

impl Drop for CameraFeed {
    fn drop(&mut self) {
        self.task.abort();
    }
}

/// Runs the tracker until it exits, and says why it did.
async fn track(device: &str, topdown: &std::path::Path, tx: &mpsc::Sender<Event>) -> String {
    let child = tokio::process::Command::new("python3")
        .arg(TRACKER)
        .args(["--device", device, "--topdown"])
        .arg(topdown)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .kill_on_drop(true)
        .spawn();
    let mut child = match child {
        Ok(child) => child,
        Err(e) => return format!("couldn't run {TRACKER}: {e}"),
    };
    let Some(stdout) = child.stdout.take() else {
        return "no output from the tracker".into();
    };
    let mut lines = BufReader::new(stdout).lines();
    loop {
        match lines.next_line().await {
            Ok(Some(line)) => {
                if let Some(event) = parse(&line, Instant::now())
                    && tx.send(event).await.is_err()
                {
                    return "station closing".into();
                }
            }
            Ok(None) => break,
            Err(e) => return format!("reading the tracker: {e}"),
        }
    }
    match child.wait().await {
        Ok(status) => format!("{TRACKER} exited ({status}); its error is in the station's log"),
        Err(e) => format!("{TRACKER}: {e}"),
    }
}

/// Maps the station's clock onto the Pi's, from when the Pi's messages
/// arrive. Each arrival bounds the offset from one side (a message can't
/// arrive before it is sent), so the smallest gap between arrival and stamp
/// over the last few seconds is the offset plus the quickest delivery, a few
/// milliseconds on Wi-Fi. A window rather than all time lets it follow the
/// two clocks drifting apart.
pub struct PiClock {
    epoch: Instant,
    /// The smallest arrival-minus-stamp gap seen in each recent second.
    buckets: VecDeque<(u64, i128)>,
}

/// Seconds of arrivals the offset is taken over.
const CLOCK_WINDOW_S: u64 = 10;

impl PiClock {
    pub fn new() -> Self {
        Self {
            epoch: Instant::now(),
            buckets: VecDeque::new(),
        }
    }

    fn station_ns(&self, at: Instant) -> i128 {
        at.saturating_duration_since(self.epoch).as_nanos() as i128
    }

    /// A Pi message stamped `t_ns` arrived at `at`.
    pub fn observe(&mut self, at: Instant, t_ns: u64) {
        let gap = self.station_ns(at) - t_ns as i128;
        let second = at.saturating_duration_since(self.epoch).as_secs();
        match self.buckets.back_mut() {
            Some((s, min)) if *s == second => *min = (*min).min(gap),
            _ => self.buckets.push_back((second, gap)),
        }
        while self
            .buckets
            .front()
            .is_some_and(|(s, _)| *s + CLOCK_WINDOW_S <= second)
        {
            self.buckets.pop_front();
        }
    }

    /// The Pi's stamp for an instant on the station's clock, once a message
    /// has arrived to tell.
    pub fn to_pi(&self, at: Instant) -> Option<u64> {
        let offset = self.buckets.iter().map(|&(_, gap)| gap).min()?;
        u64::try_from(self.station_ns(at) - offset).ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_trackers_lines() {
        let now = Instant::now();
        let seen = parse(
            r#"{"age_ms":40.0,"pose":[1.5,0.8,0.25],"tag_px":[[1,2],[3,4],[5,6],[7,8]],"side_m":0.1}"#,
            now,
        );
        assert_eq!(
            seen,
            Some(Event::Seen(Sighting {
                at: now - Duration::from_millis(40),
                pose: Pose2::new(1.5, 0.8, 0.25),
            }))
        );
        assert_eq!(
            parse(r#"{"age_ms":3,"pose":null}"#, now),
            Some(Event::Missed)
        );
        assert_eq!(
            parse(
                r#"{"topdown":"/tmp/t.jpg","bounds":[-0.1,-0.1,3.8,1.9]}"#,
                now
            ),
            Some(Event::TopDown(TopDown {
                path: "/tmp/t.jpg".into(),
                bounds: [-0.1, -0.1, 3.8, 1.9],
            }))
        );
        assert_eq!(parse("Traceback (most recent call last):", now), None);
    }

    #[test]
    fn pi_clock_takes_the_quickest_delivery() {
        let mut clock = PiClock::new();
        assert_eq!(clock.to_pi(Instant::now()), None);
        // The Pi's clock reads 5 s at the station's epoch; deliveries take
        // 3 to 30 ms.
        let epoch = clock.epoch;
        let pi_at = |at: Instant| 5_000_000_000 + (at - epoch).as_nanos() as u64;
        for (i, delay_ms) in [30u64, 3, 12, 25, 8].into_iter().enumerate() {
            let sent = epoch + Duration::from_millis(100 * i as u64);
            let t_ns = pi_at(sent);
            clock.observe(sent + Duration::from_millis(delay_ms), t_ns);
        }
        let at = epoch + Duration::from_millis(700);
        let err = clock.to_pi(at).unwrap() as i64 - pi_at(at) as i64;
        assert_eq!(err, -3_000_000, "off by the quickest delivery, 3 ms");
    }

    #[test]
    fn pi_clock_forgets_old_seconds() {
        let mut clock = PiClock::new();
        let epoch = clock.epoch;
        // A quick delivery long ago, then only slow ones after a clock jump.
        clock.observe(epoch, 0);
        for s in 1..=20u64 {
            let at = epoch + Duration::from_secs(s);
            clock.observe(at, (at - epoch).as_nanos() as u64 - 50_000_000);
        }
        let at = epoch + Duration::from_secs(20);
        assert_eq!(
            clock.to_pi(at).unwrap(),
            (at - epoch).as_nanos() as u64 - 50_000_000
        );
    }
}
