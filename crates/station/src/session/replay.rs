use std::sync::Arc;
use std::time::Duration;

use robot::log::{Entry, Recording};
use robot::{Command, Pose2, RobotConfig};
use serde_json::{Value, json};

use super::MAX_STEP;
use super::readings::Readings;

/// A replay skips through quiet stretches longer than this, such as a
/// dropped link or a restarted pi-node, whose stamps start over.
const MAX_REPLAY_GAP_NS: u64 = 2_000_000_000;

pub(super) struct ReplayRun {
    pub(super) name: String,
    pub(super) recording: Arc<Recording>,
    /// Each entry's place on the playback clock: its stamp relative to the
    /// first, with long gaps cut short.
    timeline: Vec<u64>,
    /// The next entry to play.
    next: usize,
    pub(super) clock_ns: u64,
    pub(super) speed: f64,
    pub(super) paused: bool,
    pub(super) readings: Readings,
    pub(super) truth: Option<Pose2>,
    pub(super) command: Option<Command>,
    /// Walls of a sim recording's world.
    pub(super) walls: Value,
}

impl ReplayRun {
    pub(super) fn new(
        name: String,
        recording: Arc<Recording>,
        config: &RobotConfig,
        speed: f64,
    ) -> Self {
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

    pub(super) fn restart(&mut self, config: &RobotConfig) {
        let paused = self.paused;
        *self = Self::new(
            self.name.clone(),
            self.recording.clone(),
            config,
            self.speed,
        );
        self.paused = paused;
    }

    pub(super) fn duration_ns(&self) -> u64 {
        self.timeline.last().copied().unwrap_or(0)
    }

    pub(super) fn ended(&self) -> bool {
        self.next >= self.timeline.len()
    }

    pub(super) fn advance(&mut self, elapsed: Duration, config: &RobotConfig) {
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
