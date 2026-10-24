use std::collections::VecDeque;
use std::io;
use std::path::Path;

use crate::log::{Entry, Recording};
use crate::{Command, Msg, Pose2, RobotIo, Stamped};

/// Plays back a recorded run, as fast as it is read. Pacing to the original
/// timing is the caller's job, using the stamps.
pub struct ReplayBackend {
    entries: VecDeque<Entry>,
    truth: Option<Pose2>,
}

impl ReplayBackend {
    /// Opens an MCAP log written by [`crate::log::LogWriter`].
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        Ok(Self::from_recording(Recording::read_file(path)?))
    }

    pub fn from_recording(recording: Recording) -> Self {
        Self {
            entries: recording.entries.into(),
            truth: None,
        }
    }

    pub fn from_messages(messages: impl IntoIterator<Item = Stamped<Msg>>) -> Self {
        Self {
            entries: messages.into_iter().map(Entry::Msg).collect(),
            truth: None,
        }
    }
}

impl RobotIo for ReplayBackend {
    async fn recv(&mut self) -> io::Result<Option<Stamped<Msg>>> {
        while let Some(entry) = self.entries.pop_front() {
            match entry {
                Entry::Msg(m) => return Ok(Some(m)),
                Entry::Truth { pose, .. } => self.truth = Some(pose),
                Entry::Command { .. } => {}
            }
        }
        Ok(None)
    }

    async fn send(&mut self, _command: Command) -> io::Result<()> {
        Ok(())
    }

    /// The latest true pose recorded before the message last returned.
    fn ground_truth(&self) -> Option<Pose2> {
        self.truth
    }
}
