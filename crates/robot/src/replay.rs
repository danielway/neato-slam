use std::collections::VecDeque;
use std::io;

use crate::{Command, Msg, Pose2, RobotIo, Stamped};

/// Plays back a recorded run, as fast as it is read. Pacing to the original
/// timing is the caller's job, using the stamps.
pub struct ReplayBackend {
    messages: VecDeque<Stamped<Msg>>,
}

impl ReplayBackend {
    /// Logs come in as lists until recording lands. The plan is MCAP files
    /// with one channel per message type, one for commands sent, one for
    /// ground truth when the source has it, and the run's `robot.toml` as
    /// metadata, so sim and real runs look alike downstream.
    pub fn from_messages(messages: impl IntoIterator<Item = Stamped<Msg>>) -> Self {
        Self {
            messages: messages.into_iter().collect(),
        }
    }
}

impl RobotIo for ReplayBackend {
    async fn recv(&mut self) -> io::Result<Option<Stamped<Msg>>> {
        Ok(self.messages.pop_front())
    }

    async fn send(&mut self, _command: Command) -> io::Result<()> {
        Ok(())
    }

    fn ground_truth(&self) -> Option<Pose2> {
        None
    }
}
