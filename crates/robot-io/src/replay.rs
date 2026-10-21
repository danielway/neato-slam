use std::collections::VecDeque;
use std::io;

use robot_msgs::{Command, Msg, Pose2, Stamped};

use crate::RobotIo;

/// Plays back a recorded run, as fast as it is read. Pacing to the original
/// timing is the caller's job, using the stamps.
pub struct ReplayBackend {
    messages: VecDeque<Stamped<Msg>>,
}

impl ReplayBackend {
    /// Until the MCAP reader lands in `recorder`, logs come in as lists.
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
