//! One interface over three interchangeable robot sources. SLAM, recording
//! and the station only ever see `RobotIo`.

mod real;
mod replay;
mod sim;

pub use real::RealBackend;
pub use replay::ReplayBackend;
pub use sim::SimBackend;

use std::future::Future;
use std::io;

use robot_msgs::{Command, Msg, Pose2, Stamped};

pub trait RobotIo {
    /// The next message, waiting if needed. None means the source has ended.
    fn recv(&mut self) -> impl Future<Output = io::Result<Option<Stamped<Msg>>>> + Send;

    /// Sends a command. Replay ignores commands, because a log can't react.
    fn send(&mut self, command: Command) -> impl Future<Output = io::Result<()>> + Send;

    /// The true pose, when the source knows it: always in the sim, and in
    /// replay when the log carries ground truth.
    fn ground_truth(&self) -> Option<Pose2>;
}
