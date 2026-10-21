use std::collections::VecDeque;
use std::io;

use robot_msgs::{Command, Msg, Pose2, Stamped};
use sim::Sim;

use crate::RobotIo;

/// The simulator, stepped on demand so it runs as fast as it is consumed.
/// It never ends on its own.
pub struct SimBackend {
    sim: Sim,
    pending: VecDeque<Stamped<Msg>>,
}

impl SimBackend {
    pub fn new(sim: Sim) -> Self {
        Self {
            sim,
            pending: VecDeque::new(),
        }
    }

    pub fn sim(&self) -> &Sim {
        &self.sim
    }
}

impl RobotIo for SimBackend {
    async fn recv(&mut self) -> io::Result<Option<Stamped<Msg>>> {
        while self.pending.is_empty() {
            self.pending.extend(self.sim.step());
        }
        Ok(self.pending.pop_front())
    }

    async fn send(&mut self, command: Command) -> io::Result<()> {
        self.sim.set_command(command);
        Ok(())
    }

    fn ground_truth(&self) -> Option<Pose2> {
        Some(self.sim.ground_truth())
    }
}
