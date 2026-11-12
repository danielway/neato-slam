use robot::log::Entry;
use robot::{Command, Msg, Pose2, RobotConfig, RobotIo};
use sim::{Segment, Sim, SimBackend, World};

use super::readings::Readings;
use super::tape::Tape;

/// A room with a few obstacles, so scans have some structure to show. The
/// sim uses it unless the station is given the arena's layout.
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

pub(super) struct SimRun {
    pub(super) robot: SimBackend,
    pub(super) readings: Readings,
    /// Where sim time should be by now. The sim steps in whole physics
    /// steps, so it runs up to this rather than a frame past its last
    /// stamp, which would round every frame up and run fast.
    clock_ns: u64,
    pub(super) command: Command,
    pub(super) paused: bool,
}

impl SimRun {
    pub(super) fn new(world: &World, config: &RobotConfig, start: Pose2) -> Self {
        Self {
            robot: SimBackend::new(Sim::new(world.clone(), config.clone(), start, 0.01)),
            readings: Readings::new(config, start),
            clock_ns: 0,
            command: Command::Stop,
            paused: false,
        }
    }

    /// Sends the command if it changed, then runs the sim `step_ns` further.
    pub(super) async fn advance(
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

    pub(super) fn walls(&self) -> Vec<[f64; 4]> {
        self.robot
            .sim()
            .world
            .segments
            .iter()
            .map(|s| [s.a.0, s.a.1, s.b.0, s.b.1])
            .collect()
    }
}
