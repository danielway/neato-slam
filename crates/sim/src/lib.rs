//! A fixed-step 2D simulator of the Neato.
//!
//! It turns `Command`s into wheel motion and emits the same `Stamped<Msg>`
//! stream as the real robot, plus the exact pose and map as ground truth.
//!
//! Not yet modelled: wheel slip and encoder noise, lidar range noise and
//! dropouts, motion skew within a scan, collisions and bumpers. Their
//! parameters will be fitted from real logs.

mod backend;
mod drive;
mod world;

pub use backend::SimBackend;
pub use drive::DiffDrive;
pub use world::{Segment, World};

use robot::{Command, LidarScan, Msg, Pose2, RobotConfig, Stamped, WheelPositions};

pub struct Sim {
    pub world: World,
    config: RobotConfig,
    drive: DiffDrive,
    command: Command,
    dt_ns: u64,
    t_ns: u64,
    steps_per_scan: u64,
    step: u64,
    wheel_seq: u32,
    lidar_seq: u32,
}

impl Sim {
    /// `dt_s` is the physics step; wheel positions are reported every step.
    pub fn new(world: World, config: RobotConfig, start: Pose2, dt_s: f64) -> Self {
        let steps_per_scan = ((1.0 / (config.lidar.scan_hz as f64 * dt_s)).round() as u64).max(1);
        Self {
            world,
            drive: DiffDrive::new(start, config.drive.wheel_base_m),
            config,
            command: Command::Stop,
            dt_ns: (dt_s * 1e9).round() as u64,
            t_ns: 0,
            steps_per_scan,
            step: 0,
            wheel_seq: 0,
            lidar_seq: 0,
        }
    }

    pub fn set_command(&mut self, command: Command) {
        self.command = command;
    }

    pub fn time_ns(&self) -> u64 {
        self.t_ns
    }

    /// The robot's true pose.
    pub fn ground_truth(&self) -> Pose2 {
        self.drive.pose
    }

    /// Advances one physics step and returns the messages produced during it.
    pub fn step(&mut self) -> Vec<Stamped<Msg>> {
        let max = self.config.drive.max_wheel_speed_mm_s;
        let (left, right) = match self.command {
            Command::WheelSpeeds {
                left_mm_s,
                right_mm_s,
            } => (left_mm_s.clamp(-max, max), right_mm_s.clamp(-max, max)),
            Command::Stop => (0.0, 0.0),
        };
        let dt_s = self.dt_ns as f64 * 1e-9;
        self.drive
            .step(left as f64 / 1000.0, right as f64 / 1000.0, dt_s);
        self.t_ns += self.dt_ns;
        self.step += 1;

        let mut out = Vec::with_capacity(2);
        let (left_m, right_m) = self.drive.wheel_travel_m();
        out.push(Stamped {
            t_ns: self.t_ns,
            seq: self.wheel_seq,
            msg: Msg::Wheels(WheelPositions {
                left_mm: (left_m * 1000.0).round() as i64,
                right_mm: (right_m * 1000.0).round() as i64,
            }),
        });
        self.wheel_seq = self.wheel_seq.wrapping_add(1);

        if self.step % self.steps_per_scan == 0 {
            out.push(Stamped {
                t_ns: self.t_ns,
                seq: self.lidar_seq,
                msg: Msg::Lidar(self.scan()),
            });
            self.lidar_seq = self.lidar_seq.wrapping_add(1);
        }
        out
    }

    /// An ideal scan from the current pose.
    fn scan(&self) -> LidarScan {
        let lidar = &self.config.lidar;
        let mount = Pose2::new(lidar.x_m, lidar.y_m, lidar.theta_rad);
        let origin = self.drive.pose.compose(&mount);
        let increment = std::f64::consts::TAU / lidar.beams as f64;
        let ranges = (0..lidar.beams)
            .map(|i| {
                let angle = origin.theta + i as f64 * increment;
                match self
                    .world
                    .raycast((origin.x, origin.y), angle, lidar.max_range_m as f64)
                {
                    Some(r) if r >= lidar.min_range_m as f64 => r as f32,
                    _ => f32::NAN,
                }
            })
            .collect();
        LidarScan {
            angle_min: 0.0,
            angle_increment: increment as f32,
            ranges,
            intensities: None,
            scan_duration: 1.0 / lidar.scan_hz,
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
    fn driving_straight_moves_and_reports_wheels() {
        let mut sim = Sim::new(
            World::rectangle(4.0, 4.0),
            config(),
            Pose2::new(2.0, 2.0, 0.0),
            0.01,
        );
        sim.set_command(Command::WheelSpeeds {
            left_mm_s: 100.0,
            right_mm_s: 100.0,
        });
        let mut last_wheels = None;
        let mut scans = 0;
        for _ in 0..100 {
            for m in sim.step() {
                match m.msg {
                    Msg::Wheels(w) => last_wheels = Some(w),
                    Msg::Lidar(_) => scans += 1,
                    _ => {}
                }
            }
        }
        assert!((sim.ground_truth().x - 2.1).abs() < 1e-9);
        assert_eq!(
            last_wheels,
            Some(WheelPositions {
                left_mm: 100,
                right_mm: 100
            })
        );
        assert_eq!(scans, 5);
        assert_eq!(sim.time_ns(), 1_000_000_000);
    }

    #[test]
    fn scan_sees_the_walls() {
        let mut config = config();
        config.lidar.x_m = 0.0;
        config.lidar.max_range_m = 2.0;
        let sim = Sim::new(
            World::rectangle(4.0, 4.0),
            config,
            Pose2::new(1.0, 2.0, 0.0),
            0.01,
        );
        let scan = sim.scan();
        // The wall ahead is 3 m away, past max range.
        assert!(scan.ranges[0].is_nan(), "ahead: {}", scan.ranges[0]);
        assert!(
            (scan.ranges[180] - 1.0).abs() < 1e-5,
            "behind: {}",
            scan.ranges[180]
        );
    }
}
