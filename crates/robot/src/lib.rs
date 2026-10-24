//! What every part of the stack shares: the message vocabulary, the robot
//! config, the Pi-to-desktop wire format, and the `RobotIo` interface with
//! its real and replay backends. The sim backend lives in the `sim` crate.
//!
//! Messages carry raw readings rather than interpreted ones (wheel travel
//! rather than velocities, ranges rather than points), so replay and the
//! simulator feed SLAM exactly what the hardware produces.

mod config;
mod io;
#[cfg(feature = "net")]
mod real;
mod replay;
pub mod wire;

pub use config::{DriveConfig, LidarConfig, RobotConfig};
pub use io::RobotIo;
#[cfg(feature = "net")]
pub use real::RealBackend;
pub use replay::ReplayBackend;

use serde::{Deserialize, Serialize};

/// A message stamped by the source that produced it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Stamped<T> {
    /// Source clock in nanoseconds: the Pi's monotonic clock on the real
    /// robot, a virtual clock in the simulator and in replay.
    pub t_ns: u64,
    /// Per-stream counter, so dropped messages show up as gaps.
    pub seq: u32,
    pub msg: T,
}

/// Everything a robot source can report.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Msg {
    Lidar(LidarScan),
    Wheels(WheelPositions),
    Contacts(Contacts),
    Status(RobotStatus),
}

/// One full lidar rotation, in the lidar's own frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LidarScan {
    /// Angle of the first beam, radians.
    pub angle_min: f32,
    /// Angle between consecutive beams, radians.
    pub angle_increment: f32,
    /// Ranges in metres; NaN means no valid return.
    pub ranges: Vec<f32>,
    pub intensities: Option<Vec<u16>>,
    /// Seconds taken by one rotation, for de-skewing scans taken while moving.
    pub scan_duration: f32,
}

/// Cumulative wheel travel. The Neato reports positions in millimetres
/// directly, and cumulative values mean a dropped message loses no distance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct WheelPositions {
    pub left_mm: i64,
    pub right_mm: i64,
}

/// Bumper and wheel-drop switches, one bit each.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Contacts {
    pub bumpers: u8,
    pub wheel_drop: u8,
}

impl Contacts {
    pub const LEFT_SIDE: u8 = 1 << 0;
    pub const LEFT_FRONT: u8 = 1 << 1;
    pub const RIGHT_FRONT: u8 = 1 << 2;
    pub const RIGHT_SIDE: u8 = 1 << 3;

    pub const LEFT_WHEEL: u8 = 1 << 0;
    pub const RIGHT_WHEEL: u8 = 1 << 1;

    pub fn any_bumper(&self) -> bool {
        self.bumpers != 0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RobotStatus {
    pub battery_v: f32,
    pub motors_enabled: bool,
}

/// Commands from the desktop to the robot.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Command {
    /// Wheel surface speeds in mm/s, positive forward.
    WheelSpeeds {
        left_mm_s: f32,
        right_mm_s: f32,
    },
    Stop,
}

/// A planar pose in metres and radians. Desktop-only, never on the wire.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Pose2 {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self {
            x,
            y,
            theta: normalize_angle(theta),
        }
    }

    /// Transforms a point from this pose's frame into the parent frame.
    pub fn transform_point(&self, x: f64, y: f64) -> (f64, f64) {
        let (s, c) = self.theta.sin_cos();
        (self.x + c * x - s * y, self.y + s * x + c * y)
    }

    /// The pose of `other`, given relative to this pose, in the parent frame.
    pub fn compose(&self, other: &Pose2) -> Pose2 {
        let (x, y) = self.transform_point(other.x, other.y);
        Pose2::new(x, y, self.theta + other.theta)
    }
}

/// Wraps an angle into [-pi, pi).
pub fn normalize_angle(a: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    (a + PI).rem_euclid(TAU) - PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    #[test]
    fn normalize_wraps_into_range() {
        assert!((normalize_angle(3.0 * PI) - -PI).abs() < 1e-12);
        assert!((normalize_angle(-FRAC_PI_2) - -FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn compose_rotates_then_translates() {
        let a = Pose2::new(1.0, 0.0, FRAC_PI_2);
        let b = a.compose(&Pose2::new(1.0, 0.0, 0.0));
        assert!((b.x - 1.0).abs() < 1e-12);
        assert!((b.y - 1.0).abs() < 1e-12);
        assert!((b.theta - FRAC_PI_2).abs() < 1e-12);
    }
}
