use robot::{Pose2, normalize_angle};

/// Ideal differential-drive kinematics.
#[derive(Debug, Clone)]
pub struct DiffDrive {
    pub pose: Pose2,
    wheel_base_m: f64,
    left_m: f64,
    right_m: f64,
}

impl DiffDrive {
    pub fn new(pose: Pose2, wheel_base_m: f64) -> Self {
        Self {
            pose,
            wheel_base_m,
            left_m: 0.0,
            right_m: 0.0,
        }
    }

    /// Moves with wheel surface speeds in m/s for `dt_s` seconds.
    pub fn step(&mut self, left_m_s: f64, right_m_s: f64, dt_s: f64) {
        let dl = left_m_s * dt_s;
        let dr = right_m_s * dt_s;
        self.left_m += dl;
        self.right_m += dr;
        let ds = (dl + dr) / 2.0;
        let dtheta = (dr - dl) / self.wheel_base_m;
        // Midpoint integration is exact enough at physics step sizes.
        let mid = self.pose.theta + dtheta / 2.0;
        self.pose.x += ds * mid.cos();
        self.pose.y += ds * mid.sin();
        self.pose.theta = normalize_angle(self.pose.theta + dtheta);
    }

    /// Cumulative distance travelled by each wheel, in metres.
    pub fn wheel_travel_m(&self) -> (f64, f64) {
        (self.left_m, self.right_m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn spinning_in_place_turns_without_moving() {
        let base = 0.24;
        let mut d = DiffDrive::new(Pose2::default(), base);
        // Half a turn: each wheel travels a quarter of the wheel-base circle.
        let arc = PI * base / 2.0;
        for _ in 0..100 {
            d.step(-arc, arc, 0.01);
        }
        assert!(d.pose.x.abs() < 1e-9 && d.pose.y.abs() < 1e-9);
        assert!((d.pose.theta.abs() - PI).abs() < 1e-9);
    }
}
