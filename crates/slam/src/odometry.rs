use robot_msgs::{Pose2, WheelPositions};

/// Dead reckoning from cumulative wheel positions. This is the baseline every
/// later algorithm has to beat.
#[derive(Debug, Clone)]
pub struct WheelOdometry {
    wheel_base_m: f64,
    last: Option<WheelPositions>,
    pub pose: Pose2,
}

impl WheelOdometry {
    pub fn new(start: Pose2, wheel_base_m: f64) -> Self {
        Self {
            wheel_base_m,
            last: None,
            pose: start,
        }
    }

    /// Integrates the travel since the previous reading. The first reading
    /// only sets the reference.
    pub fn update(&mut self, wheels: WheelPositions) -> Pose2 {
        if let Some(last) = self.last {
            let dl = (wheels.left_mm - last.left_mm) as f64 / 1000.0;
            let dr = (wheels.right_mm - last.right_mm) as f64 / 1000.0;
            let ds = (dl + dr) / 2.0;
            let dtheta = (dr - dl) / self.wheel_base_m;
            let mid = self.pose.theta + dtheta / 2.0;
            self.pose = Pose2::new(
                self.pose.x + ds * mid.cos(),
                self.pose.y + ds * mid.sin(),
                self.pose.theta + dtheta,
            );
        }
        self.last = Some(wheels);
        self.pose
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_travel_moves_forward() {
        let mut odo = WheelOdometry::new(Pose2::default(), 0.24);
        odo.update(WheelPositions {
            left_mm: 1000,
            right_mm: 1000,
        });
        let pose = odo.update(WheelPositions {
            left_mm: 1500,
            right_mm: 1500,
        });
        assert!((pose.x - 0.5).abs() < 1e-12);
        assert_eq!(pose.y, 0.0);
    }
}
