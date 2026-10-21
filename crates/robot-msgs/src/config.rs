use serde::{Deserialize, Serialize};

/// Contents of `robot.toml`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RobotConfig {
    pub drive: DriveConfig,
    pub lidar: LidarConfig,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DriveConfig {
    pub wheel_base_m: f64,
    pub max_wheel_speed_mm_s: f32,
}

/// Lidar mount pose in the robot frame, plus its scan properties.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LidarConfig {
    pub x_m: f64,
    pub y_m: f64,
    pub theta_rad: f64,
    pub beams: usize,
    pub min_range_m: f32,
    pub max_range_m: f32,
    pub scan_hz: f32,
}

impl RobotConfig {
    pub fn from_toml_str(s: &str) -> Result<Self, toml::de::Error> {
        toml::from_str(s)
    }

    pub fn to_toml_string(&self) -> Result<String, toml::ser::Error> {
        toml::to_string(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repo_robot_toml_parses() {
        let config = RobotConfig::from_toml_str(include_str!("../../../robot.toml")).unwrap();
        assert_eq!(config.lidar.beams, 360);
    }
}
