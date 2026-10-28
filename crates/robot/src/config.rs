use serde::{Deserialize, Serialize};

/// Contents of `robot.toml`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RobotConfig {
    pub drive: DriveConfig,
    pub lidar: LidarConfig,
    /// The ceiling camera's marker on the robot's top, if it has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub marker: Option<MarkerConfig>,
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

/// An AprilTag 36h11 on the robot's top: its id, size, height above the
/// floor, and pose in the robot frame, where `theta_rad` is the angle of the
/// tag's top edge direction from robot forward.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MarkerConfig {
    pub id: u32,
    pub size_m: f64,
    pub height_m: f64,
    pub x_m: f64,
    pub y_m: f64,
    pub theta_rad: f64,
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
        assert_eq!(config.marker.as_ref().map(|m| m.id), Some(0));
    }

    #[test]
    fn marker_is_optional_and_survives_a_round_trip() {
        let config = RobotConfig::from_toml_str(include_str!("../../../robot.toml")).unwrap();
        let back = RobotConfig::from_toml_str(&config.to_toml_string().unwrap()).unwrap();
        assert_eq!(back, config);
        let bare = RobotConfig {
            marker: None,
            ..config
        };
        let text = bare.to_toml_string().unwrap();
        assert!(!text.contains("marker"));
        assert_eq!(RobotConfig::from_toml_str(&text).unwrap(), bare);
    }
}
