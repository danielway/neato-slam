use robot::{Contacts, LidarScan, Msg, NodeInfo, Pose2, RobotConfig, RobotStatus, Stamped};
use slam::odometry::WheelOdometry;

/// What both sources feed: odometry and the latest readings.
pub(super) struct Readings {
    pub(super) odometry: WheelOdometry,
    /// Stamp of the latest message.
    pub(super) t_ns: u64,
    /// Latest scan hits in the robot frame, metres.
    pub(super) hits: Vec<[f64; 2]>,
    pub(super) contacts: Contacts,
    pub(super) status: Option<RobotStatus>,
    /// What pi-node said about itself when it connected.
    pub(super) node: Option<NodeInfo>,
}

impl Readings {
    pub(super) fn new(config: &RobotConfig, start: Pose2) -> Self {
        Self {
            odometry: WheelOdometry::new(start, config.drive.wheel_base_m),
            t_ns: 0,
            hits: Vec::new(),
            contacts: Contacts::default(),
            status: None,
            node: None,
        }
    }

    pub(super) fn update(&mut self, m: Stamped<Msg>, config: &RobotConfig) {
        self.t_ns = m.t_ns;
        match m.msg {
            Msg::Wheels(w) => {
                self.odometry.update(w);
            }
            Msg::Lidar(scan) => self.hits = robot_frame_hits(&scan, config),
            Msg::Contacts(c) => self.contacts = c,
            Msg::Status(s) => self.status = Some(s),
            Msg::Node(n) => {
                // A new pi-node connection: its wheel counters needn't
                // follow on from the last ones seen.
                self.odometry.rebase();
                self.node = Some(n);
            }
        }
    }
}

/// A scan's valid returns as points in the robot frame.
fn robot_frame_hits(scan: &LidarScan, config: &RobotConfig) -> Vec<[f64; 2]> {
    let lidar = &config.lidar;
    let mount = Pose2::new(lidar.x_m, lidar.y_m, lidar.theta_rad);
    scan.ranges
        .iter()
        .enumerate()
        .filter(|(_, r)| r.is_finite())
        .map(|(i, &r)| {
            let a = (scan.angle_min + i as f32 * scan.angle_increment) as f64;
            let (x, y) = mount.transform_point(r as f64 * a.cos(), r as f64 * a.sin());
            [x, y]
        })
        .collect()
}
