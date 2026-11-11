//! SLAM algorithms. Pure computation over `Stamped<Msg>` streams with no I/O,
//! so the same code runs against the robot, replay and the simulator.
//!
//! Planned progression, each step scored against ground truth before the next:
//! occupancy grid mapping from known poses, wheel odometry refined by scan
//! matching, then pose-graph SLAM with loop closure.

pub mod eval;
pub mod odometry;
