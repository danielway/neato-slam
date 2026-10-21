//! SLAM algorithms. Pure computation over `Stamped<Msg>` streams with no I/O,
//! so the same code runs against the robot, replay and the simulator.
//!
//! Planned progression, each step scored against ground truth before the next:
//! wheel odometry, scan matching, occupancy grid mapping, particle-filter
//! SLAM, then pose-graph SLAM with loop closure.

pub mod odometry;
