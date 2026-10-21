//! Recording runs to MCAP and reading them back for replay.
//!
//! Planned layout of a log: one channel per message type (lidar, wheels,
//! contacts, status), one for commands sent, one for ground truth when the
//! source has it, and the run's `robot.toml` as metadata. Sim runs and real
//! runs use the same layout, so everything downstream treats them alike.
//!
//! Not implemented yet; until then `robot_io::ReplayBackend` replays
//! in-memory message lists.
