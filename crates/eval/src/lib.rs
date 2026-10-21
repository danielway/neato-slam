//! Scoring estimated trajectories against ground truth.
//!
//! Still to come: time association between estimate and truth, SE(2)
//! alignment before ATE, Relative Pose Error, and TUM-format export so the
//! `evo` tool can plot runs.

use robot_msgs::Pose2;

/// Absolute Trajectory Error: RMS position error over pose pairs that are
/// already time-associated and in the same frame. None if there are no pairs.
pub fn ate_rmse(pairs: &[(Pose2, Pose2)]) -> Option<f64> {
    if pairs.is_empty() {
        return None;
    }
    let sum: f64 = pairs
        .iter()
        .map(|(est, truth)| (est.x - truth.x).powi(2) + (est.y - truth.y).powi(2))
        .sum();
    Some((sum / pairs.len() as f64).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ate_of_constant_offset_is_the_offset() {
        let pairs: Vec<_> = (0..10)
            .map(|i| {
                (
                    Pose2::new(i as f64, 0.3, 0.0),
                    Pose2::new(i as f64, 0.0, 0.0),
                )
            })
            .collect();
        assert!((ate_rmse(&pairs).unwrap() - 0.3).abs() < 1e-12);
        assert_eq!(ate_rmse(&[]), None);
    }
}
