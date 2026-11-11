//! Scoring an estimated trajectory against ground truth.
//!
//! Three steps, in the order they're used:
//! 1. **Time association** ([`associate`]): the estimate and the truth are
//!    sampled at different moments (wheels every 100 ms, the camera about 30
//!    times a second), so the estimate is interpolated to each truth stamp.
//! 2. **Alignment** ([`align`]): the rigid motion (rotation and shift) that
//!    best lays the estimate over the truth, by least squares. Use it when the
//!    estimate lives in its own frame; skip it when it started from the true
//!    pose, so a wrong start counts as error.
//! 3. **Errors**: [`ate`] (absolute trajectory error: how far each estimated
//!    pose is from the truth, after alignment) and [`rpe`] (relative pose
//!    error: how wrong each stretch of a given length is, which is drift,
//!    independent of where earlier errors left the robot).

use robot::Pose2;

/// A pose and its timestamp in nanoseconds, on one shared clock.
pub type Timed = (u64, Pose2);

/// Pairs each truth pose with the estimate interpolated to its stamp.
/// Truth stamps outside the estimate's span, or with no estimate within
/// `max_gap_ns` on both sides, are skipped. Both inputs must be in time
/// order.
pub fn associate(est: &[Timed], truth: &[Timed], max_gap_ns: u64) -> Vec<(Pose2, Pose2)> {
    let mut pairs = Vec::new();
    let mut i = 0;
    for &(t, true_pose) in truth {
        while i + 1 < est.len() && est[i + 1].0 <= t {
            i += 1;
        }
        let Some(&(t0, a)) = est.get(i) else { break };
        let Some(&(t1, b)) = est.get(i + 1) else {
            if t0 == t {
                pairs.push((a, true_pose));
            }
            continue;
        };
        if t < t0 || t > t1 || t - t0 > max_gap_ns || t1 - t > max_gap_ns {
            continue;
        }
        let f = if t1 == t0 {
            0.0
        } else {
            (t - t0) as f64 / (t1 - t0) as f64
        };
        let dtheta = Pose2::new(0.0, 0.0, b.theta - a.theta).theta;
        let at = Pose2::new(
            a.x + (b.x - a.x) * f,
            a.y + (b.y - a.y) * f,
            a.theta + dtheta * f,
        );
        pairs.push((at, true_pose));
    }
    pairs
}

/// The rigid motion that best maps the estimate onto the truth, in the
/// least-squares sense over positions (2D Umeyama without scale). Apply it
/// with `motion.compose(&estimate)`.
pub fn align(pairs: &[(Pose2, Pose2)]) -> Option<Pose2> {
    if pairs.is_empty() {
        return None;
    }
    let n = pairs.len() as f64;
    let (mut ex, mut ey, mut tx, mut ty) = (0.0, 0.0, 0.0, 0.0);
    for (e, t) in pairs {
        ex += e.x;
        ey += e.y;
        tx += t.x;
        ty += t.y;
    }
    let (ex, ey, tx, ty) = (ex / n, ey / n, tx / n, ty / n);
    // The rotation that best turns the centred estimate onto the centred
    // truth: the angle of Σ (truth × estimate) as a complex product.
    let (mut dot, mut cross) = (0.0, 0.0);
    for (e, t) in pairs {
        let (ax, ay, bx, by) = (e.x - ex, e.y - ey, t.x - tx, t.y - ty);
        dot += ax * bx + ay * by;
        cross += ax * by - ay * bx;
    }
    let theta = if pairs.len() < 2 {
        0.0
    } else {
        cross.atan2(dot)
    };
    let (s, c) = theta.sin_cos();
    Some(Pose2::new(
        tx - (c * ex - s * ey),
        ty - (s * ex + c * ey),
        theta,
    ))
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ate {
    /// Root mean square position error, metres.
    pub rmse_m: f64,
    pub max_m: f64,
    /// Root mean square heading error, radians.
    pub heading_rmse_rad: f64,
    pub pairs: usize,
}

/// Absolute trajectory error over associated pairs, after mapping each
/// estimate through `motion` (from [`align`], or `Pose2::default()` for
/// none).
pub fn ate(pairs: &[(Pose2, Pose2)], motion: Pose2) -> Option<Ate> {
    if pairs.is_empty() {
        return None;
    }
    let (mut sq, mut max, mut hsq) = (0.0f64, 0.0f64, 0.0);
    for (e, t) in pairs {
        let e = motion.compose(e);
        let d2 = (e.x - t.x).powi(2) + (e.y - t.y).powi(2);
        sq += d2;
        max = max.max(d2.sqrt());
        hsq += Pose2::new(0.0, 0.0, e.theta - t.theta).theta.powi(2);
    }
    let n = pairs.len() as f64;
    Some(Ate {
        rmse_m: (sq / n).sqrt(),
        max_m: max,
        heading_rmse_rad: (hsq / n).sqrt(),
        pairs: pairs.len(),
    })
}

/// Absolute trajectory error as plain RMS position error over pairs that are
/// already associated and in the same frame.
pub fn ate_rmse(pairs: &[(Pose2, Pose2)]) -> Option<f64> {
    ate(pairs, Pose2::default()).map(|a| a.rmse_m)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rpe {
    /// The stretch length the errors are measured over, metres of true travel.
    pub over_m: f64,
    /// Root mean square position error per stretch, metres.
    pub rmse_m: f64,
    /// Root mean square heading error per stretch, radians.
    pub heading_rmse_rad: f64,
    pub stretches: usize,
}

/// `b` seen from `a`: where `b` is relative to `a`'s position and heading.
fn between(a: &Pose2, b: &Pose2) -> Pose2 {
    let (s, c) = a.theta.sin_cos();
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    Pose2::new(c * dx + s * dy, -s * dx + c * dy, b.theta - a.theta)
}

/// Relative pose error over stretches of `over_m` metres of true travel:
/// for each start pair, the first later pair at least that far along the
/// true path. Turning on the spot adds no travel, so a stretch can hold
/// spins too.
pub fn rpe(pairs: &[(Pose2, Pose2)], over_m: f64) -> Option<Rpe> {
    let mut along = Vec::with_capacity(pairs.len());
    let mut d = 0.0;
    for (i, (_, t)) in pairs.iter().enumerate() {
        if i > 0 {
            let p = pairs[i - 1].1;
            d += (t.x - p.x).hypot(t.y - p.y);
        }
        along.push(d);
    }
    let (mut sq, mut hsq, mut n) = (0.0, 0.0, 0usize);
    let mut j = 0;
    for i in 0..pairs.len() {
        j = j.max(i);
        while j < pairs.len() && along[j] - along[i] < over_m {
            j += 1;
        }
        if j == pairs.len() {
            break;
        }
        let de = between(&pairs[i].0, &pairs[j].0);
        let dt = between(&pairs[i].1, &pairs[j].1);
        let err = between(&dt, &de);
        sq += err.x.powi(2) + err.y.powi(2);
        hsq += err.theta.powi(2);
        n += 1;
    }
    (n > 0).then(|| Rpe {
        over_m,
        rmse_m: (sq / n as f64).sqrt(),
        heading_rmse_rad: (hsq / n as f64).sqrt(),
        stretches: n,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

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

    #[test]
    fn association_interpolates_to_truth_stamps() {
        let est = [
            (0, Pose2::new(0.0, 0.0, 0.0)),
            (100, Pose2::new(1.0, 0.0, 0.2)),
        ];
        let truth = [(25, Pose2::default()), (300, Pose2::default())];
        let pairs = associate(&est, &truth, 1_000);
        assert_eq!(pairs.len(), 1, "the stamp past the estimate is skipped");
        assert!(close(pairs[0].0.x, 0.25) && close(pairs[0].0.theta, 0.05));
        assert!(
            associate(&est, &truth, 50).is_empty(),
            "too far from both samples"
        );
    }

    #[test]
    fn association_interpolates_heading_across_pi() {
        let est = [
            (0, Pose2::new(0.0, 0.0, 3.1)),
            (100, Pose2::new(0.0, 0.0, -3.1)),
        ];
        let pairs = associate(&est, &[(50, Pose2::default())], 1_000);
        assert!(pairs[0].0.theta.abs() > 3.1, "{}", pairs[0].0.theta);
    }

    #[test]
    fn alignment_undoes_a_rigid_motion() {
        let motion = Pose2::new(0.4, -0.2, 0.3);
        let truth: Vec<Pose2> = (0..20)
            .map(|i| Pose2::new((i as f64 * 0.3).cos(), i as f64 * 0.1, i as f64 * 0.05))
            .collect();
        let inv = between(&motion, &Pose2::default());
        let pairs: Vec<_> = truth.iter().map(|t| (inv.compose(t), *t)).collect();
        let found = align(&pairs).unwrap();
        assert!(
            close(found.x, 0.4) && close(found.y, -0.2) && close(found.theta, 0.3),
            "{found:?}"
        );
        let a = ate(&pairs, found).unwrap();
        assert!(a.rmse_m < 1e-9 && a.heading_rmse_rad < 1e-9);
        assert!(ate(&pairs, Pose2::default()).unwrap().rmse_m > 0.1);
    }

    #[test]
    fn rpe_sees_drift_but_not_a_constant_offset() {
        // A constant offset is no drift.
        let line = |i: usize| Pose2::new(i as f64 * 0.1, 0.0, 0.0);
        let offset: Vec<_> = (0..30)
            .map(|i| (Pose2::new(line(i).x, 0.5, 0.0), line(i)))
            .collect();
        assert!(rpe(&offset, 1.0).unwrap().rmse_m < 1e-9);
        // An estimate that runs 10% long errs 10 cm per metre.
        let long: Vec<_> = (0..30)
            .map(|i| (Pose2::new(line(i).x * 1.1, 0.0, 0.0), line(i)))
            .collect();
        let r = rpe(&long, 1.0).unwrap();
        assert!((r.rmse_m - 0.1).abs() < 1e-9, "{r:?}");
        assert_eq!(rpe(&long[..5], 1.0), None, "no stretch is long enough");
    }
}
