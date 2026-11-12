//! Undoing the smear a moving robot puts in a lidar scan.
//!
//! The head takes a whole rotation (about 200 ms) to measure its beams, and
//! the robot keeps moving meanwhile, but every beam is reported as if it
//! were taken from one place. At 45 degrees a second the robot turns about
//! 9 degrees during one sweep. De-skewing gives each beam its own time
//! within the sweep, asks where the robot was then, and re-expresses the
//! point from where the robot was at the end of the sweep, which is the
//! time the scan is stamped with.

use robot::{LidarConfig, LidarScan, Pose2};

/// How long before the end of the sweep beam `i` of `n` was measured, as a
/// fraction of the sweep. The Neato's head runs up through the beams, from
/// beam 0 to the last: de-skewing that way puts the lidar's heading on the
/// wheels' through a spin, while the other way scatters it by several
/// degrees (drive test, 2026-11-12).
fn age(i: usize, n: usize) -> f64 {
    1.0 - (i as f64 + 0.5) / n as f64
}

/// The inverse of a pose: `inverse(p).compose(&p)` is the identity.
pub fn inverse(p: &Pose2) -> Pose2 {
    let (s, c) = p.theta.sin_cos();
    Pose2::new(-(c * p.x + s * p.y), -(-s * p.x + c * p.y), -p.theta)
}

/// Re-expresses `scan` as if every beam were measured at the end of the
/// sweep. `motion(age)` gives the robot's pose `age` seconds before the end,
/// relative to its pose at the end, for example from interpolated
/// odometry. The moved points are put back into beams by angle, keeping
/// the nearest where two land in one beam, so a few beams can come out
/// empty.
pub fn deskew(scan: &LidarScan, mount: &LidarConfig, motion: impl Fn(f64) -> Pose2) -> LidarScan {
    let n = scan.ranges.len();
    let lidar = Pose2::new(mount.x_m, mount.y_m, mount.theta_rad);
    let from_robot = inverse(&lidar);
    let mut ranges = vec![f32::NAN; n];
    let inc = scan.angle_increment as f64;
    for (i, &r) in scan.ranges.iter().enumerate() {
        if !r.is_finite() {
            continue;
        }
        let a = scan.angle_min as f64 + i as f64 * inc;
        let then = motion(age(i, n) * scan.scan_duration as f64);
        let (rx, ry) = lidar.transform_point(r as f64 * a.cos(), r as f64 * a.sin());
        let (ex, ey) = then.transform_point(rx, ry);
        let (lx, ly) = from_robot.transform_point(ex, ey);
        let k = ((ly.atan2(lx) - scan.angle_min as f64) / inc).round();
        let k = k.rem_euclid(n as f64) as usize % n;
        let range = lx.hypot(ly) as f32;
        if ranges[k].is_nan() || range < ranges[k] {
            ranges[k] = range;
        }
    }
    LidarScan {
        ranges,
        intensities: None,
        ..scan.clone()
    }
}

/// Linear interpolation of a pose track sorted by time, holding the ends.
pub fn pose_at(track: &[(f64, Pose2)], t: f64) -> Option<Pose2> {
    let i = track.partition_point(|(tt, _)| *tt <= t);
    match (i.checked_sub(1).and_then(|j| track.get(j)), track.get(i)) {
        (Some(&(t0, a)), Some(&(t1, b))) => {
            let f = if t1 > t0 { (t - t0) / (t1 - t0) } else { 0.0 };
            let dtheta = robot::normalize_angle(b.theta - a.theta);
            Some(Pose2::new(
                a.x + f * (b.x - a.x),
                a.y + f * (b.y - a.y),
                a.theta + f * dtheta,
            ))
        }
        (Some(&(_, a)), None) => Some(a),
        (None, Some(&(_, b))) => Some(b),
        (None, None) => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mount() -> LidarConfig {
        LidarConfig {
            x_m: -0.1,
            y_m: 0.0,
            theta_rad: 0.0,
            beams: 360,
            min_range_m: 0.1,
            max_range_m: 5.0,
            scan_hz: 5.0,
        }
    }

    /// A robot spinning on the spot at `omega` rad/s scans one fixed point;
    /// de-skewed, the point lands where the robot sees it at the sweep end.
    #[test]
    fn deskew_puts_a_point_where_it_is_at_the_end() {
        let mount = mount();
        let lidar = Pose2::new(mount.x_m, mount.y_m, 0.0);
        let omega = 45f64.to_radians();
        let period = 0.2;
        // Behind and to the left, so it falls mid-sweep.
        let world = (-1.4, 0.6);
        {
            // Robot heading at time `age` before the end (end heading 0).
            let heading = |age: f64| -omega * age;
            // Find the beam that sees the point, given its own time.
            let mut scan = LidarScan {
                angle_min: 0.0,
                angle_increment: 1f32.to_radians(),
                ranges: vec![f32::NAN; 360],
                intensities: None,
                scan_duration: period as f32,
            };
            for i in 0..360 {
                let age = age(i, 360) * period;
                let robot = Pose2::new(0.0, 0.0, heading(age));
                let seen = inverse(&robot.compose(&lidar)).transform_point(world.0, world.1);
                let k = (seen.1.atan2(seen.0).to_degrees().round() as i64).rem_euclid(360) as usize;
                if k == i {
                    scan.ranges[i] = seen.0.hypot(seen.1) as f32;
                }
            }
            assert!(scan.ranges.iter().any(|r| r.is_finite()));
            let fixed = deskew(&scan, &mount, |age| Pose2::new(0.0, 0.0, heading(age)));
            let at_end = inverse(&lidar).transform_point(world.0, world.1);
            let want =
                (at_end.1.atan2(at_end.0).to_degrees().round() as i64).rem_euclid(360) as usize;
            // Beams are 1 degree wide, so the point may land one beam over.
            let near = [359, 0, 1].map(|d| fixed.ranges[(want + d) % 360]);
            assert!(
                near.iter()
                    .any(|&r| (r as f64 - at_end.0.hypot(at_end.1)).abs() < 0.02),
                "beams around {want} hold {near:?}"
            );
            // Without de-skewing it would be several beams away.
            let raw = scan.ranges.iter().position(|r| r.is_finite()).unwrap();
            assert!(
                (raw as i64 - want as i64).rem_euclid(360) >= 3,
                "{raw} vs {want}"
            );
        }
    }

    #[test]
    fn pose_at_interpolates_across_the_wrap() {
        let track = [
            (0.0, Pose2::new(0.0, 0.0, 3.1)),
            (1.0, Pose2::new(1.0, 0.0, -3.1)),
        ];
        let p = pose_at(&track, 0.5).unwrap();
        assert!((p.x - 0.5).abs() < 1e-9);
        assert!(p.theta.abs() > 3.1);
    }
}
