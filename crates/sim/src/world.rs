/// A wall, as a line segment between two points in metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment {
    pub a: (f64, f64),
    pub b: (f64, f64),
}

/// The static map: a set of wall segments. Its segments are also the
/// ground-truth map for scoring.
#[derive(Debug, Clone, Default)]
pub struct World {
    pub segments: Vec<Segment>,
}

fn cross(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.1 - a.1 * b.0
}

impl World {
    /// An empty room with corners at (0, 0) and (width, height).
    pub fn rectangle(width: f64, height: f64) -> Self {
        let c = [(0.0, 0.0), (width, 0.0), (width, height), (0.0, height)];
        let segments = (0..4)
            .map(|i| Segment {
                a: c[i],
                b: c[(i + 1) % 4],
            })
            .collect();
        Self { segments }
    }

    /// Distance to the nearest wall along a ray, if one is within `max_range`.
    pub fn raycast(&self, origin: (f64, f64), angle: f64, max_range: f64) -> Option<f64> {
        let dir = (angle.cos(), angle.sin());
        self.segments
            .iter()
            .filter_map(|s| {
                let edge = (s.b.0 - s.a.0, s.b.1 - s.a.1);
                let denom = cross(dir, edge);
                if denom.abs() < 1e-12 {
                    return None;
                }
                let to_a = (s.a.0 - origin.0, s.a.1 - origin.1);
                let t = cross(to_a, edge) / denom;
                let u = cross(to_a, dir) / denom;
                (t >= 0.0 && (0.0..=1.0).contains(&u)).then_some(t)
            })
            .filter(|&t| t <= max_range)
            .min_by(f64::total_cmp)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn raycast_hits_nearest_wall() {
        let w = World::rectangle(4.0, 3.0);
        let hit = w.raycast((1.0, 1.0), 0.0, 10.0).unwrap();
        assert!((hit - 3.0).abs() < 1e-12);
        let up = w.raycast((1.0, 1.0), FRAC_PI_2, 10.0).unwrap();
        assert!((up - 2.0).abs() < 1e-12);
        assert_eq!(w.raycast((1.0, 1.0), 0.0, 2.0), None);
    }
}
