//! A scripted drive, the same every time, for measuring how the robot
//! follows commands: smoothness while moving, how long it takes to stop,
//! and how odometry holds up through spins. Started from the page; any
//! drive key cancels it.

use std::time::Duration;

use robot::Command;

/// One step: what it is called on the page, how long it lasts, and the
/// wheel speeds in mm/s (left, right), or `None` for a stop.
type Step = (&'static str, f32, Option<(f32, f32)>);

const STRAIGHT_MM_S: f32 = 150.0;
const SPIN_MM_S: f32 = 100.0;

/// 24 s. Forward and back cancel out, so it ends about where it
/// started and can be run from the dock and repeated.
const STEPS: &[Step] = &[
    ("forward", 2.0, Some((STRAIGHT_MM_S, STRAIGHT_MM_S))),
    ("stop", 2.0, None),
    ("reverse", 2.0, Some((-STRAIGHT_MM_S, -STRAIGHT_MM_S))),
    ("stop", 2.0, None),
    ("spin left", 3.0, Some((-SPIN_MM_S, SPIN_MM_S))),
    ("stop", 2.0, None),
    ("spin right", 3.0, Some((SPIN_MM_S, -SPIN_MM_S))),
    ("stop", 2.0, None),
    ("tap forward", 0.3, Some((STRAIGHT_MM_S, STRAIGHT_MM_S))),
    ("stop", 1.2, None),
    ("tap forward", 0.3, Some((STRAIGHT_MM_S, STRAIGHT_MM_S))),
    ("stop", 1.2, None),
    ("tap back", 0.3, Some((-STRAIGHT_MM_S, -STRAIGHT_MM_S))),
    ("stop", 1.2, None),
    ("tap back", 0.3, Some((-STRAIGHT_MM_S, -STRAIGHT_MM_S))),
    ("stop", 1.2, None),
];

/// The command and step name at `elapsed` into the test, or `None` once it
/// is over.
pub fn at(elapsed: Duration) -> Option<(Command, &'static str)> {
    let mut t = elapsed.as_secs_f32();
    for &(name, secs, speeds) in STEPS {
        if t < secs {
            let command = match speeds {
                Some((left_mm_s, right_mm_s)) => Command::WheelSpeeds {
                    left_mm_s,
                    right_mm_s,
                },
                None => Command::Stop,
            };
            return Some((command, name));
        }
        t -= secs;
    }
    None
}

/// How long the whole test takes.
pub fn length() -> Duration {
    Duration::from_secs_f32(STEPS.iter().map(|s| s.1).sum())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn walks_the_steps_and_ends() {
        assert_eq!(at(Duration::ZERO).unwrap().1, "forward");
        assert_eq!(
            at(Duration::from_secs_f32(2.5)).unwrap(),
            (Command::Stop, "stop")
        );
        assert_eq!(at(Duration::from_secs_f32(4.5)).unwrap().1, "reverse");
        assert!(at(length() + Duration::from_millis(1)).is_none());
        // The straight moves and the taps cancel out in distance.
        let net: f32 = STEPS
            .iter()
            .filter_map(|s| s.2.filter(|(l, r)| l == r).map(|(l, _)| l * s.1))
            .sum();
        assert!(net.abs() < 1.0, "{net}");
    }
}
