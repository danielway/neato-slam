use robot::Command;
use serde::Deserialize;

/// Wheel speed for driving straight, mm/s. Shift drives at the robot's limit.
const DRIVE_MM_S: f32 = 200.0;
/// Speed difference between the wheels when turning, mm/s.
const TURN_MM_S: f32 = 100.0;

/// The drive keys held on the page.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
pub struct Keys {
    pub forward: bool,
    pub back: bool,
    pub left: bool,
    pub right: bool,
    pub fast: bool,
}

/// Maps held keys to a wheel-speed command. No keys means `Stop`, so the
/// robot halts as soon as the keys are released.
pub fn command(keys: Keys, max_mm_s: f32) -> Command {
    let axis = |pos: bool, neg: bool| pos as i8 as f32 - neg as i8 as f32;
    let drive = axis(keys.forward, keys.back);
    let turn = axis(keys.left, keys.right);
    if drive == 0.0 && turn == 0.0 {
        return Command::Stop;
    }
    let scale = if keys.fast {
        max_mm_s / DRIVE_MM_S
    } else {
        1.0
    };
    let v = drive * DRIVE_MM_S * scale;
    let w = turn * TURN_MM_S * scale;
    let clamp = |s: f32| s.clamp(-max_mm_s, max_mm_s);
    Command::WheelSpeeds {
        left_mm_s: clamp(v - w),
        right_mm_s: clamp(v + w),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_keys_stops() {
        assert_eq!(command(Keys::default(), 300.0), Command::Stop);
    }

    #[test]
    fn left_turns_counterclockwise() {
        let keys = Keys {
            left: true,
            ..Keys::default()
        };
        let Command::WheelSpeeds {
            left_mm_s,
            right_mm_s,
        } = command(keys, 300.0)
        else {
            panic!("expected wheel speeds");
        };
        assert!(right_mm_s > 0.0 && left_mm_s < 0.0);
    }

    #[test]
    fn fast_respects_the_limit() {
        let keys = Keys {
            forward: true,
            left: true,
            fast: true,
            ..Keys::default()
        };
        let Command::WheelSpeeds { right_mm_s, .. } = command(keys, 300.0) else {
            panic!("expected wheel speeds");
        };
        assert_eq!(right_mm_s, 300.0);
    }
}
