//! The Neato's USB serial test-mode protocol.
//!
//! The robot answers text commands with CSV-like tables terminated by Ctrl-Z
//! (0x1A). This crate only builds command strings and parses responses; it
//! does no I/O, so it can be tested without hardware.
//!
//! Formats follow Neato's programmer manual. The test fixtures are written
//! from that manual, not captured from Daniel's robot yet, so check them
//! against real output once `pi-node` can talk to it.

use std::fmt;

use robot::{Contacts, LidarScan, WheelPositions};

/// Marks the end of each response.
pub const END_OF_RESPONSE: char = '\u{1a}';

pub mod cmd {
    pub const TEST_MODE_ON: &str = "TestMode On";
    pub const TEST_MODE_OFF: &str = "TestMode Off";
    pub const LDS_ON: &str = "SetLDSRotation On";
    pub const LDS_OFF: &str = "SetLDSRotation Off";
    pub const GET_LDS_SCAN: &str = "GetLDSScan";
    pub const GET_MOTORS: &str = "GetMotors";
    pub const GET_DIGITAL_SENSORS: &str = "GetDigitalSensors";
    pub const GET_VERSION: &str = "GetVersion";

    /// Drives each wheel a distance in mm at up to `speed_mm_s`. A zero
    /// distance on both wheels stops the robot.
    pub fn set_motor(left_mm: i32, right_mm: i32, speed_mm_s: u32) -> String {
        format!("SetMotor LWheelDist {left_mm} RWheelDist {right_mm} Speed {speed_mm_s}")
    }

    /// Converts wheel speeds into a `SetMotor` that runs for about
    /// `horizon_s`. The Pi re-sends it every control tick, so the robot
    /// keeps moving only while commands keep arriving.
    pub fn wheel_speeds(left_mm_s: f32, right_mm_s: f32, horizon_s: f32) -> String {
        let speed = left_mm_s.abs().max(right_mm_s.abs());
        set_motor(
            (left_mm_s * horizon_s).round() as i32,
            (right_mm_s * horizon_s).round() as i32,
            speed.round() as u32,
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    MissingField(&'static str),
    BadValue { field: String, value: String },
    WrongBeamCount(usize),
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParseError::MissingField(name) => write!(f, "response has no {name} field"),
            ParseError::BadValue { field, value } => write!(f, "bad value {value:?} for {field}"),
            ParseError::WrongBeamCount(n) => write!(f, "expected 360 lidar readings, got {n}"),
        }
    }
}

impl std::error::Error for ParseError {}

/// Splits a response into trimmed, comma-separated rows, dropping blank
/// lines, the end-of-response marker and an echoed command line.
fn rows(response: &str) -> impl Iterator<Item = Vec<&str>> {
    response
        .lines()
        .map(|l| l.trim_matches(|c: char| c.is_whitespace() || c == END_OF_RESPONSE))
        .filter(|l| l.contains(','))
        .map(|l| l.split(',').map(str::trim).collect())
}

fn parse_num<T: std::str::FromStr>(field: &str, value: &str) -> Result<T, ParseError> {
    value.parse().map_err(|_| ParseError::BadValue {
        field: field.to_owned(),
        value: value.to_owned(),
    })
}

fn lookup<'a>(response: &'a str, field: &'static str) -> Result<&'a str, ParseError> {
    response
        .lines()
        .filter_map(|l| l.trim().split_once(','))
        .find(|(name, _)| name.trim() == field)
        .map(|(_, value)| value.trim().trim_end_matches(END_OF_RESPONSE))
        .ok_or(ParseError::MissingField(field))
}

/// Parses `GetLDSScan`: 360 rows of `angle_deg,dist_mm,intensity,error_hex`
/// followed by `ROTATION_SPEED,<hz>`.
pub fn parse_lds_scan(response: &str) -> Result<LidarScan, ParseError> {
    let mut ranges = vec![f32::NAN; 360];
    let mut intensities = vec![0u16; 360];
    let mut readings = 0;
    for row in rows(response) {
        let [angle, dist, intensity, error] = row[..] else {
            continue;
        };
        // Skips the header row, whose first column isn't a number.
        let Ok(angle) = angle.parse::<usize>() else {
            continue;
        };
        if angle >= 360 {
            return Err(ParseError::BadValue {
                field: "AngleInDegrees".into(),
                value: angle.to_string(),
            });
        }
        let dist: u32 = parse_num("DistInMM", dist)?;
        let error = u32::from_str_radix(error, 16).map_err(|_| ParseError::BadValue {
            field: "ErrorCodeHEX".into(),
            value: error.to_owned(),
        })?;
        intensities[angle] = parse_num("Intensity", intensity)?;
        if error == 0 && dist > 0 {
            ranges[angle] = dist as f32 / 1000.0;
        }
        readings += 1;
    }
    if readings != 360 {
        return Err(ParseError::WrongBeamCount(readings));
    }
    let hz: f32 = parse_num("ROTATION_SPEED", lookup(response, "ROTATION_SPEED")?)?;
    Ok(LidarScan {
        angle_min: 0.0,
        angle_increment: 1f32.to_radians(),
        ranges,
        intensities: Some(intensities),
        scan_duration: if hz > 0.0 { 1.0 / hz } else { 0.0 },
    })
}

/// Parses the wheel positions out of `GetMotors`.
pub fn parse_motors(response: &str) -> Result<WheelPositions, ParseError> {
    let left = lookup(response, "LeftWheel_PositionInMM")?;
    let right = lookup(response, "RightWheel_PositionInMM")?;
    Ok(WheelPositions {
        left_mm: parse_num("LeftWheel_PositionInMM", left)?,
        right_mm: parse_num("RightWheel_PositionInMM", right)?,
    })
}

/// Parses bumpers and wheel-drop switches out of `GetDigitalSensors`.
pub fn parse_digital_sensors(response: &str) -> Result<Contacts, ParseError> {
    let bit = |field: &'static str, mask: u8| -> Result<u8, ParseError> {
        let on: u8 = parse_num(field, lookup(response, field)?)?;
        Ok(if on != 0 { mask } else { 0 })
    };
    Ok(Contacts {
        bumpers: bit("LSIDEBIT", Contacts::LEFT_SIDE)?
            | bit("LFRONTBIT", Contacts::LEFT_FRONT)?
            | bit("RFRONTBIT", Contacts::RIGHT_FRONT)?
            | bit("RSIDEBIT", Contacts::RIGHT_SIDE)?,
        wheel_drop: bit("SNSR_LEFT_WHEEL_EXTENDED", Contacts::LEFT_WHEEL)?
            | bit("SNSR_RIGHT_WHEEL_EXTENDED", Contacts::RIGHT_WHEEL)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lds_response() -> String {
        let mut s =
            String::from("GetLDSScan\r\nAngleInDegrees,DistInMM,Intensity,ErrorCodeHEX\r\n");
        for angle in 0..360 {
            // Angle 90 reports an error, so it should come back as NaN.
            let error = if angle == 90 { "8035" } else { "0" };
            s += &format!("{angle},{},{},{error}\r\n", 1000 + angle, 100 + angle);
        }
        s += "ROTATION_SPEED,5.00\r\n\u{1a}";
        s
    }

    #[test]
    fn lds_scan_parses() {
        let scan = parse_lds_scan(&lds_response()).unwrap();
        assert_eq!(scan.ranges.len(), 360);
        assert_eq!(scan.ranges[0], 1.0);
        assert_eq!(scan.ranges[359], 1.359);
        assert!(scan.ranges[90].is_nan());
        assert_eq!(scan.intensities.as_ref().unwrap()[10], 110);
        assert!((scan.scan_duration - 0.2).abs() < 1e-6);
    }

    #[test]
    fn short_lds_scan_is_rejected() {
        let short = "AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX\r\n0,100,10,0\r\nROTATION_SPEED,5.0\r\n";
        assert_eq!(parse_lds_scan(short), Err(ParseError::WrongBeamCount(1)));
    }

    #[test]
    fn motors_parse() {
        let response = "Parameter,Value\r\nBrush_RPM,0\r\nLeftWheel_RPM,0\r\nLeftWheel_PositionInMM,-469\r\nLeftWheel_Speed,0\r\nRightWheel_PositionInMM,512\r\n\u{1a}";
        assert_eq!(
            parse_motors(response).unwrap(),
            WheelPositions {
                left_mm: -469,
                right_mm: 512
            }
        );
    }

    #[test]
    fn digital_sensors_parse() {
        let response = "Digital Sensor Name, Value\r\nSNSR_DC_JACK_CONNECT,0\r\nSNSR_LEFT_WHEEL_EXTENDED,1\r\nSNSR_RIGHT_WHEEL_EXTENDED,0\r\nLSIDEBIT,0\r\nLFRONTBIT,1\r\nRSIDEBIT,0\r\nRFRONTBIT,1\r\n\u{1a}";
        let c = parse_digital_sensors(response).unwrap();
        assert_eq!(c.bumpers, Contacts::LEFT_FRONT | Contacts::RIGHT_FRONT);
        assert_eq!(c.wheel_drop, Contacts::LEFT_WHEEL);
    }

    #[test]
    fn wheel_speeds_become_set_motor() {
        assert_eq!(
            cmd::wheel_speeds(100.0, -50.0, 0.5),
            "SetMotor LWheelDist 50 RWheelDist -25 Speed 100"
        );
    }
}
