//! The Neato's USB serial test-mode protocol.
//!
//! The robot answers text commands with CSV-like tables terminated by Ctrl-Z
//! (0x1A). This crate only builds command strings and parses responses; it
//! does no I/O, so it can be tested without hardware.
//!
//! Formats follow Neato's programmer manual. The test fixtures are written
//! from that manual, not captured from the robot yet, so check them
//! against real output once `pi-node` can talk to it.

use std::fmt;

use robot::{Contacts, LidarScan, RobotStatus, WheelPositions};

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
    pub const GET_CHARGER: &str = "GetCharger";

    /// Every fixed command pi-node sends, for recognising stale echoes.
    pub const ALL: &[&str] = &[
        TEST_MODE_ON,
        TEST_MODE_OFF,
        LDS_ON,
        LDS_OFF,
        GET_LDS_SCAN,
        GET_MOTORS,
        GET_DIGITAL_SENSORS,
        GET_VERSION,
        GET_CHARGER,
    ];

    /// Drives each wheel a distance in mm at up to `speed_mm_s`. A zero
    /// distance on both wheels stops the robot.
    pub const WHEELS_DISABLE: &str = "SetMotor LWheelDisable RWheelDisable";
    pub const WHEELS_ENABLE: &str = "SetMotor LWheelEnable RWheelEnable";

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

/// Keeps reported wheel positions continuous across the Neato's counter
/// resets. Its counters go back to zero when test mode is switched, so after
/// [`rebase`](Self::rebase) the next reading is taken to continue from the
/// last one reported (the robot is still while test mode changes).
#[derive(Debug, Default)]
pub struct ContinuousWheels {
    offset: (i64, i64),
    last: Option<WheelPositions>,
    rebase: bool,
}

impl ContinuousWheels {
    /// The counters may have jumped: join the next reading to the last.
    pub fn rebase(&mut self) {
        self.rebase = true;
    }

    pub fn next(&mut self, raw: WheelPositions) -> WheelPositions {
        if std::mem::take(&mut self.rebase)
            && let Some(last) = self.last
        {
            self.offset = (last.left_mm - raw.left_mm, last.right_mm - raw.right_mm);
        }
        let out = WheelPositions {
            left_mm: raw.left_mm + self.offset.0,
            right_mm: raw.right_mm + self.offset.1,
        };
        self.last = Some(out);
        out
    }
}

/// Whether a reply belongs to `command`. The Neato echoes each command as
/// the first line of its reply, so a reply that echoes a different command
/// is left over from an earlier exchange and must be skipped, or every reply
/// after it would be read against the wrong command.
pub fn echoes_other_command(command: &str, reply: &str) -> bool {
    let first = reply
        .lines()
        .map(|l| l.trim_matches(|c: char| c.is_whitespace() || c == END_OF_RESPONSE))
        .find(|l| !l.is_empty());
    // Only the echo of another command we send counts. Anything else, such
    // as a table or an error line, belongs to this command.
    first.is_some_and(|line| {
        !line.eq_ignore_ascii_case(command)
            && (cmd::ALL.iter().any(|c| line.eq_ignore_ascii_case(c))
                || line.to_ascii_lowercase().starts_with("setmotor "))
    })
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

/// Parses `GetCharger`.
pub fn parse_charger(response: &str) -> Result<RobotStatus, ParseError> {
    let flag = |field: &'static str| -> Result<bool, ParseError> {
        Ok(parse_num::<u8>(field, lookup(response, field)?)? != 0)
    };
    Ok(RobotStatus {
        battery_v: parse_num("VBattV", lookup(response, "VBattV")?)?,
        fuel_percent: parse_num("FuelPercent", lookup(response, "FuelPercent")?)?,
        charging: flag("ChargingActive")?,
        external_power: flag("ExtPwrPresent")?,
    })
}

/// Works out when the Neato's lidar finishes each sweep, which the firmware
/// doesn't report.
///
/// `GetLDSScan` returns the last finished sweep, and the whole reply changes
/// at once when the next one finishes (about every 200 ms). So a reply
/// that is still the old sweep says the next end is later than its request,
/// and a new one says the end came before it. Each request holds the serial
/// link for about 36 ms, so rather than polling fast the clock keeps a
/// window that must hold the next sweep's end, carries it forward by one
/// rotation each sweep, and asks in the middle of it. Every reply halves
/// the window, so it narrows to a few milliseconds and stays there.
///
/// Times are seconds on the caller's clock, taken when a request was sent.
#[derive(Debug, Default)]
pub struct SweepClock {
    /// Fingerprint of the last reply's beams.
    last: Option<u64>,
    /// The next sweep ends after this...
    after: Option<f64>,
    /// ...and no later than this.
    before: Option<f64>,
    period: f64,
}

impl SweepClock {
    /// How much the window widens each sweep, for the head's speed
    /// wandering.
    const SLACK: f64 = 0.0005;

    pub fn new() -> Self {
        Self {
            period: 0.2,
            ..Self::default()
        }
    }

    /// When to send the next request, or `None` to ask at every chance
    /// while there is no window yet.
    pub fn next_request(&self) -> Option<f64> {
        Some((self.after? + self.before?) / 2.0)
    }

    pub fn due(&self, now: f64) -> bool {
        self.next_request().is_none_or(|t| now >= t)
    }

    /// Forgets the timing, for when the lidar stops or the port reopens.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Records the reply to a request sent at `sent`. Returns when the sweep
    /// in it finished if it holds a new sweep, or `None` if it is the one
    /// seen before.
    pub fn observe(&mut self, sent: f64, reply: &str, hz: f32) -> Option<f64> {
        use std::hash::{Hash, Hasher};
        if (2.0..10.0).contains(&hz) {
            self.period = 1.0 / hz as f64;
        }
        // ROTATION_SPEED updates on its own, so only the beams count.
        let beams = reply.split("ROTATION_SPEED").next().unwrap_or(reply);
        let mut h = std::collections::hash_map::DefaultHasher::new();
        beams.hash(&mut h);
        let print = h.finish();
        let first = self.last.is_none();
        let fresh = !first && self.last != Some(print);
        self.last = Some(print);

        if !fresh {
            // The next end is after this request.
            self.after = Some(self.after.map_or(sent, |a| a.max(sent)));
            if self.before.is_some_and(|b| b <= sent) {
                // The window was wrong; start again from here.
                self.before = None;
            }
            return None;
        }
        // Requests may have skipped whole sweeps: this reply is the latest,
        // so carry the window forward to the last end before the request.
        if let (Some(a), Some(b)) = (self.after.as_mut(), self.before.as_mut()) {
            while *a + self.period < sent {
                *a += self.period;
                *b += self.period;
            }
        }
        let before = self.before.map_or(sent, |b| b.min(sent));
        let after = self.after.filter(|&a| a < before);
        let end = after.map_or(before, |a| (a + before) / 2.0);
        self.after = after.map(|a| a + self.period - Self::SLACK);
        self.before = Some(before + self.period + Self::SLACK);
        Some(end)
    }
}

#[cfg(test)]
mod tests {

    /// A fake lidar whose sweeps end at `phase + k * period`, polled the
    /// way pi-node polls it: at every chance (10 ms apart) without a window,
    /// else when due but never within 40 ms of the last request, which is
    /// about what the other queries take. Returns (estimate, truth) per new
    /// sweep and the number of requests.
    fn run_clock(phase: f64, period: f64) -> (Vec<(f64, f64)>, usize) {
        let mut clock = SweepClock::new();
        let (mut t, mut requests) = (0.0, 0);
        let mut found = Vec::new();
        while t < 20.0 {
            t = match clock.next_request() {
                None => t + 0.01,
                Some(due) => due.max(t + 0.04),
            };
            requests += 1;
            let sweep = ((t - phase) / period).floor();
            let reply = format!("{sweep}\r\nROTATION_SPEED,{}", 1.0 / period);
            if let Some(end) = clock.observe(t, &reply, (1.0 / period) as f32) {
                found.push((end, phase + sweep * period));
            }
            t += 0.036;
        }
        (found, requests)
    }

    #[test]
    fn sweep_clock_locks_onto_sweep_ends() {
        for phase in [0.0, 0.05, 0.123, 0.19] {
            let (found, requests) = run_clock(phase, 0.1987);
            // Every sweep is seen, at well under two requests each.
            assert!(found.len() >= 99, "{phase}: {} sweeps", found.len());
            assert!(requests < found.len() * 2, "{phase}: {requests} requests");
            let worst = found[5..]
                .iter()
                .map(|(est, truth)| (est - truth).abs())
                .fold(0.0, f64::max);
            assert!(worst < 0.004, "phase {phase}: off by up to {worst}");
        }
    }
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
    fn stale_replies_are_recognised_by_their_echo() {
        let motors = "GetMotors\r\nParameter,Value\r\nLeftWheel_PositionInMM,1\r\n";
        assert!(!echoes_other_command("GetMotors", motors));
        assert!(!echoes_other_command("getmotors", motors));
        assert!(echoes_other_command("GetDigitalSensors", motors));
        // No echo at all: a table straight away still counts as ours.
        assert!(!echoes_other_command(
            "GetMotors",
            "Parameter,Value\r\nLeftWheel_PositionInMM,1\r\n"
        ));
        // An error line isn't another command's echo.
        assert!(!echoes_other_command(
            "GetVersion",
            "Unknown Cmd: '***'\r\n"
        ));
        assert!(echoes_other_command("GetVersion", "TestMode On\r\n"));
        assert!(echoes_other_command(
            "GetMotors",
            "SetMotor LWheelDist 30 RWheelDist 30 Speed 100\r\n"
        ));
    }

    #[test]
    fn wheel_speeds_become_set_motor() {
        assert_eq!(
            cmd::wheel_speeds(100.0, -50.0, 0.5),
            "SetMotor LWheelDist 50 RWheelDist -25 Speed 100"
        );
    }

    #[test]
    fn charger_parses() {
        let reply = "GetCharger\r\nLabel,Value\r\nFuelPercent,83\r\nBatteryOverTemp,0\r\n\
            ChargingActive,1\r\nChargingEnabled,1\r\nExtPwrPresent,1\r\n\
            BattTempCAvg[0],27\r\nVBattV,15.96\r\nVExtV,23.10\r\n\u{1a}";
        let s = parse_charger(reply).unwrap();
        assert_eq!(s.fuel_percent, 83);
        assert!((s.battery_v - 15.96).abs() < 1e-4);
        assert!(s.charging);
        assert!(s.external_power);
    }

    #[test]
    fn wheels_stay_continuous_across_a_counter_reset() {
        let w = |l, r| WheelPositions {
            left_mm: l,
            right_mm: r,
        };
        let mut wheels = ContinuousWheels::default();
        assert_eq!(wheels.next(w(1200, 1300)), w(1200, 1300));
        assert_eq!(wheels.next(w(1250, 1350)), w(1250, 1350));
        wheels.rebase();
        assert_eq!(wheels.next(w(0, 0)), w(1250, 1350));
        assert_eq!(wheels.next(w(10, -5)), w(1260, 1345));
    }
}
