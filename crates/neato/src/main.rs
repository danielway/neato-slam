//! Runs on the Raspberry Pi and bridges the Neato to the desktop.
//!
//! Planned loop:
//! 1. Open the Neato's USB serial port, send `TestMode On` and
//!    `SetLDSRotation On`.
//! 2. Accept one desktop connection on the listen address.
//! 3. Poll `GetMotors` and `GetDigitalSensors` at the control rate and
//!    `GetLDSScan` at the scan rate, parse with the `neato` library, stamp each
//!    reading with the monotonic clock when it was taken, and stream them.
//! 4. Turn each `Command` into `SetMotor` via `neato::cmd`.
//! 5. Watchdog: stop the motors if no command arrives within 300 ms or the
//!    connection drops.

use anyhow::{Result, bail};

const USAGE: &str = "usage: pi-node [--serial /dev/ttyACM0] [--listen 0.0.0.0:7878]";

fn main() -> Result<()> {
    let mut serial = String::from("/dev/ttyACM0");
    let mut listen = String::from("0.0.0.0:7878");
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--serial" => serial = args.next().unwrap_or_default(),
            "--listen" => listen = args.next().unwrap_or_default(),
            _ => bail!("unknown argument {arg:?}\n{USAGE}"),
        }
    }
    bail!("not implemented yet: would bridge {serial} to {listen}")
}
