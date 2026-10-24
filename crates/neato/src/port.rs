use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::process::Command;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use neato::END_OF_RESPONSE;

/// Longest a reply may take. A full lidar scan is the slowest, at about
/// 360 lines.
const REPLY_TIMEOUT: Duration = Duration::from_secs(3);

/// The Neato's USB serial port, one command and reply at a time.
pub struct NeatoPort {
    file: File,
}

impl NeatoPort {
    pub fn open(path: &str) -> Result<Self> {
        // Raw mode with reads that give up after 0.2 s of silence (`time` is
        // in tenths), so a missing end marker can't hang the loop. Uses the
        // Pi's own stty rather than another crate.
        let status = Command::new("stty")
            .args([
                "-F", path, "115200", "raw", "-echo", "min", "0", "time", "2",
            ])
            .status()
            .context("running stty")?;
        if !status.success() {
            bail!(
                "stty could not configure {path}; is the Neato plugged in and is this user in the dialout group?"
            );
        }
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .with_context(|| format!("opening {path}"))?;
        let mut port = Self { file };
        port.drain();
        Ok(port)
    }

    /// Discards anything left over from before, such as a half-read reply.
    fn drain(&mut self) {
        let mut buf = [0u8; 4096];
        while matches!(self.file.read(&mut buf), Ok(n) if n > 0) {}
    }

    /// Sends a command and returns its reply, without the end marker.
    pub fn query(&mut self, command: &str) -> Result<String> {
        self.file
            .write_all(format!("{command}\n").as_bytes())
            .with_context(|| format!("sending {command}"))?;
        let deadline = Instant::now() + REPLY_TIMEOUT;
        let mut reply = Vec::with_capacity(8192);
        let mut buf = [0u8; 4096];
        loop {
            let n = self.file.read(&mut buf).context("reading from the Neato")?;
            if let Some(end) = buf[..n].iter().position(|&b| b == END_OF_RESPONSE as u8) {
                reply.extend_from_slice(&buf[..end]);
                return Ok(String::from_utf8_lossy(&reply).into_owned());
            }
            reply.extend_from_slice(&buf[..n]);
            if Instant::now() >= deadline {
                bail!("no complete reply to {command} within {REPLY_TIMEOUT:?}");
            }
        }
    }
}
