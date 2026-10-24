use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use neato::END_OF_RESPONSE;

/// Longest a reply may take. A full lidar scan is the slowest, at about
/// 360 lines.
const REPLY_TIMEOUT: Duration = Duration::from_secs(3);

/// The Neato's USB serial port, one command and reply at a time.
pub struct NeatoPort {
    file: File,
    /// Bytes read past the end of the last reply.
    pending: Vec<u8>,
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
        let mut port = Self {
            file,
            pending: Vec::new(),
        };
        // A bare newline ends any half-typed line left in the Neato's input
        // from earlier, so our first command isn't glued onto it.
        port.file.write_all(b"\n")?;
        thread::sleep(Duration::from_millis(300));
        port.drain();
        Ok(port)
    }

    /// Discards anything left over from before, such as a half-read reply.
    fn drain(&mut self) {
        let mut buf = [0u8; 4096];
        while matches!(self.file.read(&mut buf), Ok(n) if n > 0) {}
        self.pending.clear();
    }

    /// Sends a command and returns its reply, without the end marker.
    /// Replies that echo a different command are left over from an earlier
    /// exchange; they are logged and skipped.
    pub fn query(&mut self, command: &str) -> Result<String> {
        self.file
            .write_all(format!("{command}\n").as_bytes())
            .with_context(|| format!("sending {command}"))?;
        let deadline = Instant::now() + REPLY_TIMEOUT;
        let mut buf = [0u8; 4096];
        loop {
            while let Some(end) = self
                .pending
                .iter()
                .position(|&b| b == END_OF_RESPONSE as u8)
            {
                let reply: Vec<u8> = self.pending.drain(..=end).take(end).collect();
                let reply = String::from_utf8_lossy(&reply).into_owned();
                if !neato::echoes_other_command(command, &reply) {
                    return Ok(reply);
                }
                let head: String = reply.chars().take(80).collect();
                eprintln!("skipping a stale reply while waiting for {command}: {head:?}");
            }
            if Instant::now() >= deadline {
                bail!("no complete reply to {command} within {REPLY_TIMEOUT:?}");
            }
            let n = self.file.read(&mut buf).context("reading from the Neato")?;
            self.pending.extend_from_slice(&buf[..n]);
        }
    }
}
