//! Recording the live source to MCAP logs in the recordings folder, and
//! finding the logs there to replay.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{self, BufWriter};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use robot::log::{Entry, LogWriter};
use serde::Serialize;

/// How often a recording is flushed to disk, so a crash loses little.
const FLUSH_EVERY: Duration = Duration::from_secs(1);

pub struct Recorder {
    writer: LogWriter<BufWriter<File>>,
    pub name: String,
    pub started: Instant,
    pub entries: u64,
    flushed: Instant,
}

impl Recorder {
    /// Starts `<UTC time>-<source>.mcap` in `dir`, creating `dir` if needed.
    /// `info` is stored in the log, along with the start time.
    pub fn start(dir: &Path, source: &str, mut info: BTreeMap<String, String>) -> io::Result<Self> {
        std::fs::create_dir_all(dir)?;
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
        let stamp = utc_stamp(now.as_secs());
        let mut name = format!("{stamp}-{source}.mcap");
        let mut n = 2;
        while dir.join(&name).exists() {
            name = format!("{stamp}-{source}-{n}.mcap");
            n += 1;
        }
        info.insert("source".into(), source.into());
        info.insert("started_unix_ms".into(), now.as_millis().to_string());
        let file = File::create_new(dir.join(&name))?;
        let writer = LogWriter::new(BufWriter::new(file), &info)?;
        Ok(Self {
            writer,
            name,
            started: Instant::now(),
            entries: 0,
            flushed: Instant::now(),
        })
    }

    pub fn write(&mut self, entry: &Entry) -> io::Result<()> {
        self.writer.write(entry)?;
        self.entries += 1;
        if self.flushed.elapsed() >= FLUSH_EVERY {
            self.writer.flush()?;
            self.flushed = Instant::now();
        }
        Ok(())
    }

    pub fn finish(self) -> io::Result<()> {
        self.writer.finish()
    }
}

/// `YYYYMMDD-HHMMSS` in UTC, which sorts by time.
fn utc_stamp(unix_s: u64) -> String {
    let (days, secs) = (unix_s / 86_400, unix_s % 86_400);
    // Howard Hinnant's days-to-civil algorithm.
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}{month:02}{day:02}-{:02}{:02}{:02}",
        secs / 3600,
        secs / 60 % 60,
        secs % 60
    )
}

/// A log in the recordings folder, as the page lists it.
#[derive(Debug, Serialize)]
pub struct Listed {
    pub name: String,
    pub bytes: u64,
}

/// The logs in `dir`, newest first. A missing folder has none.
pub fn list(dir: &Path) -> Vec<Listed> {
    let Ok(read) = std::fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut logs: Vec<_> = read
        .filter_map(|e| {
            let e = e.ok()?;
            let name = e.file_name().into_string().ok()?;
            let meta = e.metadata().ok()?;
            (meta.is_file() && valid_name(&name)).then_some(Listed {
                name,
                bytes: meta.len(),
            })
        })
        .collect();
    // Names start with their UTC start time.
    logs.sort_by(|a, b| b.name.cmp(&a.name));
    logs
}

/// Only plain `.mcap` names inside the folder, so the page can't reach
/// other files.
pub fn valid_name(name: &str) -> bool {
    name.ends_with(".mcap")
        && !name.starts_with('.')
        && !name.contains(['/', '\\'])
        && Path::new(name).components().count() == 1
}

pub fn path_of(dir: &Path, name: &str) -> io::Result<PathBuf> {
    if valid_name(name) {
        Ok(dir.join(name))
    } else {
        Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("{name:?} isn't a recording name"),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamps_are_utc_calendar_times() {
        assert_eq!(utc_stamp(0), "19700101-000000");
        // 2026-10-24T17:41:09Z
        assert_eq!(utc_stamp(1_792_863_669), "20261024-174109");
        // A leap day.
        assert_eq!(utc_stamp(951_782_400), "20000229-000000");
    }

    #[test]
    fn names_stay_inside_the_folder() {
        assert!(valid_name("20261024-174109-real.mcap"));
        assert!(!valid_name("../secret.mcap"));
        assert!(!valid_name("a/b.mcap"));
        assert!(!valid_name(".mcap"));
        assert!(!valid_name("robot.toml"));
    }
}
