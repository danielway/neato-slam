//! Recorded runs, as MCAP files (<https://mcap.dev/spec>).
//!
//! One channel per message type, one for the commands sent and one for
//! ground truth when the source knows it, each carrying postcard-encoded
//! bodies, the same encoding as the wire. Log and publish times are the
//! source's own stamps. A metadata record holds the run's `robot.toml` and
//! whatever else the recorder knows (source, address, start time).
//!
//! The writer is a small subset of the format, with no chunks, compression
//! or index, so it needs no extra crates. Such files are still valid MCAP,
//! so `mcap info` and Foxglove can open them, though only this crate can
//! decode the postcard bodies. Messages are written as they come, so a run
//! cut short by a crash is still readable up to its last whole record.

use std::collections::BTreeMap;
use std::io::{self, Write};

use serde::{Deserialize, Serialize};

use crate::{Command, Contacts, LidarScan, Msg, Pose2, RobotStatus, Stamped, WheelPositions};

const MAGIC: &[u8; 8] = b"\x89MCAP0\r\n";
const PROFILE: &str = "neato-slam";
/// Name of the metadata record that carries [`Recording::info`].
const INFO: &str = "neato-slam";
/// The key in [`Recording::info`] that holds the run's `robot.toml`.
pub const ROBOT_TOML: &str = "robot.toml";
const ENCODING: &str = "postcard";

mod op {
    pub const HEADER: u8 = 0x01;
    pub const FOOTER: u8 = 0x02;
    pub const CHANNEL: u8 = 0x04;
    pub const MESSAGE: u8 = 0x05;
    pub const METADATA: u8 = 0x0C;
    pub const DATA_END: u8 = 0x0F;
}

/// Channel ids, fixed so every log uses the same ones.
const TOPICS: [(u16, &str); 6] = [
    (1, "/lidar"),
    (2, "/wheels"),
    (3, "/contacts"),
    (4, "/status"),
    (5, "/command"),
    (6, "/ground_truth"),
];

/// One record of a run.
#[derive(Debug, Clone, PartialEq)]
pub enum Entry {
    Msg(Stamped<Msg>),
    /// A command sent to the source, stamped with the source's clock.
    Command {
        t_ns: u64,
        command: Command,
    },
    /// The true pose, from a source that knows it.
    Truth {
        t_ns: u64,
        pose: Pose2,
    },
}

impl Entry {
    pub fn t_ns(&self) -> u64 {
        match self {
            Entry::Msg(m) => m.t_ns,
            Entry::Command { t_ns, .. } | Entry::Truth { t_ns, .. } => *t_ns,
        }
    }
}

/// `Pose2` stays off the wire, so the log has its own copy.
#[derive(Serialize, Deserialize)]
struct TruthPose {
    x: f64,
    y: f64,
    theta: f64,
}

/// Writes a run as it happens.
pub struct LogWriter<W: Write> {
    out: W,
    command_seq: u32,
    truth_seq: u32,
    finished: bool,
}

fn put_str(buf: &mut Vec<u8>, s: &str) {
    buf.extend_from_slice(&(s.len() as u32).to_le_bytes());
    buf.extend_from_slice(s.as_bytes());
}

fn put_map(buf: &mut Vec<u8>, map: &BTreeMap<String, String>) {
    let mut body = Vec::new();
    for (k, v) in map {
        put_str(&mut body, k);
        put_str(&mut body, v);
    }
    buf.extend_from_slice(&(body.len() as u32).to_le_bytes());
    buf.extend_from_slice(&body);
}

fn invalid(e: impl std::fmt::Display) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, e.to_string())
}

impl<W: Write> LogWriter<W> {
    /// Starts a log. `info` is stored as metadata and comes back as
    /// [`Recording::info`]; put the run's `robot.toml` under [`ROBOT_TOML`].
    pub fn new(mut out: W, info: &BTreeMap<String, String>) -> io::Result<Self> {
        out.write_all(MAGIC)?;
        let mut header = Vec::new();
        put_str(&mut header, PROFILE);
        put_str(
            &mut header,
            concat!("neato-slam robot ", env!("CARGO_PKG_VERSION")),
        );
        let mut writer = Self {
            out,
            command_seq: 0,
            truth_seq: 0,
            finished: false,
        };
        writer.record(op::HEADER, &header)?;
        for (id, topic) in TOPICS {
            let mut channel = Vec::new();
            channel.extend_from_slice(&id.to_le_bytes());
            // Schema 0: none. The bodies are postcard, described by this crate.
            channel.extend_from_slice(&0u16.to_le_bytes());
            put_str(&mut channel, topic);
            put_str(&mut channel, ENCODING);
            put_map(&mut channel, &BTreeMap::new());
            writer.record(op::CHANNEL, &channel)?;
        }
        let mut metadata = Vec::new();
        put_str(&mut metadata, INFO);
        put_map(&mut metadata, info);
        writer.record(op::METADATA, &metadata)?;
        Ok(writer)
    }

    fn record(&mut self, opcode: u8, body: &[u8]) -> io::Result<()> {
        self.out.write_all(&[opcode])?;
        self.out.write_all(&(body.len() as u64).to_le_bytes())?;
        self.out.write_all(body)
    }

    fn message<T: Serialize>(
        &mut self,
        channel: u16,
        seq: u32,
        t_ns: u64,
        value: &T,
    ) -> io::Result<()> {
        let mut body = Vec::with_capacity(22 + 64);
        body.extend_from_slice(&channel.to_le_bytes());
        body.extend_from_slice(&seq.to_le_bytes());
        body.extend_from_slice(&t_ns.to_le_bytes());
        body.extend_from_slice(&t_ns.to_le_bytes());
        body.extend_from_slice(&postcard::to_allocvec(value).map_err(invalid)?);
        self.record(op::MESSAGE, &body)
    }

    pub fn write_msg(&mut self, m: &Stamped<Msg>) -> io::Result<()> {
        let (seq, t) = (m.seq, m.t_ns);
        match &m.msg {
            Msg::Lidar(scan) => self.message(1, seq, t, scan),
            Msg::Wheels(w) => self.message(2, seq, t, w),
            Msg::Contacts(c) => self.message(3, seq, t, c),
            Msg::Status(s) => self.message(4, seq, t, s),
        }
    }

    pub fn write_command(&mut self, t_ns: u64, command: &Command) -> io::Result<()> {
        let seq = self.command_seq;
        self.command_seq = seq.wrapping_add(1);
        self.message(5, seq, t_ns, command)
    }

    pub fn write_truth(&mut self, t_ns: u64, pose: Pose2) -> io::Result<()> {
        let seq = self.truth_seq;
        self.truth_seq = seq.wrapping_add(1);
        let pose = TruthPose {
            x: pose.x,
            y: pose.y,
            theta: pose.theta,
        };
        self.message(6, seq, t_ns, &pose)
    }

    pub fn write(&mut self, entry: &Entry) -> io::Result<()> {
        match entry {
            Entry::Msg(m) => self.write_msg(m),
            Entry::Command { t_ns, command } => self.write_command(*t_ns, command),
            Entry::Truth { t_ns, pose } => self.write_truth(*t_ns, *pose),
        }
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.out.flush()
    }

    /// Closes the data section and writes the footer. A log that never gets
    /// here still reads, and is reported as truncated.
    pub fn finish(mut self) -> io::Result<()> {
        self.close()
    }

    fn close(&mut self) -> io::Result<()> {
        if self.finished {
            return Ok(());
        }
        self.finished = true;
        // A zero CRC means "not computed", which readers accept.
        self.record(op::DATA_END, &0u32.to_le_bytes())?;
        // No summary section, so both offsets are zero.
        self.record(op::FOOTER, &[0; 20])?;
        self.out.write_all(MAGIC)?;
        self.out.flush()
    }
}

impl<W: Write> Drop for LogWriter<W> {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

/// A whole run, read back.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Recording {
    /// The metadata the recorder stored, such as [`ROBOT_TOML`].
    pub info: BTreeMap<String, String>,
    /// Every record, in the order written.
    pub entries: Vec<Entry>,
    /// The file ends before its footer, as when the recorder was killed.
    pub truncated: bool,
}

struct Cursor<'a>(&'a [u8]);

impl<'a> Cursor<'a> {
    fn take(&mut self, n: usize) -> io::Result<&'a [u8]> {
        if self.0.len() < n {
            return Err(io::ErrorKind::UnexpectedEof.into());
        }
        let (head, rest) = self.0.split_at(n);
        self.0 = rest;
        Ok(head)
    }

    fn u16(&mut self) -> io::Result<u16> {
        Ok(u16::from_le_bytes(self.take(2)?.try_into().unwrap()))
    }

    fn u32(&mut self) -> io::Result<u32> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }

    fn u64(&mut self) -> io::Result<u64> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }

    fn string(&mut self) -> io::Result<String> {
        let n = self.u32()? as usize;
        String::from_utf8(self.take(n)?.to_vec()).map_err(invalid)
    }

    fn map(&mut self) -> io::Result<BTreeMap<String, String>> {
        let n = self.u32()? as usize;
        let mut inner = Cursor(self.take(n)?);
        let mut map = BTreeMap::new();
        while !inner.0.is_empty() {
            map.insert(inner.string()?, inner.string()?);
        }
        Ok(map)
    }
}

impl Recording {
    pub fn read_file(path: impl AsRef<std::path::Path>) -> io::Result<Self> {
        Self::from_bytes(&std::fs::read(path)?)
    }

    /// Parses a log. Channels it doesn't know are skipped, so newer logs
    /// with extra channels still play.
    pub fn from_bytes(bytes: &[u8]) -> io::Result<Self> {
        let mut cur = Cursor(bytes);
        if cur.take(8).ok() != Some(MAGIC.as_slice()) {
            return Err(invalid("not an MCAP file"));
        }
        let mut rec = Recording {
            truncated: true,
            ..Recording::default()
        };
        // Channel id to topic, from the log's own channel records.
        let mut topics: BTreeMap<u16, String> = BTreeMap::new();
        while let Ok(opcode) = cur.take(1).map(|b| b[0]) {
            let Ok(len) = cur.u64() else { break };
            let Ok(body) = usize::try_from(len)
                .map_err(invalid)
                .and_then(|n| cur.take(n))
            else {
                break;
            };
            let mut body = Cursor(body);
            match opcode {
                op::CHANNEL => {
                    let id = body.u16()?;
                    let _schema = body.u16()?;
                    let topic = body.string()?;
                    let encoding = body.string()?;
                    if encoding == ENCODING {
                        topics.insert(id, topic);
                    }
                }
                op::METADATA => {
                    if body.string()? == INFO {
                        rec.info.extend(body.map()?);
                    }
                }
                op::MESSAGE => {
                    let channel = body.u16()?;
                    let seq = body.u32()?;
                    let t_ns = body.u64()?;
                    let _publish = body.u64()?;
                    if let Some(topic) = topics.get(&channel)
                        && let Some(entry) = decode(topic, seq, t_ns, body.0)?
                    {
                        rec.entries.push(entry);
                    }
                }
                op::FOOTER => {
                    rec.truncated = false;
                    break;
                }
                _ => {}
            }
        }
        Ok(rec)
    }

    /// Stamps of the first and last entries.
    pub fn span_ns(&self) -> Option<(u64, u64)> {
        let first = self.entries.first()?.t_ns();
        let last = self.entries.last()?.t_ns();
        Some((first, last))
    }
}

fn decode(topic: &str, seq: u32, t_ns: u64, body: &[u8]) -> io::Result<Option<Entry>> {
    fn de<'a, T: Deserialize<'a>>(body: &'a [u8]) -> io::Result<T> {
        postcard::from_bytes(body).map_err(invalid)
    }
    let msg = |msg| Some(Entry::Msg(Stamped { t_ns, seq, msg }));
    Ok(match topic {
        "/lidar" => msg(Msg::Lidar(de::<LidarScan>(body)?)),
        "/wheels" => msg(Msg::Wheels(de::<WheelPositions>(body)?)),
        "/contacts" => msg(Msg::Contacts(de::<Contacts>(body)?)),
        "/status" => msg(Msg::Status(de::<RobotStatus>(body)?)),
        "/command" => Some(Entry::Command {
            t_ns,
            command: de(body)?,
        }),
        "/ground_truth" => {
            let p: TruthPose = de(body)?;
            Some(Entry::Truth {
                t_ns,
                pose: Pose2::new(p.x, p.y, p.theta),
            })
        }
        _ => None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entries() -> Vec<Entry> {
        vec![
            Entry::Msg(Stamped {
                t_ns: 100,
                seq: 0,
                msg: Msg::Wheels(WheelPositions {
                    left_mm: -3,
                    right_mm: 4,
                }),
            }),
            Entry::Command {
                t_ns: 150,
                command: Command::WheelSpeeds {
                    left_mm_s: 10.0,
                    right_mm_s: -10.0,
                },
            },
            Entry::Msg(Stamped {
                t_ns: 200,
                seq: 9,
                msg: Msg::Lidar(LidarScan {
                    angle_min: 0.0,
                    angle_increment: 0.5,
                    ranges: vec![1.0, 2.5],
                    intensities: Some(vec![7, 8]),
                    scan_duration: 0.2,
                }),
            }),
            Entry::Truth {
                t_ns: 200,
                pose: Pose2::new(1.0, 2.0, 0.5),
            },
            Entry::Msg(Stamped {
                t_ns: 250,
                seq: 1,
                msg: Msg::Contacts(Contacts {
                    bumpers: Contacts::LEFT_FRONT,
                    wheel_drop: 0,
                }),
            }),
            Entry::Msg(Stamped {
                t_ns: 300,
                seq: 2,
                msg: Msg::Status(RobotStatus {
                    battery_v: 16.1,
                    fuel_percent: 87,
                    charging: false,
                    external_power: true,
                }),
            }),
        ]
    }

    fn info() -> BTreeMap<String, String> {
        BTreeMap::from([
            ("source".to_owned(), "test".to_owned()),
            (ROBOT_TOML.to_owned(), "[drive]\n".to_owned()),
        ])
    }

    fn write(entries: &[Entry]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut w = LogWriter::new(&mut out, &info()).unwrap();
        for e in entries {
            w.write(e).unwrap();
        }
        w.finish().unwrap();
        out
    }

    #[test]
    fn round_trips() {
        let bytes = write(&entries());
        assert!(bytes.ends_with(MAGIC));
        let rec = Recording::from_bytes(&bytes).unwrap();
        assert!(!rec.truncated);
        assert_eq!(rec.info, info());
        assert_eq!(rec.entries, entries());
        assert_eq!(rec.span_ns(), Some((100, 300)));
    }

    #[test]
    fn missing_lidar_returns_survive() {
        let scan = Entry::Msg(Stamped {
            t_ns: 1,
            seq: 0,
            msg: Msg::Lidar(LidarScan {
                angle_min: 0.0,
                angle_increment: 1.0,
                ranges: vec![f32::NAN, 1.0],
                intensities: None,
                scan_duration: 0.2,
            }),
        });
        let rec = Recording::from_bytes(&write(&[scan])).unwrap();
        let Entry::Msg(Stamped {
            msg: Msg::Lidar(back),
            ..
        }) = &rec.entries[0]
        else {
            panic!("{:?}", rec.entries)
        };
        assert!(back.ranges[0].is_nan());
        assert_eq!(back.ranges[1], 1.0);
    }

    #[test]
    fn a_log_cut_short_reads_up_to_the_cut() {
        let bytes = write(&entries());
        // Drop the footer and magic, then half of the last message.
        let cut = &bytes[..bytes.len() - 8 - 29 - 13 - 10];
        let rec = Recording::from_bytes(cut).unwrap();
        assert!(rec.truncated);
        assert_eq!(rec.entries, entries()[..5]);
    }

    #[test]
    fn dropping_the_writer_finishes_the_file() {
        let mut out = Vec::new();
        {
            let mut w = LogWriter::new(&mut out, &info()).unwrap();
            w.write(&entries()[0]).unwrap();
        }
        assert!(!Recording::from_bytes(&out).unwrap().truncated);
    }

    #[test]
    fn rejects_other_files() {
        assert!(Recording::from_bytes(b"hello").is_err());
    }
}
