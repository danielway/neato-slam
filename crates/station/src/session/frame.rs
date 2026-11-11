use robot::{Command, Pose2, RobotIo};
use serde_json::{Value, json};

use super::real::{Link, STALL_AFTER};
use super::{Session, Source};
use crate::camera::CameraFeed;
use crate::drive_test;

impl Session {
    pub(super) fn frame(&self) -> Value {
        let pose = |p: Pose2| json!([round(p.x), round(p.y), round(p.theta)]);
        let (readings, mut frame) = match &self.source {
            Source::Sim(sim) => {
                let frame = json!({
                    "backend": "sim",
                    "state": if sim.paused { "paused" } else { "running" },
                    "command": describe(sim.command),
                    "truth": pose(sim.robot.ground_truth().expect("the sim always knows its pose")),
                    "walls": sim.walls(),
                });
                (&sim.readings, frame)
            }
            Source::Real(real) => {
                let (state, detail) = match &real.link {
                    Link::Connecting => ("connecting", String::new()),
                    Link::Connected { last_data } if last_data.elapsed() >= STALL_AFTER => (
                        "stalled",
                        format!(
                            "connected, but no data for {:.0} s",
                            last_data.elapsed().as_secs_f64()
                        ),
                    ),
                    Link::Connected { .. } => ("connected", String::new()),
                    Link::Closed { why, .. } => ("disconnected", format!("{why}; retrying")),
                };
                let frame = json!({
                    "backend": "real",
                    "addr": real.addr,
                    "state": state,
                    "detail": detail,
                    "command": describe(real.command),
                    "lidar": real.lidar,
                    "test_mode": real.test_mode,
                    "truth": self.camera.as_ref().and_then(CameraFeed::fresh).map(pose),
                    "walls": [],
                });
                (&real.readings, frame)
            }
            Source::Replay(replay) => {
                let state = if replay.ended() {
                    "ended"
                } else if replay.paused {
                    "paused"
                } else {
                    "playing"
                };
                let mut detail = format!(
                    "{}  {:.1} / {:.1} s",
                    replay.name,
                    replay.clock_ns as f64 * 1e-9,
                    replay.duration_ns() as f64 * 1e-9,
                );
                if replay.recording.truncated {
                    detail.push_str(" (cut short)");
                }
                let frame = json!({
                    "backend": "replay",
                    "file": replay.name,
                    "state": state,
                    "detail": detail,
                    "command": replay.command.map_or_else(|| "replay".to_owned(), describe),
                    "truth": replay.truth.map(pose),
                    "walls": replay.walls,
                    "speed": replay.speed,
                });
                (&replay.readings, frame)
            }
        };
        let hits: Vec<_> = readings
            .hits
            .iter()
            .map(|&[x, y]| [round(x), round(y)])
            .collect();
        let extra = json!({
            "run": self.run,
            "t": readings.t_ns as f64 * 1e-9,
            "odometry": pose(readings.odometry.pose),
            "hits": hits,
            "bumpers": readings.contacts.bumpers,
            "wheel_drop": readings.contacts.wheel_drop,
            "battery": readings.status.map(|s| json!({
                "v": s.battery_v,
                "percent": s.fuel_percent,
                "charging": s.charging,
                "external_power": s.external_power,
            })),
            "recording": self.tape.recorder.as_ref().map(|r| json!({
                "file": r.name,
                "seconds": r.started.elapsed().as_secs_f64().round(),
                "entries": r.entries,
            })),
            "notice": self.tape.error.as_ref().or(self.notice.as_ref()),
            "test": self.test.as_ref().map(|t| json!({
                "step": t.step,
                "seconds": t.started.elapsed().as_secs_f64().round(),
                "length": drive_test::length().as_secs_f64().round(),
            })),
            "camera": self.camera.as_ref().map(|c| json!({
                "state": c.state.as_str(),
                "error": c.error,
            })),
        });
        if let (Value::Object(frame), Value::Object(extra)) = (&mut frame, extra) {
            frame.extend(extra);
            // The arena and the floor under it, everywhere but the sim.
            if let Some(camera) = &self.camera
                && !matches!(self.source, Source::Sim(_))
            {
                frame.insert("arena".into(), json!(self.arena));
                if let Some(t) = &camera.topdown {
                    frame.insert("floor".into(), json!({ "bounds": t.bounds }));
                }
            }
        }
        frame
    }
}

/// Millimetre precision is plenty for drawing and keeps frames small.
fn round(v: f64) -> f64 {
    (v * 1000.0).round() / 1000.0
}

fn describe(command: Command) -> String {
    match command {
        Command::WheelSpeeds {
            left_mm_s,
            right_mm_s,
        } => {
            format!("wheels L {left_mm_s:+.0} R {right_mm_s:+.0} mm/s")
        }
        Command::Stop => "stopped".to_owned(),
        Command::Lidar { on } => format!("lidar {}", if on { "on" } else { "off" }),
        Command::TestMode { on } => format!("test mode {}", if on { "on" } else { "off" }),
    }
}
