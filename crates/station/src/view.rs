//! An interactive window onto the simulator: drive with the keyboard and
//! watch the true pose, wheel odometry and the lidar together.

use macroquad::prelude::*;
use robot::{Command, LidarScan, Msg, Pose2, RobotConfig, RobotIo};
use sim::{Segment, Sim, SimBackend, World};
use slam::odometry::WheelOdometry;

use crate::teleop::{self, Keys};

/// Neato body radius in metres, for drawing only.
const BODY_RADIUS_M: f32 = 0.165;
/// Longest frame the sim catches up on, so a stalled window doesn't jump.
const MAX_FRAME_S: f64 = 0.1;
/// Trail points closer together than this are skipped, in metres.
const TRAIL_STEP_M: f64 = 0.01;

const BACKGROUND: Color = Color::new(0.08, 0.09, 0.11, 1.0);
const WALL: Color = Color::new(0.85, 0.85, 0.88, 1.0);
const GRID: Color = Color::new(1.0, 1.0, 1.0, 0.05);
const TRUTH: Color = Color::new(0.30, 0.80, 0.45, 1.0);
const ODOMETRY: Color = Color::new(0.95, 0.60, 0.20, 1.0);
const RAY: Color = Color::new(0.90, 0.25, 0.25, 0.12);
const HIT: Color = Color::new(1.0, 0.30, 0.30, 0.9);
const TEXT: Color = Color::new(0.85, 0.87, 0.90, 1.0);

pub fn conf() -> Conf {
    Conf {
        window_title: "neato-slam station".to_owned(),
        window_width: 1100,
        window_height: 850,
        high_dpi: true,
        sample_count: 4,
        ..Default::default()
    }
}

/// A room with a few obstacles, so scans have some structure to show.
pub fn demo_world() -> World {
    let mut world = World::rectangle(6.0, 4.5);
    let mut wall = |a: (f64, f64), b: (f64, f64)| world.segments.push(Segment { a, b });
    // An interior wall with a doorway.
    wall((3.5, 0.0), (3.5, 1.6));
    wall((3.5, 2.4), (3.5, 3.2));
    // A box, like a cabinet.
    let (x0, y0, x1, y1) = (1.2, 3.2, 2.2, 3.8);
    wall((x0, y0), (x1, y0));
    wall((x1, y0), (x1, y1));
    wall((x1, y1), (x0, y1));
    wall((x0, y1), (x0, y0));
    // An angled piece of furniture in the second room.
    wall((4.5, 3.6), (5.4, 2.9));
    world
}

/// Everything that resets with R.
struct Run {
    robot: SimBackend,
    odometry: WheelOdometry,
    /// Stamp of the latest message.
    t_ns: u64,
    /// Where sim time should be by now. The sim steps in whole physics
    /// steps, so it runs up to this rather than a frame past its last stamp,
    /// which would round every frame up and run fast.
    clock_ns: u64,
    command: Command,
    scan: Option<LidarScan>,
    truth_trail: Vec<Vec2>,
    odometry_trail: Vec<Vec2>,
}

impl Run {
    fn new(config: &RobotConfig, start: Pose2) -> Self {
        let sim = Sim::new(demo_world(), config.clone(), start, 0.01);
        Self {
            robot: SimBackend::new(sim),
            odometry: WheelOdometry::new(start, config.drive.wheel_base_m),
            t_ns: 0,
            clock_ns: 0,
            command: Command::Stop,
            scan: None,
            truth_trail: vec![point(start)],
            odometry_trail: vec![point(start)],
        }
    }

    fn truth(&self) -> Pose2 {
        self.robot
            .ground_truth()
            .expect("the sim always knows its pose")
    }

    /// Sends the command if it changed, then runs the sim `frame_ns` further.
    async fn advance(&mut self, command: Command, frame_ns: u64) -> std::io::Result<()> {
        if command != self.command {
            self.robot.send(command).await?;
            self.command = command;
        }
        self.clock_ns += frame_ns;
        while self.t_ns < self.clock_ns {
            let Some(m) = self.robot.recv().await? else {
                break;
            };
            self.t_ns = m.t_ns;
            match m.msg {
                Msg::Wheels(w) => {
                    self.odometry.update(w);
                    let truth = self.truth();
                    push_trail(&mut self.truth_trail, truth);
                    push_trail(&mut self.odometry_trail, self.odometry.pose);
                }
                Msg::Lidar(scan) => self.scan = Some(scan),
                _ => {}
            }
        }
        Ok(())
    }
}

fn point(p: Pose2) -> Vec2 {
    vec2(p.x as f32, p.y as f32)
}

fn push_trail(trail: &mut Vec<Vec2>, pose: Pose2) {
    let p = point(pose);
    if trail
        .last()
        .is_none_or(|last| last.distance(p) as f64 >= TRAIL_STEP_M)
    {
        trail.push(p);
    }
}

/// Maps world metres (y up) to screen pixels (y down), fitting the world's
/// bounds into the window.
struct Viewport {
    scale: f32,
    origin: Vec2,
}

impl Viewport {
    fn fit(world: &World, top_px: f32) -> Self {
        let (mut min, mut max) = (Vec2::splat(f32::MAX), Vec2::splat(f32::MIN));
        for s in &world.segments {
            for (x, y) in [s.a, s.b] {
                let p = vec2(x as f32, y as f32);
                min = min.min(p);
                max = max.max(p);
            }
        }
        let margin = 24.0;
        let avail = vec2(screen_width(), screen_height() - top_px) - 2.0 * margin;
        let size = (max - min).max(Vec2::splat(1e-3));
        let scale = (avail.x / size.x).min(avail.y / size.y);
        let used = size * scale;
        let corner = vec2(margin, top_px + margin) + (avail - used) / 2.0;
        Self {
            scale,
            origin: vec2(corner.x - min.x * scale, corner.y + max.y * scale),
        }
    }

    fn px(&self, p: Vec2) -> Vec2 {
        vec2(
            self.origin.x + p.x * self.scale,
            self.origin.y - p.y * self.scale,
        )
    }

    fn line(&self, a: Vec2, b: Vec2, thickness: f32, color: Color) {
        let (a, b) = (self.px(a), self.px(b));
        draw_line(a.x, a.y, b.x, b.y, thickness, color);
    }
}

fn draw_trail(view: &Viewport, trail: &[Vec2], color: Color) {
    for pair in trail.windows(2) {
        view.line(pair[0], pair[1], 1.5, color);
    }
}

fn draw_robot(view: &Viewport, pose: Pose2, color: Color, filled: bool) {
    let c = view.px(point(pose));
    let r = BODY_RADIUS_M * view.scale;
    if filled {
        draw_circle(c.x, c.y, r, Color { a: 0.35, ..color });
    }
    draw_circle_lines(c.x, c.y, r, 2.0, color);
    let (s, co) = (pose.theta as f32).sin_cos();
    let nose = point(pose) + vec2(co, s) * BODY_RADIUS_M;
    view.line(point(pose), nose, 2.0, color);
}

/// Draws a scan as seen from `pose`: rays from the lidar and dots at the hits.
fn draw_scan(view: &Viewport, scan: &LidarScan, pose: Pose2, config: &RobotConfig) {
    let lidar = &config.lidar;
    let origin = pose.compose(&Pose2::new(lidar.x_m, lidar.y_m, lidar.theta_rad));
    let from = point(origin);
    for (i, &r) in scan.ranges.iter().enumerate() {
        if r.is_nan() {
            continue;
        }
        let a = (scan.angle_min + i as f32 * scan.angle_increment) as f64;
        let (x, y) = origin.transform_point(r as f64 * a.cos(), r as f64 * a.sin());
        let hit = vec2(x as f32, y as f32);
        view.line(from, hit, 1.0, RAY);
        let p = view.px(hit);
        draw_circle(p.x, p.y, 2.5, HIT);
    }
}

fn draw_grid(view: &Viewport, world: &World) {
    let max = world
        .segments
        .iter()
        .flat_map(|s| [s.a, s.b])
        .fold((0.0f64, 0.0f64), |m, p| (m.0.max(p.0), m.1.max(p.1)));
    for x in 0..=max.0.ceil() as i32 {
        view.line(vec2(x as f32, 0.0), vec2(x as f32, max.1 as f32), 1.0, GRID);
    }
    for y in 0..=max.1.ceil() as i32 {
        view.line(vec2(0.0, y as f32), vec2(max.0 as f32, y as f32), 1.0, GRID);
    }
}

fn held(keys: &[KeyCode]) -> bool {
    keys.iter().any(|&k| is_key_down(k))
}

fn read_keys() -> Keys {
    Keys {
        forward: held(&[KeyCode::W, KeyCode::Up]),
        back: held(&[KeyCode::S, KeyCode::Down]),
        left: held(&[KeyCode::A, KeyCode::Left]),
        right: held(&[KeyCode::D, KeyCode::Right]),
        fast: held(&[KeyCode::LeftShift, KeyCode::RightShift]),
    }
}

fn describe(command: Command) -> String {
    match command {
        Command::WheelSpeeds {
            left_mm_s,
            right_mm_s,
        } => format!("wheels L {left_mm_s:+.0} R {right_mm_s:+.0} mm/s"),
        Command::Stop => "stopped".to_owned(),
    }
}

pub async fn run(config: RobotConfig, start: Pose2) {
    let mut state = Run::new(&config, start);
    let mut paused = false;
    let mut odometry_frame = false;

    loop {
        if is_key_pressed(KeyCode::Escape) || is_key_pressed(KeyCode::Q) {
            break;
        }
        if is_key_pressed(KeyCode::R) {
            state = Run::new(&config, start);
        }
        if is_key_pressed(KeyCode::P) {
            paused = !paused;
        }
        if is_key_pressed(KeyCode::O) {
            odometry_frame = !odometry_frame;
        }

        let command = if paused {
            Command::Stop
        } else {
            teleop::command(read_keys(), config.drive.max_wheel_speed_mm_s)
        };
        let frame_s = if paused {
            0.0
        } else {
            (get_frame_time() as f64).min(MAX_FRAME_S)
        };
        if let Err(e) = state.advance(command, (frame_s * 1e9) as u64).await {
            eprintln!("robot error: {e}");
            break;
        }

        let truth = state.truth();
        let odometry = state.odometry.pose;
        let world = &state.robot.sim().world;

        clear_background(BACKGROUND);
        let hud_px = 64.0;
        let view = Viewport::fit(world, hud_px);
        draw_grid(&view, world);
        for s in &world.segments {
            let to = |(x, y): (f64, f64)| vec2(x as f32, y as f32);
            view.line(to(s.a), to(s.b), 3.0, WALL);
        }
        if let Some(scan) = &state.scan {
            let pose = if odometry_frame { odometry } else { truth };
            draw_scan(&view, scan, pose, &config);
        }
        draw_trail(&view, &state.truth_trail, TRUTH);
        draw_trail(&view, &state.odometry_trail, ODOMETRY);
        draw_robot(&view, odometry, ODOMETRY, false);
        draw_robot(&view, truth, TRUTH, true);

        let error = (odometry.x - truth.x).hypot(odometry.y - truth.y);
        let status = format!(
            "t {:7.2} s   {}   truth ({:.2}, {:.2}, {:+.2})   odometry ({:.2}, {:.2}, {:+.2})   error {:.3} m{}",
            state.t_ns as f64 * 1e-9,
            describe(state.command),
            truth.x,
            truth.y,
            truth.theta,
            odometry.x,
            odometry.y,
            odometry.theta,
            error,
            if paused { "   PAUSED" } else { "" },
        );
        draw_text(&status, 16.0, 24.0, 20.0, TEXT);
        let help = format!(
            "WASD/arrows drive, Shift fast, P pause, R reset, O scan drawn from {} pose, Esc quit   \
             green: truth   orange: odometry",
            if odometry_frame { "odometry" } else { "true" },
        );
        draw_text(&help, 16.0, 48.0, 18.0, Color { a: 0.6, ..TEXT });

        next_frame().await;
    }
}
