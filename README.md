# neato-slam

Developing SLAM in Rust for a Neato robot vacuum, against the real robot, recorded logs, and a 2D simulator.

SLAM and tooling talk to one interface, `RobotIo`, with three interchangeable backends:

- **Real:** the Neato, through `pi-node` on a Raspberry Pi, over TCP.
- **Replay:** a recorded run, played back with its original stamps. Commands are ignored.
- **Sim:** a 2D world that emits the same messages and also knows the true pose.

## Crates

| Crate | Runs on | Purpose |
| --- | --- | --- |
| `robot` | both | Messages, `robot.toml` config, TCP framing, the `RobotIo` trait, real and replay backends |
| `neato` | Pi | Neato USB serial protocol, plus the `pi-node` binary that bridges it to the desktop (stub) |
| `sim` | desktop | World, diff-drive kinematics, lidar raycasting, the sim backend |
| `slam` | desktop | SLAM algorithms and their scoring against ground truth |
| `station` | desktop | Control app served as a web page: the sim driven from the keyboard, or the real robot view-only |

Robot geometry and sensor parameters live in [`robot.toml`](robot.toml). Values marked TODO still need measuring on the real robot.

## Running

```sh
cargo test
cargo run -p station                         # serves the station on http://127.0.0.1:8080
cargo run -p station -- --listen 0.0.0.0:8080  # the same, reachable from outside a container
cargo run -p station -- --headless           # drives the sim for 10 s and prints odometry against truth
```

The station has no native window. It serves one page, so it can run headless in a sandbox and be opened from any browser. The page picks the source:

- **Sim:** click the map, then drive with WASD or the arrow keys (Shift for full speed). P pauses, R resets, O draws the scan from the odometry pose instead of the true one.
- **Real robot:** enter `pi-node`'s address, such as `192.168.1.50:7878`. This is view-only: the station shows the lidar, odometry, bumpers and battery but never sends the robot a command, whatever keys are pressed.
- **Replay** is listed but disabled until runs can be recorded.

It shows the walls, the true pose and its trail in green, wheel odometry and its trail in orange, and the latest lidar scan in red. If the page stops re-sending held keys (a closed tab, a dropped connection), the sim treats them as released within half a second.

[docs/setup.md](docs/setup.md) covers the usual setup: the station in a Docker sandbox (`sbx`) on the Mac, `pi-node` on the Pi over Wi-Fi, and cross-compiling `pi-node` for the Pi.

`tools/capture-neato.sh` records the Neato's raw replies to the serial commands the stack uses (run it on the Pi with the robot plugged in). It never drives the wheels.

## Dependencies

Third-party crates are kept few and widely used, pinned by `Cargo.lock`, and checked in CI by [cargo-deny](https://github.com/EmbarkStudios/cargo-deny) for known vulnerabilities, licenses and sources (`deny.toml`). Run `cargo deny check` locally after adding one.
