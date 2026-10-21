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
| `station` | desktop | Control app; for now a scripted sim drive |

Robot geometry and sensor parameters live in [`robot.toml`](robot.toml). Values marked TODO still need measuring on the real robot.

## Running

```sh
cargo test
cargo run -p station   # drives the sim for 10 s and compares odometry with truth
```

`tools/capture-neato.sh` records the Neato's raw replies to the serial commands the stack uses (run it on the Pi with the robot plugged in). It never drives the wheels.

## Dependencies

Third-party crates are kept few and widely used, pinned by `Cargo.lock`, and checked in CI by [cargo-deny](https://github.com/EmbarkStudios/cargo-deny) for known vulnerabilities, licenses and sources (`deny.toml`). Run `cargo deny check` locally after adding one.
