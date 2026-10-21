# neato-slam

Developing SLAM in Rust for a Neato robot vacuum, against the real robot, recorded logs, and a 2D simulator.

SLAM and tooling talk to one interface, `RobotIo`, with three interchangeable backends:

- **Real:** the Neato, through `pi-node` on a Raspberry Pi, over TCP.
- **Replay:** a recorded run, played back with its original stamps. Commands are ignored.
- **Sim:** a 2D world that emits the same messages and also knows the true pose.

## Crates

| Crate | Runs on | Purpose |
| --- | --- | --- |
| `robot-msgs` | both | Message types, `robot.toml` config, TCP framing |
| `neato-proto` | Pi | Neato USB serial commands and response parsers |
| `pi-node` | Pi | Bridges the Neato's serial port to the desktop (stub) |
| `robot-io` | desktop | The `RobotIo` trait and the real, replay and sim backends |
| `sim` | desktop | World, diff-drive kinematics, lidar raycasting |
| `slam` | desktop | SLAM algorithms, starting with wheel odometry |
| `eval` | desktop | Scoring against ground truth (ATE so far) |
| `recorder` | desktop | MCAP logs (planned) |
| `station` | desktop | Control app; for now a scripted sim drive |

Robot geometry and sensor parameters live in [`robot.toml`](robot.toml). Values marked TODO still need measuring on the real robot.

## Running

```sh
cargo test
cargo run -p station   # drives the sim for 10 s and compares odometry with truth
```
