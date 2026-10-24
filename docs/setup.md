# Setup: station in a sandbox, pi-node on the Pi

```
 MacBook                                              Raspberry Pi 3B on the Neato
 ┌───────────────────────────────────────────┐        ┌───────────────────────────┐
 │ browser ── localhost:8080 ──┐             │        │ pi-node :7878             │
 │                     sbx sandbox (Linux)   │  Wi-Fi │   │                       │
 │                     station :8080 ────────┼────────┼──►│ USB serial ── Neato   │
 └───────────────────────────────────────────┘  TCP   └───────────────────────────┘
```

- `station` builds and runs inside the sandbox, so Rust and every crate stay there. It serves a web page instead of opening a window, because a Linux sandbox can't open windows on macOS.
- `pi-node` runs on the Pi and streams readings to the station over the home network. The Pi needs the LAN, not the internet.
- A lidar scan is about 360 × 6 bytes at 5 Hz, roughly 11 KB/s, which Wi-Fi handles easily. The Pi stamps each reading when it is taken, so network delay doesn't skew SLAM.

## 1. Run the station in the sandbox

Inside the sandbox, listen on all interfaces so a published port can reach it:

```sh
cargo run --release -p station -- --listen 0.0.0.0:8080
```

On the Mac, publish the port when creating the sandbox (`sbx run --publish 8080:8080 ...`), or add it to an existing one with `sbx ports` (re-attaching with `sbx run` ignores `--publish`). Then open <http://localhost:8080>. See Docker's [Develop and test locally](https://docs.docker.com/ai/sandboxes/workflows/development/) guide for the current flags.

The station's page takes controls from any browser that can reach the port, so publish it only on the Mac, not on a shared network.

## 2. Let the sandbox reach the Pi

The sandbox's network policy has to allow the Pi's address and port. With the Pi at a fixed address (set a DHCP reservation for it on the router):

```sh
sbx policy allow network 192.168.1.50:7878
sbx policy ls
```

Add `--sandbox <name>` to scope the rule to one sandbox. See [`sbx policy allow network`](https://docs.docker.com/reference/cli/sbx/policy/allow/network/). To check from inside the sandbox, with `pi-node` running:

```sh
nc -vz 192.168.1.50 7878
```

## 3. Cross-compile pi-node for the Pi

The Pi 3 Model B v1.2 has a 64-bit Cortex-A53. With a 64-bit Raspberry Pi OS, build a static aarch64 musl binary, which runs on any 64-bit Pi OS without matching its glibc. Zig serves as the cross linker, so no ARM toolchain is needed, and the same steps work in an x86_64 or an arm64 sandbox. One-time setup, in the sandbox:

```sh
rustup target add aarch64-unknown-linux-musl
pip install ziglang          # or install zig from ziglang.org
cargo install --locked cargo-zigbuild
```

Then build:

```sh
cargo zigbuild --release -p neato --bin pi-node --target aarch64-unknown-linux-musl
# -> target/aarch64-unknown-linux-musl/release/pi-node
```

On a 32-bit Pi OS (check with `uname -m` on the Pi: `aarch64` is 64-bit, `armv7l` is 32-bit), use `armv7-unknown-linux-musleabihf` instead.

Copy it over, which also needs the policy rule above, for port 22:

```sh
scp target/aarch64-unknown-linux-musl/release/pi-node pi@192.168.1.50:
ssh pi@192.168.1.50 ./pi-node --listen 0.0.0.0:7878
```

If the sandbox can't reach the Pi over SSH, copy the binary out of the sandbox to the Mac and `scp` it from there.

### Or build it on the Pi

Building on the Pi skips cross-compiling and copying. `pi-node` leaves out the `robot` crate's `net` feature, so the Pi compiles about 18 small crates and no tokio. The repo is public, so cloning needs internet but no GitHub credentials. One-time setup on the Pi:

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
git clone https://github.com/danielway/neato-slam
```

Then build, with `--locked` so the Pi uses exactly the committed `Cargo.lock`, and `-j 2` to stay within the Pi 3B's 1 GB of RAM:

```sh
cd neato-slam && git pull --ff-only
cargo build --release --locked -j 2 -p neato --bin pi-node
# -> target/release/pi-node
```

The first build takes several minutes; later ones are quick.

## 4. Run pi-node on the Pi

```sh
./pi-node --listen 0.0.0.0:7878
```

`--serial` defaults to `auto`, which finds the Neato under `/dev/serial/by-id/` or as the first `/dev/ttyACM*`. That matters because the robot sometimes drops off USB (when it sleeps, or on a marginal cable) and comes back under a new ttyACM number. `pi-node` notices the serial error, keeps the station connected, and reopens the robot when it reappears. On start it prints the Neato's `GetVersion` reply and waits for the station. If a new station connects while another is still connected, the new one takes over, so a dead connection can't block it. If it can't open the port, check `ls /dev/ttyACM*` and that your user is in the `dialout` group (`sudo usermod -aG dialout $USER`, then log in again).

When the station connects (pick "real" on its page and enter the Pi's address, such as `192.168.1.50:7878`, or `<hostname>.local:7878`, which keeps working if DHCP gives the Pi a new address), `pi-node` spins up the lidar, waits 2 seconds for it to reach speed, then streams wheel positions and bumpers at 10 Hz, a scan at 5 Hz and the battery state at 1 Hz. If the link drops, the station retries every 2 seconds on its own. When the station disconnects, it stops the lidar and waits for the next connection. A reply it can't parse is logged with its first few lines and skipped, which is the quickest way to spot a firmware that formats replies differently.

Stopping `pi-node` with Ctrl-C while a station is connected can leave the lidar spinning. Disconnecting the station first stops it, and so does unplugging USB or power-cycling the robot.

### Start it on boot

A systemd user service keeps `pi-node` running across reboots and crashes, with no sudo needed. Create `~/.config/systemd/user/pi-node.service`:

```ini
[Unit]
Description=Neato bridge for the SLAM station
After=network-online.target

[Service]
ExecStart=%h/neato-slam/target/release/pi-node --listen 0.0.0.0:7878
Restart=always
RestartSec=2

[Install]
WantedBy=default.target
```

Then enable it, and enable lingering so it starts without anyone logging in:

```sh
systemctl --user daemon-reload
systemctl --user enable --now pi-node
loginctl enable-linger $USER
journalctl --user -u pi-node -f   # follow its log
```

After rebuilding, run `systemctl --user restart pi-node`.

### Turn off Wi-Fi power saving

The Pi's Wi-Fi power saving adds latency spikes that show up as stuttering scans. With NetworkManager, run `nmcli con modify "<connection>" 802-11-wireless.powersave 2`, which takes effect on the next reconnect. `sudo iw dev wlan0 set power_save off` applies it right away.

## Safety

Driving the real robot is off on both ends. The station never sends it a command, since the page's drive keys only reach the sim. `pi-node` never sends the Neato a drive command either: anything that arrives from the station is logged and dropped. It only uses `GetVersion`, `TestMode`, `SetLDSRotation`, `GetMotors`, `GetDigitalSensors` and `GetLDSScan`. Enabling driving later will be a deliberate change to `pi-node`, together with the 300 ms stop watchdog from the design.
