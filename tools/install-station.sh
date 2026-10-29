#!/usr/bin/env bash
# Sets up the station laptop to run like an appliance: the station starts on
# boot with the ceiling camera, and the desktop opens its page fullscreen on
# login. Run it from the repo folder on the station laptop, as the user who
# will own the desktop (it asks for sudo once, for auto-login):
#
#   tools/install-station.sh [/dev/video0]
#
# Run it again after moving the repo; it is safe to repeat. Undo with
#   systemctl --user disable --now station; rm ~/.config/autostart/station.desktop
set -euo pipefail

camera="${1:-/dev/video0}"
repo="$(cd "$(dirname "$0")/.." && pwd)"
port=8080

cargo build --release -p station --manifest-path "$repo/Cargo.toml"

# The station, as a user service that restarts if it stops. Lingering starts
# it at boot without anyone logging in.
mkdir -p ~/.config/systemd/user
cat > ~/.config/systemd/user/station.service <<EOF
[Unit]
Description=neato-slam station with the ceiling camera
After=network-online.target

[Service]
WorkingDirectory=$repo
ExecStart=$repo/target/release/station --listen 0.0.0.0:$port --camera $camera
Restart=always
RestartSec=2

[Install]
WantedBy=default.target
EOF
systemctl --user daemon-reload
systemctl --user enable station
systemctl --user restart station
sudo loginctl enable-linger "$USER"

# The camera needs the video group when nobody is logged in at the desk.
if ! id -nG "$USER" | grep -qw video; then
  sudo usermod -aG video "$USER"
  echo "Added $USER to the video group; it applies after a reboot."
fi

# The page, fullscreen in its own window, whenever the desktop starts.
browser=""
for b in chromium chromium-browser google-chrome firefox; do
  if command -v "$b" > /dev/null; then browser="$b"; break; fi
done
if [ -z "$browser" ]; then
  echo "No browser found for the station window; install chromium or firefox." >&2
  exit 1
fi
if [ "$browser" = firefox ]; then
  open="firefox --kiosk http://localhost:$port"
else
  open="$browser --app=http://localhost:$port --start-fullscreen --no-first-run --password-store=basic"
fi
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/station.desktop <<EOF
[Desktop Entry]
Type=Application
Name=neato-slam station
Comment=The station page, fullscreen
Exec=sh -c 'sleep 3; $open'
X-GNOME-Autostart-enabled=true
EOF

# Log the desktop straight in, so the page comes up after a power cut.
if [ -d /etc/lightdm ]; then
  sudo mkdir -p /etc/lightdm/lightdm.conf.d
  printf '[Seat:*]\nautologin-user=%s\nautologin-user-timeout=0\n' "$USER" |
    sudo tee /etc/lightdm/lightdm.conf.d/50-station-autologin.conf > /dev/null
fi

echo "Station running on port $port with $camera; the page opens with $browser at login."
echo "From another computer: http://$(hostname -I | awk '{print $1}'):$port"
