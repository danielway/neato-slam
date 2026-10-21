#!/usr/bin/env bash
# Captures raw replies to the Neato serial commands the stack uses, so the
# parsers in crates/neato can be tested against real output.
#
# Run on the Pi (or any Linux machine) with the Neato plugged in over USB:
#   tools/capture-neato.sh [/dev/ttyACM0] [output-dir]
#
# It only reads sensors and spins the lidar; it never drives the wheels.
set -euo pipefail

port="${1:-/dev/ttyACM0}"
out="${2:-neato-capture}"
mkdir -p "$out"

stty -F "$port" 115200 raw -echo
exec 3<>"$port"

# Sends one command and saves everything the robot replies within $2 seconds.
send() {
  local cmd="$1" wait="${2:-2}"
  printf '%s\r\n' "$cmd" >&3
  timeout "$wait" cat <&3 > "$out/$(echo "$cmd" | tr ' ' '_').txt" || true
  echo "captured: $cmd"
}

send "GetVersion"
send "TestMode On"
send "SetLDSRotation On" 1
sleep 3 # let the lidar reach speed
send "GetLDSScan" 3
send "GetMotors"
echo "Hold the front-left bumper pressed, then press Enter."
read -r
send "GetDigitalSensors"
mv "$out/GetDigitalSensors.txt" "$out/GetDigitalSensors_pressed.txt"
echo "Release the bumper, then press Enter."
read -r
send "GetDigitalSensors"
send "SetLDSRotation Off" 1
send "TestMode Off" 1

echo "Done. Files are in $out/"
