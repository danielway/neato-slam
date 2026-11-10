#!/usr/bin/env bash
# Builds pi-node for the Pi, copies it over, and restarts its service. Run it
# from the station laptop, which needs key-based ssh to the Pi:
#
#   tools/deploy-pi.sh [user@host]
#
# The previous binary is kept on the Pi as pi-node.prev.
set -euo pipefail

host="${1:-danielway@192.168.4.61}"
repo="$(cd "$(dirname "$0")/.." && pwd)"
bin="$repo/target/aarch64-unknown-linux-musl/release/pi-node"

cargo zigbuild --release -p neato --bin pi-node --target aarch64-unknown-linux-musl \
  --manifest-path "$repo/Cargo.toml"
scp -o BatchMode=yes "$bin" "$host:neato-slam/target/release/pi-node.new"
ssh -o BatchMode=yes "$host" '
  set -e
  cd ~/neato-slam/target/release
  if [ -e pi-node ]; then cp pi-node pi-node.prev; fi
  mv pi-node.new pi-node
  systemctl --user restart pi-node
  sleep 2
  systemctl --user is-active pi-node
  journalctl --user-unit pi-node -b --no-pager | tail -8
'
