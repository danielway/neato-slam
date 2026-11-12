#!/usr/bin/env python3
"""Asks the Neato for lidar scans back to back and reports, for each reply,
how long it took and which beams changed since the previous reply.

That answers two questions about GetLDSScan: how long one request costs, and
whether a reply is one finished sweep (all beams change at once, every
~200 ms) or a rolling buffer (only the beams the head passed since the last
request change).

Stop pi-node first, since it holds the port:

    systemctl --user stop pi-node
    python3 lds-probe.py [port] [seconds] > probe.txt
    systemctl --user start pi-node

The robot can sit still. Output: one line per reply,
`t_ms dur_ms first_byte_ms hz changed runs`, where runs lists the changed beam
indices as start-end ranges.
"""

import os
import subprocess
import sys
import time

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

subprocess.run(["stty", "-F", port, "115200", "raw", "-echo", "min", "0", "time", "2"], check=True)
fd = os.open(port, os.O_RDWR)


def query(cmd):
    """Returns (reply, seconds to first byte, seconds to end)."""
    start = time.monotonic()
    os.write(fd, (cmd + "\n").encode())
    buf, first = b"", None
    while b"\x1a" not in buf:
        chunk = os.read(fd, 8192)
        if chunk and first is None:
            first = time.monotonic() - start
        if not chunk and time.monotonic() - start > 3:
            raise RuntimeError(f"no reply to {cmd}")
        buf += chunk
    return buf.split(b"\x1a")[0].decode(errors="replace"), first or 0.0, time.monotonic() - start


def beams(reply):
    rows, hz = {}, ""
    for line in reply.splitlines():
        parts = line.strip().split(",")
        if parts[0] == "ROTATION_SPEED":
            hz = parts[1]
        elif len(parts) == 4 and parts[0].isdigit():
            rows[int(parts[0])] = tuple(parts[1:])
    return rows, hz


def runs(idx):
    out, start, prev = [], None, None
    for i in idx:
        if start is None:
            start = prev = i
        elif i == prev + 1:
            prev = i
        else:
            out.append(f"{start}-{prev}")
            start = prev = i
    if start is not None:
        out.append(f"{start}-{prev}")
    return ",".join(out) or "-"


os.write(fd, b"\n")
time.sleep(0.3)
while os.read(fd, 8192):
    pass
query("TestMode On")
query("SetLDSRotation On")
time.sleep(3)

t0 = time.monotonic()
prev = None
print("t_ms dur_ms first_byte_ms hz changed runs")
try:
    while time.monotonic() - t0 < seconds:
        t = time.monotonic() - t0
        reply, first, dur = query("GetLDSScan")
        rows, hz = beams(reply)
        if prev is not None:
            changed = sorted(i for i in rows if rows[i] != prev.get(i))
            print(f"{t*1000:.0f} {dur*1000:.0f} {first*1000:.0f} {hz} {len(changed)} {runs(changed)}")
        prev = rows
finally:
    query("SetLDSRotation Off")
    query("TestMode Off")
