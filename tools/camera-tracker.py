#!/usr/bin/env python3
"""Tracks the robot's marker with the ceiling camera and prints its pose.

The station runs this when started with --camera, and turns its output into
ground truth. It prints one JSON object per line on stdout:

    {"topdown": "PATH", "bounds": [x0, y0, x1, y1]}
        once at the start with --topdown: the floor seen straight down is
        saved to PATH every second, covering those arena coordinates.
    {"age_ms": 41.5, "pose": [x, y, theta], "tag_px": [[u, v], ...], "side_m": 0.1}
        for each frame the marker is found in. pose is the robot's, in the
        arena frame of arena.toml; side_m is the marker's mean side as
        measured, a check on the calibration and the marker's height.
    {"age_ms": 40.0, "pose": null}
        at most once a second while the marker isn't found.

age_ms is how long before the line was printed the frame was taken,
including the camera's own delay from camera.toml, so the reader can stamp
the pose on its own clock.

Test it on a still frame without the station:

    tools/camera-tracker.py --image frame.jpg

Needs python3-opencv (with the aruco module) and python3-numpy.
"""

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
import tomllib

import cv2
import numpy as np

from ceiling import Camera, load_arena

# How far around the last sighting to look first, in pixels.
SEARCH_MARGIN = 150
TOPDOWN_MM_PER_PX = 8.0
TOPDOWN_MARGIN_M = 0.15
# While the marker is lost, how often to search the whole frame. A search
# takes most of the laptop for 40 ms, so doing it every frame would starve
# the station.
FULL_SEARCH_EVERY_S = 0.25


def detector():
    """AprilTag 36h11 detection, for OpenCV before and after 4.7's API change."""
    tags = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return cv2.aruco.ArucoDetector(tags, params).detectMarkers
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return lambda img: cv2.aruco.detectMarkers(img, tags, parameters=params)


class Marker:
    """The [marker] section of robot.toml."""

    def __init__(self, path):
        with open(path, "rb") as f:
            m = tomllib.load(f)["marker"]
        self.id, self.size = m["id"], m["size_m"]
        self.height, self.x, self.y, self.theta = m["height_m"], m["x_m"], m["y_m"], m["theta_rad"]


def robot_pose(cam, marker, corners):
    """The robot's pose from the marker's four corners in pixels, in the
    order OpenCV gives them: the printed tag's top left, top right, bottom
    right, bottom left, where its top is the robot's forward arrow."""
    pts = cam.to_arena(corners, marker.height)
    centre = pts.mean(0)
    up = (pts[0] + pts[1] - pts[2] - pts[3]) / 2
    theta = math.atan2(up[1], up[0]) - marker.theta
    c, s = math.cos(theta), math.sin(theta)
    x = centre[0] - (c * marker.x - s * marker.y)
    y = centre[1] - (s * marker.x + c * marker.y)
    theta = math.atan2(math.sin(theta), math.cos(theta))
    side = np.linalg.norm(pts - np.roll(pts, 1, axis=0), axis=1).mean()
    return [x, y, theta], side


class Finder:
    """Finds the marker, looking near its last sighting first."""

    def __init__(self, marker):
        self.marker, self.detect, self.last = marker, detector(), None
        self.searched = -math.inf

    def find(self, gray, now=math.inf):
        """The marker's corners, or None. The whole frame is only searched
        once per FULL_SEARCH_EVERY_S, judged by `now`."""
        if self.last is not None:
            h, w = gray.shape
            (u0, v0), (u1, v1) = self.last.min(0), self.last.max(0)
            u0, v0 = max(int(u0) - SEARCH_MARGIN, 0), max(int(v0) - SEARCH_MARGIN, 0)
            u1, v1 = min(int(u1) + SEARCH_MARGIN, w), min(int(v1) + SEARCH_MARGIN, h)
            found = self._find(gray[v0:v1, u0:u1])
            if found is not None:
                self.last = found + [u0, v0]
                return self.last
        if now - self.searched < FULL_SEARCH_EVERY_S:
            self.last = None
            return None
        self.searched = now
        self.last = self._find(gray)
        return self.last

    def _find(self, gray):
        corners, ids, _ = self.detect(gray)
        if ids is None:
            return None
        for c, i in zip(corners, ids.ravel()):
            if i == self.marker.id:
                return c.reshape(4, 2).astype(float)
        return None


class Grabber(threading.Thread):
    """Keeps only the newest frame, so detection never works on a stale one."""

    def __init__(self, cap):
        super().__init__(daemon=True)
        self.cap, self.cond = cap, threading.Condition()
        self.frame, self.at, self.seq, self.error = None, 0.0, 0, None

    def run(self):
        while True:
            ok, frame = self.cap.read()
            at = time.monotonic()
            with self.cond:
                if not ok:
                    self.error = "the camera stopped sending frames"
                else:
                    self.frame, self.at, self.seq = frame, at, self.seq + 1
                self.cond.notify_all()
            if not ok:
                return

    def next(self, after):
        with self.cond:
            self.cond.wait_for(lambda: self.seq > after or self.error, timeout=5)
            if self.error or self.seq <= after:
                sys.exit(f"camera-tracker: {self.error or 'no frame from the camera in 5 s'}")
            return self.frame, self.at, self.seq


class TopDown:
    """The floor seen straight down, saved now and then for the station's map."""

    def __init__(self, cam, arena, path):
        xs = [c[0] for c in arena.values()]
        ys = [c[1] for c in arena.values()]
        m, s = TOPDOWN_MARGIN_M, TOPDOWN_MM_PER_PX / 1000
        self.bounds = [min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m]
        x0, y0, x1, y1 = self.bounds
        w, h = int((x1 - x0) / s), int((y1 - y0) / s)
        gx, gy = np.meshgrid(x0 + (np.arange(w) + 0.5) * s, y1 - (np.arange(h) + 0.5) * s)
        px = cam.to_image(np.c_[gx.ravel(), gy.ravel()]).astype(np.float32)
        self.map_u, self.map_v = px[:, 0].reshape(h, w), px[:, 1].reshape(h, w)
        self.path, self.saved = path, 0.0

    def maybe_save(self, frame, now, every=1.0):
        if now - self.saved < every:
            return
        self.saved = now
        img = cv2.remap(frame, self.map_u, self.map_v, cv2.INTER_LINEAR)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            tmp = self.path + ".tmp"
            with open(tmp, "wb") as f:
                f.write(jpg.tobytes())
            os.replace(tmp, self.path)


def emit(obj):
    try:
        print(json.dumps(obj, separators=(",", ":")), flush=True)
    except BrokenPipeError:
        # The station has gone; leave without unwinding the camera thread.
        os._exit(0)


def describe(cam, marker, finder, gray, now=math.inf):
    corners = finder.find(gray, now)
    if corners is None:
        return None
    pose, side = robot_pose(cam, marker, corners)
    return {
        "pose": [round(v, 4) for v in pose],
        "tag_px": np.round(corners, 1).tolist(),
        "side_m": round(float(side), 4),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--image", help="find the marker in one still frame instead, and exit")
    ap.add_argument("--camera", default="camera.toml")
    ap.add_argument("--arena", default="arena.toml")
    ap.add_argument("--robot", default="robot.toml")
    ap.add_argument("--topdown", help="save the floor seen straight down to this JPEG every second")
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
    # Leave the station a core or two.
    cv2.setNumThreads(2)

    cam, marker = Camera(args.camera), Marker(args.robot)
    with open(args.camera, "rb") as f:
        latency = tomllib.load(f).get("timing", {}).get("latency_ms", 0.0)
    finder = Finder(marker)

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            sys.exit(f"camera-tracker: can't read {args.image}")
        t0 = time.monotonic()
        found = describe(cam, marker, finder, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        emit({**(found or {"pose": None}), "detect_ms": round((time.monotonic() - t0) * 1000, 1)})
        return

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam.height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        sys.exit(f"camera-tracker: can't open {args.device}")
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (w, h) != (cam.width, cam.height):
        sys.exit(f"camera-tracker: {args.device} gives {w}x{h}, but it was calibrated at {cam.width}x{cam.height}")

    topdown = None
    if args.topdown:
        topdown = TopDown(cam, load_arena(args.arena), args.topdown)
        emit({"topdown": args.topdown, "bounds": [round(v, 4) for v in topdown.bounds]})

    grabber = Grabber(cap)
    grabber.start()
    seq, missed_said = 0, 0.0
    while True:
        frame, at, seq = grabber.next(seq)
        found = describe(cam, marker, finder, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), at)
        now = time.monotonic()
        age = round((now - at) * 1000 + latency, 1)
        if found:
            emit({"age_ms": age, **found})
        elif now - missed_said >= 1.0:
            missed_said = now
            emit({"age_ms": age, "pose": None})
        if topdown:
            topdown.maybe_save(frame, now)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        os._exit(0)
