#!/usr/bin/env python3
"""Calibrates the ceiling camera against the taped arena, from one frame.

The arena's inside tape edges are straight lines on a flat floor, and
arena.toml says where its corners are. That is enough to solve for the lens's
radial distortion and the floor homography together: undistort the tape edge
pixels, map them to the floor through the homography that puts the four
corners where they were measured, and adjust the lens until every edge pixel
lands on its measured edge line. The leftover distance is the calibration's
error, in millimetres on the floor.

Run on the station, with a frame from the ceiling camera:

    tools/camera-calibrate.py frame.jpg --out camera.toml --overlay overlay.png --topdown topdown.png

Needs python3-opencv and python3-numpy.
"""

import argparse
import datetime
import sys

import cv2
import numpy as np

from ceiling import NAMES, Lens, apply, load_arena

# Station palette (crates/station/web/index.html), BGR.
TRUTH = (115, 204, 76)
LINE = (226, 219, 216)


def tape_mask(img):
    """Pixels that look like the blue tape, in sun or shade."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (80, 70, 40), (135, 255, 255))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)


def inside_edge(img):
    """Contour of the floor enclosed by the tape: the inside edge of the tape."""
    h, w = img.shape[:2]
    free = cv2.bitwise_not(tape_mask(img))
    count, labels = cv2.connectedComponents(free, connectivity=4)
    label = labels[h // 2, w // 2]
    if label == 0:
        sys.exit("the image centre is on tape; is the arena in view?")
    region = (labels == label).astype(np.uint8)
    touches = sum(
        bool(edge.any()) for edge in (region[0], region[-1], region[:, 0], region[:, -1])
    )
    if touches >= 3:
        sys.exit("the floor inside the tape leaks out of the frame; is the tape loop closed and in view?")
    contours, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return max(contours, key=cv2.contourArea)[:, 0, :].astype(np.float64)


def split_sides(contour, shape, corner_gap=20, border_gap=3):
    """The four corners and the edge pixels between them, named as in arena.toml."""
    s, d = contour.sum(1), contour[:, 0] - contour[:, 1]
    idx = {
        "top_left": int(np.argmin(s)),
        "bottom_right": int(np.argmax(s)),
        "top_right": int(np.argmax(d)),
        "bottom_left": int(np.argmin(d)),
    }
    corners = {k: contour[i] for k, i in idx.items()}
    h, w = shape[:2]
    on_border = (
        (contour[:, 0] < border_gap)
        | (contour[:, 1] < border_gap)
        | (contour[:, 0] > w - 1 - border_gap)
        | (contour[:, 1] > h - 1 - border_gap)
    )
    sides = {}
    for a, b in [
        ("top_left", "top_right"),
        ("top_right", "bottom_right"),
        ("bottom_right", "bottom_left"),
        ("bottom_left", "top_left"),
    ]:
        # The contour may run either way round; take the run between the two
        # corners that holds neither of the other corners.
        i, j, n = idx[a], idx[b], len(contour)
        run = np.arange(i, j + 1) if i <= j else np.r_[i:n, 0 : j + 1]
        others = {idx[k] for k in idx if k not in (a, b)}
        if others & set(run.tolist()):
            run = np.arange(j, i + 1) if j <= i else np.r_[j:n, 0 : i + 1]
        pts = contour[run]
        far = (np.linalg.norm(pts - corners[a], axis=1) > corner_gap) & (
            np.linalg.norm(pts - corners[b], axis=1) > corner_gap
        )
        pts = pts[far & ~on_border[run]]
        if len(pts) > 300:
            pts = pts[np.linspace(0, len(pts) - 1, 300).astype(int)]
        sides[(a, b)] = pts
    return corners, sides


def homography(src, dst):
    h, _ = cv2.findHomography(np.asarray(src, np.float64), np.asarray(dst, np.float64), 0)
    return h


def residuals(params, f, corners, sides, arena):
    lens = Lens(f, *params)
    h = homography([lens.undistort(corners[k][None])[0] for k in NAMES], [arena[k] for k in NAMES])
    out = []
    for (a, b), pts in sides.items():
        p = apply(h, lens.undistort(pts))
        u = arena[b] - arena[a]
        normal = np.array([-u[1], u[0]]) / np.linalg.norm(u)
        out.append((p - arena[a]) @ normal)
    return out, h


def fit(f, corners, sides, arena, shape):
    """Levenberg-Marquardt: k1 and k2 about the image centre first, then the
    distortion centre too, kept within a tenth of the image of the middle."""
    h, w = shape[:2]
    mid = np.array([w / 2, h / 2])

    def solve(p, free):
        step = np.array([1e-4, 1e-4, 0.5, 0.5])

        def r(p):
            if np.abs(p[2:] - mid).max() > w / 10:
                return None
            return np.concatenate(residuals(p, f, corners, sides, arena)[0])

        lam, cur = 1e-3, r(p)
        for _ in range(100):
            jac = np.stack([(r(p + e * step[i]) - cur) / step[i] for i, e in zip(free, np.eye(4)[free])], 1)
            jtj, g = jac.T @ jac, jac.T @ cur
            while True:
                dp = np.zeros(4)
                dp[free] = -np.linalg.solve(jtj + lam * np.diag(np.diag(jtj)), g)
                new = r(p + dp)
                if new is not None and new @ new < cur @ cur:
                    p, cur, lam = p + dp, new, lam / 3
                    break
                lam *= 4
                if lam > 1e10:
                    return p
            if np.abs(dp[:2]).max() < 1e-7 and np.abs(dp[2:]).max() < 1e-2:
                break
        return p

    p = solve(np.array([0.0, 0.0, *mid]), [0, 1])
    return solve(p, [0, 1, 2, 3])


def to_image(lens, hinv, pts):
    return lens.distort(apply(hinv, pts))


def inside(arena, pts):
    """Which points lie inside the (convex) arena, corners clockwise from top left."""
    ok = np.ones(len(pts), bool)
    for a, b in zip(NAMES, NAMES[1:] + NAMES[:1]):
        e = arena[b] - arena[a]
        ok &= (pts - arena[a]) @ np.array([e[1], -e[0]]) >= -1e-9
    return ok


def draw_overlay(img, lens, hinv, arena, path):
    out = img.copy()
    xs = [c[0] for c in arena.values()]
    ys = [c[1] for c in arena.values()]
    lines = [np.c_[np.full(400, x), np.linspace(min(ys), max(ys), 400)] for x in np.arange(0.5, max(xs), 0.5)]
    lines += [np.c_[np.linspace(min(xs), max(xs), 800), np.full(800, y)] for y in np.arange(0.5, max(ys), 0.5)]
    for line in lines:
        line = line[inside(arena, line)]
        cv2.polylines(out, [np.round(to_image(lens, hinv, line)).astype(np.int32)], False, LINE, 1, cv2.LINE_AA)
    for a, b in zip(NAMES, NAMES[1:] + NAMES[:1]):
        t = np.linspace(0, 1, 400)[:, None]
        line = to_image(lens, hinv, arena[a] + t * (arena[b] - arena[a]))
        cv2.polylines(out, [np.round(line).astype(np.int32)], False, TRUTH, 2, cv2.LINE_AA)
    cv2.imwrite(path, out)


def draw_topdown(img, lens, hinv, arena, path, mm_per_px=4.0, margin=0.1):
    xs = [c[0] for c in arena.values()]
    ys = [c[1] for c in arena.values()]
    x0, x1, y0, y1 = min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin
    s = mm_per_px / 1000
    w, h = int((x1 - x0) / s), int((y1 - y0) / s)
    gx, gy = np.meshgrid(x0 + (np.arange(w) + 0.5) * s, y1 - (np.arange(h) + 0.5) * s)
    px = to_image(lens, hinv, np.c_[gx.ravel(), gy.ravel()]).astype(np.float32)
    out = cv2.remap(img, px[:, 0].reshape(h, w), px[:, 1].reshape(h, w), cv2.INTER_LINEAR)
    poly = np.array([[(arena[k][0] - x0) / s, (y1 - arena[k][1]) / s] for k in NAMES])
    cv2.polylines(out, [np.round(poly).astype(np.int32)], True, TRUTH, 1, cv2.LINE_AA)
    cv2.imwrite(path, out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("frame")
    ap.add_argument("--arena", default="arena.toml")
    ap.add_argument("--out", default="camera.toml")
    ap.add_argument("--overlay", help="save the frame with the fitted arena grid drawn on it")
    ap.add_argument("--topdown", help="save the floor seen straight down, 4 mm per pixel")
    ap.add_argument("--mask", help="save what was taken for tape, with the inside edge and corners found")
    ap.add_argument("--focal", type=float, help="nominal focal length in pixels (default: half the width)")
    args = ap.parse_args()

    img = cv2.imread(args.frame)
    if img is None:
        sys.exit(f"can't read {args.frame}")
    arena = load_arena(args.arena)

    contour = inside_edge(img)
    corners, sides = split_sides(contour, img.shape)
    if args.mask:
        dbg = cv2.cvtColor(tape_mask(img), cv2.COLOR_GRAY2BGR) // 3
        for pts in sides.values():
            for x, y in np.round(pts).astype(int):
                cv2.circle(dbg, (x, y), 2, TRUTH, -1)
        for x, y in np.round(list(corners.values())).astype(int):
            cv2.circle(dbg, (x, y), 8, LINE, 2)
        cv2.imwrite(args.mask, dbg)
        print(f"wrote {args.mask}")
    f = args.focal or img.shape[1] / 2
    params = fit(f, corners, sides, arena, img.shape)
    res, h = residuals(params, f, corners, sides, arena)
    lens, hinv = Lens(f, *params), np.linalg.inv(h)
    allres = np.concatenate(res) * 1000
    before = np.concatenate(residuals([0, 0, img.shape[1] / 2, img.shape[0] / 2], f, corners, sides, arena)[0]) * 1000

    print("corners (px): " + ", ".join(f"{k} ({corners[k][0]:.0f}, {corners[k][1]:.0f})" for k in NAMES))
    print("edge pixels: " + ", ".join(f"{a}-{b} {len(p)}" for (a, b), p in sides.items()))
    print(f"lens: k1 {lens.k1:.5f}, k2 {lens.k2:.5f}, centre ({lens.cx:.1f}, {lens.cy:.1f}) at f {f:.0f}")
    print(f"edge error before lens fit: rms {np.sqrt((before**2).mean()):.1f} mm, max {np.abs(before).max():.1f} mm")
    print(f"edge error after lens fit:  rms {np.sqrt((allres**2).mean()):.1f} mm, max {np.abs(allres).max():.1f} mm")
    for ((a, b), r) in zip(sides, res):
        print(f"  {a}-{b}: rms {np.sqrt((r**2).mean()) * 1000:.1f} mm, max {np.abs(r).max() * 1000:.1f} mm")

    rows = ",\n  ".join("[" + ", ".join(f"{v:.9e}" for v in row) + "]" for row in h)
    with open(args.out, "w") as out:
        out.write(
            f"""# Ceiling camera calibration, written by tools/camera-calibrate.py.
# Pixel -> floor: undistort the pixel with [lens] (OpenCV's model, normalised
# by fx), then apply [floor].homography to get arena metres (see arena.toml).

[image]
width = {img.shape[1]}
height = {img.shape[0]}

[lens]
fx = {f:.3f}  # nominal; one square-on view of the floor can't separate it from height
fy = {f:.3f}
cx = {lens.cx:.3f}
cy = {lens.cy:.3f}
k1 = {lens.k1:.9f}
k2 = {lens.k2:.9f}

[floor]
homography = [
  {rows},
]

[fit]
frame = "{args.frame}"
date = {datetime.date.today().isoformat()}
edge_pixels = {len(allres)}
edge_rms_mm = {np.sqrt((allres**2).mean()):.2f}
edge_max_mm = {np.abs(allres).max():.2f}
"""
        )
    print(f"wrote {args.out}")
    if args.overlay:
        draw_overlay(img, lens, hinv, arena, args.overlay)
        print(f"wrote {args.overlay}")
    if args.topdown:
        draw_topdown(img, lens, hinv, arena, args.topdown)
        print(f"wrote {args.topdown}")


if __name__ == "__main__":
    main()
