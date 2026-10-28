"""The ceiling camera's geometry: its lens, the floor homography and the
arena, shared by camera-calibrate.py and camera-tracker.py.

Pixels map to the arena frame of arena.toml, in metres. A point above the
floor is seen along the same ray as the floor point behind it, so it maps
further from the spot under the lens than it really is; `to_arena` takes the
point's height to undo that.
"""

import tomllib

import numpy as np

# Arena corners clockwise from top left, as arena.toml names them.
NAMES = ["top_left", "top_right", "bottom_right", "bottom_left"]


class Lens:
    """OpenCV's pinhole model with two radial terms. The focal length is fixed
    at a nominal value: one view of a plane square-on to the camera can't tell
    focal length from height, and the floor homography absorbs it."""

    def __init__(self, f, k1, k2, cx, cy):
        self.f, self.k1, self.k2, self.cx, self.cy = f, k1, k2, cx, cy

    def undistort(self, px):
        """Pixels to undistorted normalised coordinates, by Newton on the radius."""
        xd = (np.asarray(px, float).reshape(-1, 2) - [self.cx, self.cy]) / self.f
        rd = np.linalg.norm(xd, axis=1)
        r = rd.copy()
        for _ in range(30):
            r2 = r * r
            g = r * (1 + self.k1 * r2 + self.k2 * r2 * r2) - rd
            r = r - g / (1 + 3 * self.k1 * r2 + 5 * self.k2 * r2 * r2)
        scale = np.divide(r, rd, out=np.ones_like(rd), where=rd > 0)
        return xd * scale[:, None]

    def distort(self, xy):
        r2 = (xy**2).sum(1, keepdims=True)
        xy = xy * (1 + self.k1 * r2 + self.k2 * r2**2)
        return xy * self.f + [self.cx, self.cy]


def apply(h, pts):
    p = np.c_[pts, np.ones(len(pts))] @ np.asarray(h).T
    return p[:, :2] / p[:, 2:]


def load_arena(path="arena.toml"):
    with open(path, "rb") as f:
        return {k: np.array(v, float) for k, v in tomllib.load(f)["corners"].items()}


class Camera:
    """A calibration from camera.toml: pixels to arena metres and back."""

    def __init__(self, path="camera.toml"):
        with open(path, "rb") as f:
            cal = tomllib.load(f)
        lens = cal["lens"]
        self.lens = Lens(lens["fx"], lens["k1"], lens["k2"], lens["cx"], lens["cy"])
        self.width, self.height = cal["image"]["width"], cal["image"]["height"]
        self.h = np.array(cal["floor"]["homography"], float)
        self.hinv = np.linalg.inv(self.h)
        self.mount_height = cal["mount"]["height_m"]
        # Taken as straight down: the floor under the optical axis. A tilt of
        # a few degrees moves a point 10 cm up by under a centimetre.
        self.nadir = apply(self.h, np.zeros((1, 2)))[0]

    def to_arena(self, px, height=0.0):
        """Pixels to arena metres, for points `height` metres above the floor."""
        floor = apply(self.h, self.lens.undistort(px))
        return self.nadir + (floor - self.nadir) * (1 - height / self.mount_height)

    def to_image(self, pts, height=0.0):
        """Arena metres, `height` above the floor, to pixels."""
        floor = self.nadir + (np.asarray(pts, float) - self.nadir) / (1 - height / self.mount_height)
        return self.lens.distort(apply(self.hinv, floor))
