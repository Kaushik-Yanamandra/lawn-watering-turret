"""Camera side: map pixels to lawn coordinates, find where the water landed,
and turn thermal frames into per-cell dryness scores.

Both cameras look at a flat lawn, so a single homography per camera (per
pan stop, for the thermal camera) maps image pixels to ground (x, y) in
metres. Set it once by placing 4+ markers (golf tees with white caps work,
cups of hot water for the thermal camera) at measured positions and clicking them.
"""
from __future__ import annotations

import cv2
import numpy as np


class LensModel:
    """Pinhole intrinsics + distortion for one camera, from a checkerboard
    calibration (tools/calibrate_lens.py). The Camera Module 3 Wide has strong
    barrel distortion; a single homography on raw pixels bends straight lines
    and puts errors of several decimetres into the ground mapping at the
    frame edges. Points are undistorted before the homography is applied."""

    def __init__(self, K, D):
        self.K = np.asarray(K, float)
        self.D = np.asarray(D, float).ravel()

    @classmethod
    def load(cls, path):
        z = np.load(path)
        return cls(z["K"], z["D"])

    def undistort(self, pts):
        """Raw pixel points (N, 2) -> ideal pinhole pixel points (N, 2)."""
        p = np.asarray(pts, float).reshape(-1, 1, 2)
        crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-9)
        return cv2.undistortPoints(p, self.K, self.D, None, self.K, criteria=crit).reshape(-1, 2)

    def distort(self, pts):
        """Ideal pinhole pixel points -> raw pixel points."""
        p = np.asarray(pts, float).reshape(-1, 2)
        n = cv2.undistortPoints(p.reshape(-1, 1, 2), self.K, np.zeros(5)).reshape(-1, 2)    # to normalised
        obj = np.column_stack([n, np.ones(len(n))])
        out, _ = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), self.K, self.D)
        return out.reshape(-1, 2)


class GroundMapper:
    """Pixel <-> ground (metres) mapping for one fixed camera.

    lens: optional LensModel. With it, marker pixels and every later pixel
    are undistorted first, so the homography models a true pinhole view."""

    def __init__(self, pixel_pts, ground_pts, lens: LensModel | None = None):
        self.lens = lens
        px = np.asarray(pixel_pts, dtype=np.float64).reshape(-1, 2)
        if lens is not None:
            px = lens.undistort(px)
        gd = np.asarray(ground_pts, dtype=np.float64).reshape(-1, 1, 2)
        if len(px) < 4:
            raise ValueError("need at least 4 marker correspondences")
        self.H, _ = cv2.findHomography(px.reshape(-1, 1, 2), gd, method=0)
        self.Hinv = np.linalg.inv(self.H)

    def to_ground(self, u, v):
        if self.lens is not None:
            u, v = self.lens.undistort([[u, v]])[0]
        p = cv2.perspectiveTransform(np.array([[[u, v]]], dtype=np.float64), self.H)
        return float(p[0, 0, 0]), float(p[0, 0, 1])

    def to_pixel(self, x, y):
        p = cv2.perspectiveTransform(np.array([[[x, y]]], dtype=np.float64), self.Hinv)
        u, v = float(p[0, 0, 0]), float(p[0, 0, 1])
        if self.lens is not None:
            u, v = (float(c) for c in self.lens.distort([[u, v]])[0])
        return u, v

    def save(self, path):
        np.savez(path, H=self.H)

    @classmethod
    def load(cls, path, lens: LensModel | None = None):
        obj = cls.__new__(cls)
        obj.lens = lens
        obj.H = np.load(path)["H"]
        obj.Hinv = np.linalg.inv(obj.H)
        return obj


def _gray(img):
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(img.astype(np.float32), (7, 7), 0)


def detect_landing(before, after, mapper: GroundMapper, expected_xy=None,
                   search_radius_m: float = 2.0, min_area_px: int = 30,
                   thresh: float = 12.0):
    """Locate a shot's landing point from two frames.

    before  frame just before the burst
    after   frame during or just after the burst

    Works on the absolute difference, so it catches both wet turf going dark
    (dry lawn, calibration) and white splash at the impact point (dew-wet
    lawn at dawn). The airborne stream also shows up in the difference, but
    it is above the ground plane; restricting the search to blobs near the
    expected landing point rejects it.

    Returns (x, y, area_px) in metres, or None.
    """
    b, a = _gray(before), _gray(after)
    # Remove global brightness change (cloud, auto-exposure) before differencing.
    a = a * (np.median(b) / max(np.median(a), 1e-3))
    diff = np.abs(a - b)
    mask = (diff > thresh).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)

    hgt, wid = mask.shape
    best, best_score = None, None
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_area_px:
            continue
        # A splash cut off by the frame edge gives a biased centroid: ignore it.
        l, t = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
        if l <= 1 or t <= 1 or l + stats[i, cv2.CC_STAT_WIDTH] >= wid - 1 \
                or t + stats[i, cv2.CC_STAT_HEIGHT] >= hgt - 1:
            continue
        # Weighted centroid: pixels that changed most count most.
        ys, xs = np.nonzero(labels == i)
        w = diff[ys, xs]
        u, v = float(np.sum(xs * w) / np.sum(w)), float(np.sum(ys * w) / np.sum(w))
        gx, gy = mapper.to_ground(u, v)
        if expected_xy is not None:
            d = float(np.hypot(gx - expected_xy[0], gy - expected_xy[1]))
            if d > search_radius_m:
                continue
            score = -d
        else:
            score = area
        if best_score is None or score > best_score:
            best, best_score = (gx, gy, int(area)), score
    return best


def _bilinear(img, u, v):
    """Bilinear sample of a 2-D array at pixel (u, v); NaN outside."""
    hgt, wid = img.shape
    if not (0 <= u <= wid - 1 and 0 <= v <= hgt - 1):
        return np.nan
    u0, v0 = int(min(u, wid - 2)), int(min(v, hgt - 2))
    fu, fv = u - u0, v - v0
    return float((1 - fu) * (1 - fv) * img[v0, u0] + fu * (1 - fv) * img[v0, u0 + 1]
                 + (1 - fu) * fv * img[v0 + 1, u0] + fu * fv * img[v0 + 1, u0 + 1])


def _cell_samples(x, y, half=0.25):
    """Four points inside a 1 m cell. The MLX90640 pixel footprint is
    0.3-0.5 m across and up to 1.2 m long at the far edge, so sampling
    inside the cell (instead of averaging a 3x3 pixel patch, which reached
    1.4 x 3.6 m) keeps paving and the turret post out of edge cells."""
    return [(x - half, y - half), (x + half, y - half), (x + half, y + half), (x - half, y + half)]


def thermal_cell_temps(thermal_c, mapper: GroundMapper, cells, edge: int = 1):
    """Surface temperature (deg C) of each cell: mean of bilinear samples at
    four points inside the cell. NaN when the cell is not fully in view.

    thermal_c  2-D array of temperatures from the thermal camera
    cells      iterable of (x, y) cell centres in metres
    """
    hgt, wid = thermal_c.shape
    out = []
    for (x, y) in cells:
        vals = []
        for sx, sy in _cell_samples(x, y):
            u, v = mapper.to_pixel(sx, sy)
            if not (edge <= u <= wid - 1 - edge and edge <= v <= hgt - 1 - edge):
                vals = []
                break
            vals.append(_bilinear(thermal_c, u, v))
        out.append(float(np.mean(vals)) if vals else np.nan)
    return np.array(out)


def thermal_cell_temps_multi(frames: dict, mappers: dict, cells, edge: int = 2):
    """Cell temperatures from a thermal camera panned to several stops.

    frames, mappers: {stop_angle: 24x32 array}, {stop_angle: GroundMapper}.
    Each cell is read from the stop where it sits closest to the frame's
    centre column (least lens distortion), skipping the outer `edge` pixels.
    """
    out = []
    for (x, y) in cells:
        best, best_d = np.nan, None
        for stop, frame in frames.items():
            hgt, wid = frame.shape
            u, v = mappers[stop].to_pixel(x, y)
            if not (edge <= u <= wid - 1 - edge and edge <= v <= hgt - 1 - edge):
                continue
            d = abs(u - (wid - 1) / 2)
            if best_d is None or d < best_d:
                t = thermal_cell_temps(frame, mappers[stop], [(x, y)], edge=edge)[0]
                if np.isfinite(t):
                    best, best_d = t, d
        out.append(best)
    return np.array(out)


def dryness_scores(cell_temps):
    """Temperature excess over the lawn's median (deg C). Positive = drier."""
    t = np.asarray(cell_temps, dtype=float)
    return t - np.nanmedian(t)
