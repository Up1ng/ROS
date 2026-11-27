"""Detect a plausible pair of painted boundaries in multiple ground arcs."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectorConfig:
    roi_top: float = 0.38
    roi_bottom: float = 0.94
    target_x: float = 0.5
    yellow_left: bool = True
    yellow_lower: tuple = (15, 90, 100)
    yellow_upper: tuple = (42, 255, 255)
    white_lower: tuple = (0, 0, 228)
    white_upper: tuple = (179, 62, 255)
    camera_height: float = 0.14
    camera_pitch: float = 0.20
    camera_hfov: float = 1.51843645
    near_radius: float = 0.24
    far_radius: float = 0.85
    target_radius: float = 0.46
    min_width_m: float = 0.18
    max_width_m: float = 0.49
    scan_count: int = 28
    min_pairs: int = 4

    def validate(self):
        if not (0 <= self.roi_top < self.roi_bottom <= 1
                and self.roi_bottom - self.roi_top >= 0.05):
            raise ValueError('ROI must span at least 0.05 of image height')
        if not 0 < self.target_x < 1:
            raise ValueError('target_x must be within the image')
        if not (0.02 < self.camera_height < 1 and 0 <= self.camera_pitch <= 0.8
                and 0.2 < self.camera_hfov < 2.6):
            raise ValueError('invalid camera calibration')
        if not 0.10 < self.near_radius < self.target_radius < self.far_radius <= 2:
            raise ValueError('invalid ground scan radii')
        if not 0 < self.min_width_m < self.max_width_m <= 2:
            raise ValueError('invalid lane width interval')
        if not 3 <= self.min_pairs <= self.scan_count <= 40:
            raise ValueError('require 3 <= min_pairs <= scan_count <= 40')
        for low, high in ((self.yellow_lower, self.yellow_upper),
                          (self.white_lower, self.white_upper)):
            if len(low) != 3 or len(high) != 3 or not all(
                0 <= lo <= hi <= limit for lo, hi, limit in zip(low, high, (179, 255, 255))
            ):
                raise ValueError('HSV bounds must be ordered H=0..179, S,V=0..255')


@dataclass
class Detection:
    error: float | None
    center: tuple | None
    pairs: list
    yellow: np.ndarray
    white: np.ndarray
    roi: tuple


def projection(width, height, radius, angle, cfg):
    """Project ground points relative to the RGB camera into pixel coordinates."""
    f = width / (2.0 * np.tan(cfg.camera_hfov / 2.0))
    forward = radius * np.cos(angle)
    lateral = radius * np.sin(angle)
    c, s = np.cos(cfg.camera_pitch), np.sin(cfg.camera_pitch)
    depth = c * forward + s * cfg.camera_height
    vertical = c * cfg.camera_height - s * forward
    return ((width - 1) / 2 + f * lateral / depth,
            (height - 1) / 2 + f * vertical / depth)


def spans(mask, x, y, visible, angles):
    """Find compact intersections between a ground arc and an HSV mask."""
    hit = np.zeros(angles.size, dtype=bool)
    hit[visible] = mask[y[visible], x[visible]] > 0
    edges = np.flatnonzero(np.diff(np.r_[False, hit, False]))
    return [(lo + hi - 1) // 2 for lo, hi in zip(edges[::2], edges[1::2])
            if 2 <= hi - lo and angles[hi - 1] - angles[lo] < 0.40]


def detect(image, cfg):
    """Pair edges on arcs in the ground plane and project the target back to the image."""
    height, width = image.shape[:2]
    top, bottom = int(cfg.roi_top * height), int(cfg.roi_bottom * height)
    yellow = np.zeros((height, width), np.uint8)
    white = np.zeros_like(yellow)
    if bottom - top < 5 or width < 16:
        return Detection(None, None, [], yellow, white, (top, bottom))
    hsv = cv2.cvtColor(image[top:bottom], cv2.COLOR_BGR2HSV)
    yellow[top:bottom] = cv2.inRange(hsv, cfg.yellow_lower, cfg.yellow_upper)
    white[top:bottom] = cv2.inRange(hsv, cfg.white_lower, cfg.white_upper)
    left, right = (yellow, white) if cfg.yellow_left else (white, yellow)
    angles = np.linspace(-1.12, 1.12, 481)
    groups = []
    trace = []
    for radius in np.linspace(cfg.near_radius, cfg.far_radius, cfg.scan_count):
        px, py = projection(width, height, radius, angles, cfg)
        x, y = np.rint(px).astype(int), np.rint(py).astype(int)
        visible = (0 <= x) & (x < width) & (top <= y) & (y < bottom)
        ls = spans(left, x, y, visible, angles)
        rs = spans(right, x, y, visible, angles)
        connected = trace and radius - trace[-1][0] <= 0.08
        anchor = trace[-1][1] if connected else 0.0
        choices = []
        for l in ls:
            for r in rs:
                if l >= r:
                    continue
                gap = 2 * radius * np.sin((angles[r] - angles[l]) / 2)
                center = (angles[l] + angles[r]) / 2
                if cfg.min_width_m <= gap <= cfg.max_width_m:
                    jump = abs(center - anchor)
                    if not connected or jump <= 0.28:
                        choices.append((jump + abs(gap - 0.31), l, r, center))
        if not choices:
            continue
        _, l, r, center = min(choices)
        if not connected:
            trace = []
            groups.append(trace)
        trace.append((radius, center, (int(x[l]), int(y[l]), int(x[r]), int(y[r]))))
    trace = max(groups, key=len, default=[])
    pairs = [entry[2] for entry in trace]
    if len(pairs) < cfg.min_pairs:
        return Detection(None, None, pairs, yellow, white, (top, bottom))

    radii, centers, _ = zip(*trace)
    requested = min(max(cfg.target_radius, radii[0]), radii[-1] + 0.08)
    angle = np.interp(requested, radii, centers)
    if requested > radii[-1]:
        # Use the last supported arc slope, for at most 8 cm beyond observed paint.
        slope = np.polyfit(radii[-4:], centers[-4:], 1)[0]
        angle += np.clip(slope * (requested - radii[-1]), -0.24, 0.24)
    angle = np.clip(angle, -0.65, 0.65)
    x, y = projection(width, height, requested, angle, cfg)
    target = cfg.target_x * (width - 1)
    return Detection(float((target - x) / (width / 2)), (float(x), int(y)), pairs,
                     yellow, white, (top, bottom))


def debug_panel(image, result, config, status):
    """Show the raw image, the tracked lane, and both masks at half size."""
    view = image.copy()
    height, width = view.shape[:2]
    for y in result.roi:
        cv2.line(view, (0, y), (width - 1, y), (220, 170, 0), 2)
    left_color, right_color = ((0, 215, 255), (255, 255, 255))
    if not config.yellow_left:
        left_color, right_color = right_color, left_color
    for lx, ly, rx, ry in result.pairs:
        cv2.circle(view, (lx, ly), 6, left_color, -1)
        cv2.circle(view, (rx, ry), 6, right_color, -1)
    if result.center is not None:
        x, y = result.center
        target = round(config.target_x * (width - 1))
        cv2.circle(view, (round(x), y), 9, (0, 255, 0), -1)
        cv2.drawMarker(view, (target, y), (255, 140, 0), cv2.MARKER_CROSS, 28, 3)
    panels = [(image.copy(), 'Raw camera'), (view, status),
              (cv2.cvtColor(result.yellow, cv2.COLOR_GRAY2BGR), 'Yellow mask'),
              (cv2.cvtColor(result.white, cv2.COLOR_GRAY2BGR), 'White mask')]
    tiles = []
    for tile, title in panels:
        cv2.rectangle(tile, (0, 0), (width - 1, 37), (25, 25, 25), -1)
        cv2.putText(tile, title, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                    (255, 255, 255), 2, cv2.LINE_AA)
        tiles.append(cv2.resize(tile, (max(1, width // 2), max(1, height // 2))))
    return np.vstack((np.hstack(tiles[:2]), np.hstack(tiles[2:])))
