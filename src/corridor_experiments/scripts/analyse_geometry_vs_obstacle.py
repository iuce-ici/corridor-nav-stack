#!/usr/bin/env python3
"""Corridor geometry error against how far ahead the obstacle is.

Truth for a straight corridor: lateral_offset = vehicle world y, heading_error
= vehicle yaw (corridor axis is world x), corridor_width = 8.0. The obstacle's
near face in base_link comes from the vehicle's ground truth x and the
obstacle's world position given on the command line.

Bins every /corridor_geometry message by that near face distance and prints,
per bin: heading error, offset error, width error, and for each wall its
point count and fit residual. If obstacle returns get into a wall fit, that
side's residual rises as the obstacle closes, and the heading error with it.

Test apparatus: reads ground truth.

Usage:
  python3 analyse_geometry_vs_obstacle.py bags/NAME OBS_X OBS_Y
"""
import sys
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

GEOM = "/corridor_geometry"
POSE = "/model/vehicle/pose"
SIZE = 1.2
WIDTH = 8.0
BINS = [(30, 40), (25, 30), (20, 25), (15, 20), (10, 15), (5, 10), (0, 5)]


def open_reader(uri):
    last = None
    for sid in ("", "mcap", "sqlite3"):
        try:
            r = rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid),
                   rosbag2_py.ConverterOptions("", ""))
            return r
        except Exception as e:
            last = e
    raise RuntimeError(f"could not open {uri}: {last}")


def read_sorted(uri, topics):
    reader = open_reader(uri)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        print(f"missing topics {missing}. Present: {sorted(types)}")
        sys.exit(1)
    cls = {t: get_message(types[t]) for t in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = deserialize_message(data, cls[topic])
        out[topic].append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m))
    for t in topics:
        out[t].sort(key=lambda p: p[0])
    return out


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)
    uri, ox, oy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    data = read_sorted(uri, [GEOM, POSE])
    poses = data[POSE]
    pt = np.array([s for s, _ in poses])

    rows = []   # near, d_heading, d_offset, d_width, lp, rp, lres, rres
    for s, g in data[GEOM]:
        if not g.valid:
            continue
        i = int(np.clip(np.searchsorted(pt, s), 1, len(pt) - 1))
        if abs(pt[i - 1] - s) < abs(pt[i] - s):
            i -= 1
        m = poses[i][1].pose
        vx, vy, yaw = m.position.x, m.position.y, yaw_of(m.orientation)
        near = (ox - 0.5 * SIZE) - vx
        rows.append((near, g.heading_error - yaw, g.lateral_offset - vy,
                     g.corridor_width - WIDTH, g.left_points, g.right_points,
                     g.left_residual, g.right_residual))
    if not rows:
        print("no valid geometry")
        return
    a = np.array(rows)

    print(f"bag: {uri}, obstacle near face at world x {ox - 0.5 * SIZE:.2f}, "
          f"valid geometry messages: {len(a)}")
    print("near face ahead | n   | heading err mrad  med    max | offset err mm  med   max |"
          " width err mm  med   max | left pts  res mm | right pts  res mm")
    for lo, hi in BINS:
        sel = (a[:, 0] >= lo) & (a[:, 0] < hi)
        n = int(sel.sum())
        if n == 0:
            print(f"  {lo:2d} to {hi:2d} m    | {n:3d} | no messages")
            continue
        h = a[sel, 1] * 1e3
        o = a[sel, 2] * 1e3
        w = a[sel, 3] * 1e3
        imax = lambda v: v[np.argmax(np.abs(v))]
        print(f"  {lo:2d} to {hi:2d} m    | {n:3d} | {np.median(h):+7.2f} {imax(h):+7.2f}      |"
              f" {np.median(o):+6.1f} {imax(o):+6.1f}     | {np.median(w):+6.1f} {imax(w):+6.1f}    |"
              f" {np.median(a[sel, 4]):6.0f} {np.median(a[sel, 6]) * 1e3:6.1f} |"
              f" {np.median(a[sel, 5]):6.0f} {np.median(a[sel, 7]) * 1e3:6.1f}")


if __name__ == "__main__":
    main()
