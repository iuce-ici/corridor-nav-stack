#!/usr/bin/env python3
"""Per edge width loss of raw detections against ground truth.

The track message carries a FILTERED centre and a RAW width, so edges built
from it mix two different things. This reads the raw /obstacles message
instead, where centre and width come from the same scan, and pairs each scan
with the vehicle's ground truth pose to place the true obstacle edges in
base_link.

For each edge separately: how far the measured edge sits INSIDE the true edge,
as a fraction of one azimuth step at that edge's own range from the sensor.
The beam spacing model says each edge loses at most one step, so every ratio
should be at most 1. A ratio above 1 on one edge only points at that edge.

Test apparatus: reads Gazebo ground truth, which the perception nodes never may.

Usage:
  python3 analyse_edge_loss.py bags/NAME OBS_X OBS_Y
  e.g. single_left: 50.0 0.6, slalom first obstacle: 40.0 0.8
"""
import sys
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OBS_TOPIC = "/obstacles"
POSE_TOPIC = "/model/vehicle/pose"
SIZE = 1.2               # OBSTACLE_SIZE, x and y
AZ_STEP = math.radians(0.5)
LIDAR_X = 4.2
MATCH = 2.0              # metres, raw detection to true centre
BINS = [(0, 5), (5, 10), (10, 15), (15, 20), (20, 26)]


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


def stamp_of(m):
    return m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec


def read_all(uri, topics):
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
        out[topic].append((stamp_of(m), m))
    for t in topics:
        out[t].sort(key=lambda p: p[0])
    return out


def pose_xy_yaw(m):
    p = m.pose
    if hasattr(p, "pose"):      # Odometry style
        p = p.pose
    q = p.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return p.position.x, p.position.y, yaw


def to_base(wx, wy, vx, vy, yaw):
    dx, dy = wx - vx, wy - vy
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)
    uri, ox, oy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    data = read_all(uri, [OBS_TOPIC, POSE_TOPIC])
    poses = data[POSE_TOPIC]
    pstamps = np.array([s for s, _ in poses])

    near_x = ox - 0.5 * SIZE
    right_w, left_w = oy - 0.5 * SIZE, oy + 0.5 * SIZE   # world y of the edges

    rows = []  # range_right, ratio_right, range_left, ratio_left
    worst_gap = 0.0
    for s, m in data[OBS_TOPIC]:
        if not m.obstacles:
            continue
        i = int(np.clip(np.searchsorted(pstamps, s), 1, len(pstamps) - 1))
        if abs(pstamps[i - 1] - s) < abs(pstamps[i] - s):
            i -= 1
        worst_gap = max(worst_gap, abs(pstamps[i] - s))
        vx, vy, yaw = pose_xy_yaw(poses[i][1])

        cx, cy = to_base(ox, oy, vx, vy, yaw)
        ob = min(m.obstacles, key=lambda o: math.hypot(o.x - cx, o.y - cy))
        if math.hypot(ob.x - cx, ob.y - cy) > MATCH:
            continue

        xr, yr = to_base(near_x, right_w, vx, vy, yaw)
        xl, yl = to_base(near_x, left_w, vx, vy, yaw)
        meas_r = ob.y - 0.5 * ob.width
        meas_l = ob.y + 0.5 * ob.width
        loss_r = meas_r - yr          # positive: measured edge inside the true one
        loss_l = yl - meas_l
        rng_r = math.hypot(xr - LIDAR_X, yr)
        rng_l = math.hypot(xl - LIDAR_X, yl)
        rows.append((rng_r, loss_r, loss_r / (rng_r * AZ_STEP),
                     rng_l, loss_l, loss_l / (rng_l * AZ_STEP)))

    print(f"bag: {uri}, obstacle centre ({ox}, {oy}), edges at world y "
          f"{right_w:+.2f} (right) and {left_w:+.2f} (left)")
    print(f"matched scans: {len(rows)}, worst pose pairing gap {worst_gap * 1e3:.1f} ms\n")
    if not rows:
        return
    a = np.array(rows)

    for name, rc, lc, qc in (("RIGHT edge", 0, 1, 2), ("LEFT edge", 3, 4, 5)):
        print(name)
        print("  edge range | n   | loss median   max  | ratio median  max | rows over 1")
        for lo, hi in BINS:
            sel = (a[:, rc] >= lo) & (a[:, rc] < hi)
            n = int(sel.sum())
            if n == 0:
                print(f"  {lo:4.1f} to {hi:4.1f} | {n:3d} | no rows")
                continue
            print(f"  {lo:4.1f} to {hi:4.1f} | {n:3d} | "
                  f"{np.median(a[sel, lc]):+.3f} {a[sel, lc].max():+.3f} | "
                  f"{np.median(a[sel, qc]):5.2f} {a[sel, qc].max():5.2f} | "
                  f"{int((a[sel, qc] > 1.0).sum())}")
        print()


if __name__ == "__main__":
    main()
