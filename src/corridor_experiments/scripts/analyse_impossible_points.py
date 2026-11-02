#!/usr/bin/env python3
"""Raw LiDAR returns from places the sensor cannot see: beyond the walls.

In a straight corridor nothing exists outside the two inner wall faces, so a
return with base_link y beyond either wall is impossible for a real sensor.
Extraction drops such points silently as wall points, so they never reach a
track; only the raw cloud shows them.

y needs no transform: the LiDAR sits at base_link y 0 and is pitched about
its own y axis, which leaves y unchanged. So sensor y equals base_link y.

The walls in base_link come from the vehicle's lateral position, read from
/corridor_geometry (lateral_offset, corridor_width), sorted by stamp and
paired to each cloud by nearest stamp.

For every scan, counts returns more than MARGIN beyond each wall, then
reports where they sit: sensor x, y, z, azimuth atan2(y, x), and the ring
field if the cloud has one.

Usage:
  python3 analyse_impossible_points.py bags/NAME
"""
import sys
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

CLOUD = "/scan/points"
GEOM = "/corridor_geometry"
MARGIN = 0.30          # metres beyond a wall face; well above 3 cm range noise


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


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    data = read_sorted(sys.argv[1], [CLOUD, GEOM])
    geo = data[GEOM]
    gt = np.array([s for s, _ in geo])

    clouds = data[CLOUD]
    fields = [f.name for f in clouds[0][1].fields] if clouds else []
    has_ring = "ring" in fields
    print(f"bag: {sys.argv[1]}, clouds: {len(clouds)}, fields: {fields}")
    print(f"cloud frame: {clouds[0][1].header.frame_id if clouds else 'none'}\n")

    scans_with = {"right": 0, "left": 0}
    pts = {"right": [], "left": []}
    total_finite = 0

    for s, c in clouds:
        i = int(np.clip(np.searchsorted(gt, s), 1, len(gt) - 1))
        if abs(gt[i - 1] - s) < abs(gt[i] - s):
            i -= 1
        g = geo[i][1]
        if not g.valid:
            continue
        # Walls in base_link, heading taken as zero (measured under 0.001 rad).
        centre = -g.lateral_offset
        left_wall = centre + 0.5 * g.corridor_width
        right_wall = centre - 0.5 * g.corridor_width

        names = ["x", "y", "z"] + (["ring"] if has_ring else [])
        raw = point_cloud2.read_points(c, field_names=names, skip_nans=True)
        arr = np.array([tuple(float(v) for v in p) for p in raw])
        if arr.size == 0:
            continue
        arr = arr[np.all(np.isfinite(arr[:, :3]), axis=1)]
        total_finite += len(arr)
        y = arr[:, 1]
        for side, mask in (("right", y < right_wall - MARGIN),
                           ("left", y > left_wall + MARGIN)):
            if mask.any():
                scans_with[side] += 1
                pts[side].append(arr[mask])

    print(f"finite returns examined: {total_finite}")
    for side in ("right", "left"):
        print(f"\nbeyond the {side.upper()} wall (more than {MARGIN} m): "
              f"{scans_with[side]} of {len(clouds)} scans")
        if not pts[side]:
            continue
        a = np.vstack(pts[side])
        az = np.degrees(np.arctan2(a[:, 1], a[:, 0]))
        print(f"  points: {len(a)}")
        for name, col in (("sensor x", a[:, 0]), ("y", a[:, 1]), ("sensor z", a[:, 2]),
                          ("azimuth deg", az)):
            q = np.percentile(col, [0, 50, 100])
            print(f"  {name:12s} min {q[0]:+8.3f}  median {q[1]:+8.3f}  max {q[2]:+8.3f}")
        if has_ring:
            rings, counts = np.unique(a[:, 3].astype(int), return_counts=True)
            print("  rings: " + ", ".join(f"{r}:{n}" for r, n in zip(rings, counts)))


if __name__ == "__main__":
    main()
