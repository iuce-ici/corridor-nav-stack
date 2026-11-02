#!/usr/bin/env python3
"""Reported obstacle extent against range, from a recorded bag.

Reads /obstacle_tracks, sorts by header stamp (bag order is recorder
receive order, not publish order), and bins every confirmed track's
reported length and width by its tracked near face x in base_link.

Purpose: decide how the 3d-1 cost grid treats the parts of an obstacle
the LiDAR cannot see. Two questions:
  1. Does the reported length recover the true depth, or only the
     depth that was visible? (side face in the sensor plane in
     single_left, visible in slalom)
  2. Is the reported width short of the truth, and does the shortfall
     grow with range like one azimuth step (range x 0.5 deg)?

Usage:
  python3 analyse_track_extent.py bags/NAME
"""
import sys
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPIC = "/obstacle_tracks"
TRUE_SIZE = 1.2          # OBSTACLE_SIZE in generate_corridor.py, x and y
AZ_STEP = math.radians(0.5)
LIDAR_X = 4.2            # sensor position in base_link; beams spread from here
# Binned by range from the SENSOR, not base_link x. Max sensor range for a
# near face inside max_x 30 is about 25.8 m.
BINS = [(0, 5), (5, 10), (10, 15), (15, 20), (20, 26)]


def open_reader(uri):
    last = None
    for sid in ("", "mcap", "sqlite3"):
        try:
            r = rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid),
                   rosbag2_py.ConverterOptions("", ""))
            return r
        except Exception as e:  # try the next storage plugin
            last = e
    raise RuntimeError(f"could not open {uri}: {last}")


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    reader = open_reader(sys.argv[1])
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if TOPIC not in types:
        print(f"{TOPIC} not in bag. Topics: {sorted(types)}")
        sys.exit(1)
    msg_type = get_message(types[TOPIC])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))

    msgs = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, msg_type)
        msgs.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, m))

    stamps = [s for s, _ in msgs]
    backward = sum(1 for a, b in zip(stamps, stamps[1:]) if b < a)
    msgs.sort(key=lambda p: p[0])

    rows = []  # (id, x, y, length, width)
    for _, m in msgs:
        for t in m.tracks:
            rows.append((t.id, t.x, t.y, t.length, t.width))

    print(f"bag: {sys.argv[1]}")
    print(f"messages: {len(msgs)}, backward intervals in bag order: {backward}")
    print(f"track rows: {len(rows)}, distinct ids: {len({r[0] for r in rows})}")
    print(f"true size: {TRUE_SIZE:.2f} m deep, {TRUE_SIZE:.2f} m wide\n")

    if not rows:
        print("no confirmed tracks in this bag")
        return

    arr = np.array([r[1:] for r in rows])
    x, y, length, width = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]

    # Per row: range from the sensor to the near face centre, the shortfall
    # in width, and that shortfall as a fraction of two azimuth steps at the
    # row's own range (one step lost per side is the beam spacing model).
    r = np.hypot(x - LIDAR_X, y)
    short = TRUE_SIZE - width
    two_steps = 2.0 * r * AZ_STEP
    ratio = short / two_steps

    print("sensor range | n   | length median  max | width median  min | "
          "short median  max | ratio median  max | rows over 1")
    for lo, hi in BINS:
        sel = (r >= lo) & (r < hi)
        n = int(sel.sum())
        if n == 0:
            print(f"{lo:5.1f} to {hi:4.1f} | {n:3d} | no rows")
            continue
        print(f"{lo:5.1f} to {hi:4.1f} | {n:3d} | "
              f"{np.median(length[sel]):6.3f} {length[sel].max():6.3f} | "
              f"{np.median(width[sel]):6.3f} {width[sel].min():6.3f} | "
              f"{np.median(short[sel]):+.3f} {short[sel].max():+.3f} | "
              f"{np.median(ratio[sel]):5.2f} {ratio[sel].max():5.2f} | "
              f"{int((ratio[sel] > 1.0).sum())}")

    worst = int(np.argmax(ratio))
    print(f"\nworst row overall: sensor range {r[worst]:.2f} m, width "
          f"{width[worst]:.3f} m, short {short[worst]:.3f} m, "
          f"ratio {ratio[worst]:.2f}, track y {y[worst]:+.3f} m")


if __name__ == "__main__":
    main()
