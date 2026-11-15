#!/usr/bin/env python3
"""Pipeline latency per scan, and the P7 limit it implies.

Every message derived from one LiDAR scan carries that scan's stamp. For each
stamp, this takes the time at which the RECORDER received each stage's
message, and reports each stage relative to the scan itself:
  scan -> geometry -> obstacles -> tracks -> grid -> path
These are stage to stage differences on one wall clock, not absolute
latencies: the recorder's own transport is inside every number, and most of
all in the scan, a large point cloud. That uncertainty is reported as the
spread of the scan to geometry stage, which contains it.

P7 limit, agreed 15 Nov 2026 (gate: scan to path p99 at most 50 ms):
  P7 = (50 ms - U99 - 5 ms reserved for the 3e path follower) / 2
where U99 is the p99 of scan to grid published.

Usage (from ~/ros2_ws):
  python3 src/corridor_experiments/scripts/analyse_latency.py bags/NAME
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

STAGES = [("scan", "/scan/points"), ("geometry", "/corridor_geometry"),
          ("obstacles", "/obstacles"), ("tracks", "/obstacle_tracks"),
          ("grid", "/cost_grid"), ("path", "/planned_path")]
GATE_MS = 50.0
RESERVE_MS = 5.0


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    reader = None
    for sid in ("", "mcap", "sqlite3"):
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id=sid),
                        rosbag2_py.ConverterOptions("", ""))
            break
        except Exception:
            reader = None
    if reader is None:
        sys.exit(f"could not open {sys.argv[1]}")
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = [t for _, t in STAGES]
    missing = [t for t in topics if t not in types]
    if missing:
        sys.exit(f"missing topics {missing}; record them too")
    cls = {t: get_message(types[t]) for t in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    recv = {t: {} for t in topics}        # topic -> stamp -> receive ns
    while reader.has_next():
        topic, data, t_recv = reader.read_next()
        m = deserialize_message(data, cls[topic])
        s = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        recv[topic].setdefault(s, t_recv)   # first receipt only

    scans = recv["/scan/points"]
    print(f"bag: {sys.argv[1]}")
    print("messages per stage, and how many share a stamp with a recorded scan:")
    for name, t in STAGES:
        n = len(recv[t])
        k = sum(1 for s in recv[t] if s in scans)
        print(f"  {name:9s} {n:5d}   matched to a scan {k:5d}")

    print("\nreceive time after the scan, ms:   n     median     p99      max")
    p99 = {}
    for name, t in STAGES[1:]:
        d = np.array([(r - scans[s]) / 1e6 for s, r in recv[t].items() if s in scans])
        if d.size == 0:
            print(f"  {name:9s} no matches")
            continue
        p99[name] = float(np.percentile(d, 99))
        print(f"  {name:9s}              {d.size:5d}  {np.median(d):8.2f} {p99[name]:8.2f} {d.max():8.2f}"
              + ("   (negative values: recorder got this before the scan)" if d.min() < 0 else ""))

    if "grid" in p99 and "path" in p99:
        u99 = p99["grid"]
        limit = (GATE_MS - u99 - RESERVE_MS) / 2.0
        print(f"\nU99 (scan to grid, p99) {u99:.2f} ms")
        print(f"gate: scan to path p99 {p99['path']:.2f} ms against {GATE_MS:.0f} ms -> "
              f"{'PASS' if p99['path'] <= GATE_MS else 'FAIL'}")
        print(f"P7 limit = ({GATE_MS:.0f} - {u99:.2f} - {RESERVE_MS:.0f}) / 2 = {limit:.2f} ms "
              "(compare with the planner's own 'plan time over N grids' p99)")


if __name__ == "__main__":
    main()
