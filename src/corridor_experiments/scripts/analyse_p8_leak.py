#!/usr/bin/env python3
"""Why was a path published on a scan whose corridor geometry was invalid?

For every /planned_path whose stamp carries an invalid /corridor_geometry,
prints the neighbouring geometry messages (stamp, valid, age), whether a
/obstacle_tracks and a /cost_grid exist at that stamp, and the order in which
the recorder received the geometry, the tracks and the grid for that stamp.
The recorder's receive order is only a proxy for what the cost grid node saw,
but if the tracks (which the cost grid ticks on) were received before that
stamp's geometry, the cost grid most likely built the grid from the previous,
still fresh, valid geometry.

Usage (from ~/ros2_ws):
  python3 src/corridor_experiments/scripts/analyse_p8_leak.py bags/NAME
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ["/corridor_geometry", "/obstacle_tracks", "/cost_grid", "/planned_path"]


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
    cls = {t: get_message(types[t]) for t in TOPICS}
    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))

    geom = {}              # stamp -> valid
    recv = {}              # (topic, stamp) -> receive time ns
    paths = []
    while reader.has_next():
        topic, data, t_recv = reader.read_next()
        m = deserialize_message(data, cls[topic])
        s = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        recv[(topic, s)] = t_recv
        if topic == "/corridor_geometry":
            geom[s] = m.valid
        elif topic == "/planned_path":
            paths.append((s, len(m.poses)))

    gst = sorted(geom)
    leaks = [(s, n) for s, n in paths if s in geom and not geom[s]]
    print(f"bag: {sys.argv[1]}, paths {len(paths)}, geometry {len(gst)} "
          f"({sum(1 for v in geom.values() if not v)} invalid), paths on invalid geometry {len(leaks)}")
    for s, n in leaks:
        k = gst.index(s)
        print(f"\nleaked path at stamp {s / 1e9:.3f} s, {n} poses")
        for j in range(max(0, k - 3), min(len(gst), k + 2)):
            g = gst[j]
            print(f"  geometry {g / 1e9:.3f} s  valid {geom[g]!s:5s}  "
                  f"{(g - s) / 1e6:+7.1f} ms from the leak")
        have = {t: (t, s) in recv for t in TOPICS}
        print(f"  at this stamp: tracks {have['/obstacle_tracks']}, grid {have['/cost_grid']}")
        order = sorted((recv[(t, s)], t) for t in TOPICS if (t, s) in recv)
        t0 = order[0][0]
        print("  recorder receive order: " + ", ".join(
            f"{t} +{(r - t0) / 1e6:.2f} ms" for r, t in order))


if __name__ == "__main__":
    main()
