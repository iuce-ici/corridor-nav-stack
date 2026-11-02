#!/usr/bin/env python3
"""Where are the confirmed tracks in a run that should have none?

Diagnostic for an obstacle-free corridor. Prints, from /obstacle_tracks:
  - every confirmed track row's x and y in base_link, as percentiles
  - a histogram of y in 0.25 m bins, so a mirrored wall or a fixed artefact
    shows up as one dominant bin
  - per track id: rows, x range, y range
and from /corridor_geometry, the values the extraction node was fed:
  lateral_offset, heading_error, corridor_width, valid fraction.

All topics sorted by header stamp.

Usage:
  python3 analyse_phantoms.py bags/NAME
"""
import sys
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TRACKS = "/obstacle_tracks"
GEOM = "/corridor_geometry"


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


def pct(a):
    q = np.percentile(a, [0, 5, 50, 95, 100])
    return "min {:+.3f}  p5 {:+.3f}  median {:+.3f}  p95 {:+.3f}  max {:+.3f}".format(*q)


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    data = read_sorted(sys.argv[1], [TRACKS, GEOM])

    g = data[GEOM]
    off = np.array([m.lateral_offset for _, m in g])
    hdg = np.array([m.heading_error for _, m in g])
    wid = np.array([m.corridor_width for _, m in g])
    val = np.array([m.valid for _, m in g])
    print(f"corridor_geometry: {len(g)} messages, valid fraction {val.mean():.3f}")
    print(f"  lateral_offset  {pct(off)}")
    print(f"  heading_error   {pct(hdg)}")
    print(f"  corridor_width  {pct(wid)}\n")

    rows = []
    by_id = defaultdict(list)
    for s, m in data[TRACKS]:
        for t in m.tracks:
            rows.append((t.x, t.y))
            by_id[t.id].append((s, t.x, t.y))
    if not rows:
        print("no confirmed tracks")
        return
    a = np.array(rows)
    print(f"confirmed track rows: {len(rows)}, distinct ids: {len(by_id)}")
    print(f"  x  {pct(a[:, 0])}")
    print(f"  y  {pct(a[:, 1])}\n")

    print("y histogram, 0.25 m bins (rows):")
    edges = np.arange(np.floor(a[:, 1].min() * 4) / 4, a[:, 1].max() + 0.25, 0.25)
    counts, edges = np.histogram(a[:, 1], bins=edges)
    for lo, n in zip(edges[:-1], counts):
        if n:
            print(f"  {lo:+6.2f} to {lo + 0.25:+6.2f}: {n}")

    print("\nper id (first 15 by birth):")
    print("  id    rows  t0       x range          y range")
    t_first = min(v[0][0] for v in by_id.values())
    for tid, v in sorted(by_id.items(), key=lambda kv: kv[1][0][0])[:15]:
        xs = [p[1] for p in v]
        ys = [p[2] for p in v]
        print(f"  {tid:<5d} {len(v):<5d} {v[0][0] - t_first:6.2f}  "
              f"{min(xs):6.2f} to {max(xs):6.2f}  {min(ys):+6.2f} to {max(ys):+6.2f}")


if __name__ == "__main__":
    main()
