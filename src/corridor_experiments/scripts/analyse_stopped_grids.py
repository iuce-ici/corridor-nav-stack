#!/usr/bin/env python3
"""While the vehicle stands still, does anything the planner sees change?

For every pair of consecutive grids recorded while the vehicle is stopped
(ground truth speed under 0.1 m/s over a 0.2 s window), compares the two cost
grids cell by cell and the two planned paths pose by pose, and sorts the
pairs into four classes:
  grid same,    path same      expected if nothing moves
  grid same,    path differs   the planner is not deterministic on one input
  grid differs, path same
  grid differs, path differs
For pairs whose grids differ, reports how many cells changed, which kind
(a lateral cost value, or a cell crossing into or out of 99/100), and the
spread of the confirmed track's x and y over the stopped period. That says
whether the stopped flips come from the sensor, the tracker, the cost grid,
or the planner itself.

Test apparatus: reads ground truth.

Usage (from ~/ros2_ws, next to analyse_planner.py):
  python3 src/corridor_experiments/scripts/analyse_stopped_grids.py bags/NAME
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyse_planner import load  # noqa: E402

MOVING = 0.1      # m/s


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    grids, paths, poses, tracks, _ = load(sys.argv[1])
    pt, px, py, _ = poses

    def speed_at(t):
        # Stamps from load() are integer nanoseconds.
        a = int(np.clip(np.searchsorted(pt, t - 1e8), 0, len(pt) - 1))
        b = int(np.clip(np.searchsorted(pt, t + 1e8), 0, len(pt) - 1))
        if pt[b] - pt[a] <= 0.0:
            return float("nan")
        return math.hypot(px[b] - px[a], py[b] - py[a]) / ((pt[b] - pt[a]) * 1e-9)

    stopped = [(s, p) for s, p in paths if s in grids and speed_at(s) < MOVING]
    print(f"bag: {sys.argv[1]}, paths {len(paths)}, stopped paths with a grid {len(stopped)}")
    if len(stopped) < 2:
        return

    classes = {(False, False): 0, (False, True): 0, (True, False): 0, (True, True): 0}
    changed, cost_only, crossing = [], 0, 0
    for (s0, p0), (s1, p1) in zip(stopped, stopped[1:]):
        g0, g1 = grids[s0][0], grids[s1][0]
        diff = g0 != g1
        gd = bool(diff.any())
        pd = len(p0) != len(p1) or any(
            abs(a[0] - b[0]) > 1e-9 or abs(a[1] - b[1]) > 1e-9 for a, b in zip(p0, p1))
        classes[(gd, pd)] += 1
        if gd:
            changed.append(int(diff.sum()))
            before, after = g0[diff], g1[diff]
            cross = ((before >= 99) != (after >= 99)) | ((before == 100) != (after == 100))
            crossing += int(cross.any())
            cost_only += int(not cross.any())

    print("consecutive stopped pairs:")
    print(f"  grid same,    path same:    {classes[(False, False)]}")
    print(f"  grid same,    path differs: {classes[(False, True)]}   <- above 0 means the planner"
          " is not deterministic")
    print(f"  grid differs, path same:    {classes[(True, False)]}")
    print(f"  grid differs, path differs: {classes[(True, True)]}")
    if changed:
        c = np.array(changed)
        print(f"  cells changed when the grid differs: median {np.median(c):.0f}, max {c.max()};"
              f" pairs where only lateral cost values changed {cost_only},"
              f" pairs where a cell crossed into or out of 99/100 {crossing}")

    tx, ty = [], []
    for s, _ in stopped:
        for x, y in tracks.get(s, []):
            if 0.0 < x < 30.0:
                tx.append(x)
                ty.append(y)
    if tx:
        tx, ty = np.array(tx), np.array(ty)
        print(f"confirmed tracks while stopped: {len(tx)} | near face x {tx.min():.4f} to {tx.max():.4f}"
              f" (spread {1e3 * (tx.max() - tx.min()):.1f} mm) | y {ty.min():+.4f} to {ty.max():+.4f}"
              f" (spread {1e3 * (ty.max() - ty.min()):.1f} mm)")
    else:
        print("confirmed tracks while stopped: none")


if __name__ == "__main__":
    main()
