#!/usr/bin/env python3
"""How close is the side decision? Cost of the best path on each side of an
obstacle, per grid, so a "prefer right when similar" rule can be sized from
measurement instead of guessed.

For every recorded path whose grid has the obstacle confirmed (same rule as
analyse_planner.py: a track within 2 m of the true near face), two exact
searches are run on that grid:
  C_R  best path forced to pass right of the obstacle centre: in the obstacle
       centre column, every row whose centre is at or left of it is closed
  C_L  the mirror image
and the relative margin  m = (C_L - C_R) / min(C_L, C_R)  is recorded.
m > 0 means right is cheaper. The rule "choose right unless left is cheaper by
more than e" removes a flip exactly when the margin stays above -e.

Also splits the grids into moving and stopped (ground truth speed above or
below 0.1 m/s), and cross-checks that the side the node's path actually took
agrees with the sign of m.

Test apparatus: reads ground truth.

Usage (from ~/ros2_ws, next to analyse_planner.py):
  python3 src/corridor_experiments/scripts/analyse_side_margin.py bags/NAME X,Y
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyse_planner import load, cell_index, exact_search, OBS_SIZE, MATCH  # noqa: E402

MOVING = 0.1      # m/s
EPS = (0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.10)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    uri = sys.argv[1]
    wx, wy = (float(v) for v in sys.argv[2].split(","))
    grids, paths, poses, tracks, _ = load(uri)
    pt, px, py, pyaw = poses
    dup = int(np.sum(np.diff(pt) == 0.0))
    print(f"poses: {len(pt)}, repeated stamps: {dup}, "
          f"world x from {px.min():.2f} to {px.max():.2f}")

    def speed_at(t):
        # Over a 0.2 s window, so repeated or bunched pose stamps cannot make
        # a moving vehicle look stopped (or the reverse).
        # Stamps from load() are integer nanoseconds.
        a = int(np.clip(np.searchsorted(pt, t - 1e8), 0, len(pt) - 1))
        b = int(np.clip(np.searchsorted(pt, t + 1e8), 0, len(pt) - 1))
        if pt[b] - pt[a] <= 0.0:
            return float("nan")
        return math.hypot(px[b] - px[a], py[b] - py[a]) / ((pt[b] - pt[a]) * 1e-9)

    rows = []   # stamp, margin, moving, taken side, C_R, C_L
    for stamp, pts in paths:
        g = grids.get(stamp)
        if g is None or not pts:
            continue
        i = int(np.clip(np.searchsorted(pt, stamp), 1, len(pt) - 1))
        if abs(pt[i - 1] - stamp) < abs(pt[i] - stamp):
            i -= 1
        vx, vy, yaw = px[i], py[i], pyaw[i]
        c, s = math.cos(yaw), math.sin(yaw)

        def base(x, y):
            dx, dy = x - vx, y - vy
            return c * dx + s * dy, -s * dx + c * dy

        nfx, nfy = base(wx - 0.5 * OBS_SIZE, wy)
        if not any(math.hypot(tx - nfx, ty - nfy) < MATCH for tx, ty in tracks.get(stamp, [])):
            continue
        cells, ox, oy, res = g
        ny, nx = cells.shape
        cx, cy = base(wx, wy)
        col = cell_index(cx, cy, ox, oy, res)[0]
        if not (0 < col < nx):
            continue
        si, sj = cell_index(0.0, 0.0, ox, oy, res)
        yrow = oy + (np.arange(ny) + 0.5) * res

        right = cells.copy()
        right[yrow >= cy, col] = 100
        left = cells.copy()
        left[yrow <= cy, col] = 100
        cr = exact_search(right, si, sj, res)
        cl = exact_search(left, si, sj, res)
        if not (np.isfinite(cr) and np.isfinite(cl)):
            continue
        m = (cl - cr) / min(cl, cr)

        k = col - si
        taken = None
        if 0 <= k < len(pts):
            taken = "R" if pts[k][1] < cy else "L"
        rows.append((stamp, m, speed_at(stamp) > MOVING, taken, cr, cl))

    if not rows:
        print("no scored grids")
        return
    m = np.array([r[1] for r in rows])
    mov = np.array([r[2] for r in rows])
    taken = [r[3] for r in rows]
    agree = sum(1 for r in rows if r[3] is not None and (r[3] == "R") == (r[1] > 0))
    exact_tie = int(np.sum(m == 0.0))
    cr = np.array([r[4] for r in rows])

    print(f"bag: {uri}, obstacle ({wx}, {wy}), scored grids {len(rows)} "
          f"({int(mov.sum())} moving, {int((~mov).sum())} stopped)")
    print(f"best path cost, right side: median {np.median(cr):.3f}, range {cr.min():.3f} to {cr.max():.3f}")
    print(f"side taken agrees with sign of margin: {agree}/{sum(t is not None for t in taken)}"
          f" (exact ties, margin 0: {exact_tie}; a tie counts as left here, so it means nothing there)")
    for name, want in (("moving", True), ("stopped", False)):
        tied = [r[3] for r in rows if r[1] == 0.0 and r[2] == want]
        tf = sum(1 for a, b in zip(tied, tied[1:]) if a != b)
        print(f"exact ties while {name:7s}: {len(tied):4d} grids, node took R {tied.count('R')}, "
              f"L {tied.count('L')}, flips {tf}")
    for name, sel in (("all", np.ones_like(mov)), ("moving", mov), ("stopped", ~mov)):
        if not sel.any():
            print(f"{name:8s}: no grids")
            continue
        v = m[sel] * 100
        sides = np.where(m[sel] > 0, "R", "L")
        flips = int(np.sum(sides[1:] != sides[:-1]))
        print(f"{name:8s}: n {int(sel.sum()):4d} | margin %  median {np.median(v):+.3f}  "
              f"p5 {np.percentile(v, 5):+.3f}  p95 {np.percentile(v, 95):+.3f}  "
              f"min {v.min():+.3f}  max {v.max():+.3f} | flips by sign {flips}")
    print("rule 'right unless left cheaper by more than e': grids still choosing left, flips left")
    for e in EPS:
        sides = np.where(m > -e, "R", "L")
        print(f"  e {e * 100:5.1f} %: left on {int(np.sum(sides == 'L')):4d}, "
              f"flips {int(np.sum(sides[1:] != sides[:-1])):4d}")


if __name__ == "__main__":
    main()
