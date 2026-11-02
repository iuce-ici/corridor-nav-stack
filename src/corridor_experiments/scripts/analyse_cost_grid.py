#!/usr/bin/env python3
"""Phase 3d-1 criteria C1 to C3 for the cost grid, against ground truth.

Test apparatus: reads Gazebo ground truth, which the system under test never
may. Every topic is sorted by header stamp; bag order is not publish order.

Only grids recorded while the vehicle is MOVING are scored. run_controller
keeps the stack alive after the target, and a stationary tail would add
hundreds of identical scans that say nothing new.

Grid convention (cost_grid_node.cpp): base_link, cell centres at
origin + (index + 0.5) * resolution, so centres sit on multiples of it.
100 lethal, 99 blocked for new paths, 0 to 98 lateral cost.

Truth: corridor centreline at world y = 0, walls at world y = +-4.0.

C1 clear, offset spawn: in every column, every cell achieving the column's
   minimum cost lies within one cell (0.25 m) of the true centreline.
C2 clear, offset spawn: walls. Cells with |d| beyond WALL_LETHAL + C2_TOL
   are 100. Cells with |d| below WALL_LETHAL - C2_TOL are not 100.
   Cells with |d| between WALL_BLOCK + C2_TOL and WALL_LETHAL - C2_TOL are
   exactly 99. Cells with |d| below WALL_BLOCK - C2_TOL are at most 98.
   d is the true signed distance from the centreline.
   The node evaluates each cell exactly at its centre, so the only error left
   is the wall geometry estimate (sub-millimetre in Phase 2). C2_TOL is set
   by that, NOT by the cell size: a one cell tolerance on both sides leaves a
   half metre dead zone, and a self test showed it could not see a tight
   radius 0.3 m too small.
C3 with an obstacle: every cell within TIGHT of the true box is 100. Away
   from the walls, no 100 cell lies more than TIGHT + FRONT_SLACK in front of
   the true near face, nor more than TIGHT + PAD + SIDE_SLACK beside the true
   box. Behind the near face the shadow is lethal by design and is not
   bounded. Scored only while the true near face is between 0 and
   DETECT_RANGE in base_link.
   FRONT_SLACK is the 3b near face bound (the minimum over N noisy points is
   biased toward the vehicle). SIDE_SLACK covers the 3c track error with
   margin. Neither includes a cell: lethal cells are judged at their centres.

Usage:
  python3 analyse_cost_grid.py bags/NAME                 C1 and C2
  python3 analyse_cost_grid.py bags/NAME OBS_X OBS_Y     C3 (and C2 away from it)
"""
import sys
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

GRID_TOPIC = "/cost_grid"
POSE_TOPIC = "/model/vehicle/pose"

HALF_CORRIDOR = 4.0
TIGHT = 1.0
LOOSE = 1.3
PAD = 0.5
WALL_LETHAL = HALF_CORRIDOR - TIGHT     # 3.0
WALL_BLOCK = HALF_CORRIDOR - LOOSE      # 2.7
OBS_SIZE = 1.2
TRACK_ERR = 0.062                        # 3c measured track error
C2_TOL = 0.02                            # wall geometry error, with margin
FRONT_SLACK = 0.15                       # 3b near face bound
SIDE_SLACK = 0.10                        # 3c track error 0.062, with margin
DETECT_RANGE = 28.0
MIN_SPEED = 0.5                          # m/s, a grid counts as moving above this


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
    backward = {t: 0 for t in topics}
    last = {t: -1.0 for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = deserialize_message(data, cls[topic])
        s = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        if s < last[topic]:
            backward[topic] += 1
        last[topic] = s
        out[topic].append((s, m))
    for t in topics:
        out[t].sort(key=lambda p: p[0])
    return out, backward


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    if len(sys.argv) not in (2, 4):
        print(__doc__)
        sys.exit(1)
    uri = sys.argv[1]
    obstacle = None
    if len(sys.argv) == 4:
        obstacle = (float(sys.argv[2]), float(sys.argv[3]))

    data, backward = read_sorted(uri, [GRID_TOPIC, POSE_TOPIC])
    # Drop repeated pose stamps before differentiating: an orphaned node or a
    # duplicate publisher gives two poses at one stamp and a divide by zero.
    poses, seen = [], set()
    for s, m in data[POSE_TOPIC]:
        if s not in seen:
            seen.add(s)
            poses.append((s, m))
    pt = np.array([s for s, _ in poses])
    px = np.array([m.pose.position.x for _, m in poses])
    py = np.array([m.pose.position.y for _, m in poses])
    pyaw = np.array([yaw_of(m.pose.orientation) for _, m in poses])
    speed = np.abs(np.gradient(px, pt))

    print(f"bag: {uri}")
    print(f"grids: {len(data[GRID_TOPIC])}, poses: {len(poses)}, "
          f"backward intervals in bag order: {backward}")

    c1_fail = c2_fail = c3_fail = 0
    c1_worst = 0.0
    c2_counts = {"wall not lethal": 0, "centre lethal": 0, "band not 99": 0,
                 "inside blocked": 0}
    c3_counts = {"inside tight not lethal": 0, "lethal too far in front": 0,
                 "lethal too far beside": 0}
    c3_worst_front = c3_worst_side = 0.0
    scored = c3_scored = 0
    worst_gap = 0.0

    for s, g in data[GRID_TOPIC]:
        i = int(np.clip(np.searchsorted(pt, s), 1, len(pt) - 1))
        if abs(pt[i - 1] - s) < abs(pt[i] - s):
            i -= 1
        if speed[i] < MIN_SPEED:
            continue
        worst_gap = max(worst_gap, abs(pt[i] - s))
        scored += 1
        vx, vy, yaw = px[i], py[i], pyaw[i]

        res = g.info.resolution
        nx, ny = g.info.width, g.info.height
        cells = np.array(g.data, dtype=np.int16).reshape(ny, nx)
        xs = g.info.origin.position.x + (np.arange(nx) + 0.5) * res
        ys = g.info.origin.position.y + (np.arange(ny) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)                 # shape (ny, nx), base_link
        c, sn = math.cos(yaw), math.sin(yaw)
        WX = vx + c * X - sn * Y                   # world coordinates of each cell
        WY = vy + sn * X + c * Y
        D = WY                                     # signed distance from centreline

        # Cells near an obstacle are excluded from the wall checks.
        near_obs = np.zeros_like(D, dtype=bool)
        if obstacle is not None:
            ox, oy = obstacle
            x0, x1 = ox - 0.5 * OBS_SIZE, ox + 0.5 * OBS_SIZE
            y0, y1 = oy - 0.5 * OBS_SIZE, oy + 0.5 * OBS_SIZE
            reach = LOOSE + PAD + TRACK_ERR + res
            near_obs = (WX >= x0 - reach) & (WY >= y0 - reach) & (WY <= y1 + reach)

        # C1, only without an obstacle: the minimum is the centreline only then.
        if obstacle is None:
            fail = False
            for col in range(nx):
                v = cells[:, col]
                at_min = v == v.min()
                worst = np.abs(D[at_min, col]).max()
                c1_worst = max(c1_worst, worst)
                if worst > res:
                    fail = True
            c1_fail += fail

        # C2
        ad = np.abs(D)
        keep = ~near_obs
        f = {
            "wall not lethal": (ad > WALL_LETHAL + C2_TOL) & keep & (cells != 100),
            "centre lethal": (ad < WALL_LETHAL - C2_TOL) & keep & (cells == 100),
            "band not 99": (ad > WALL_BLOCK + C2_TOL) & (ad < WALL_LETHAL - C2_TOL)
                           & keep & (cells != 99),
            "inside blocked": (ad < WALL_BLOCK - C2_TOL) & keep & (cells > 98),
        }
        any_fail = False
        for k, mask in f.items():
            n = int(mask.sum())
            c2_counts[k] += n
            any_fail |= n > 0
        c2_fail += any_fail

        # C3
        if obstacle is not None:
            # base_link x of the true near face: the range C3 is scored over.
            near_bl = (x0 - vx) * c + (oy - vy) * sn
            if 0.0 <= near_bl <= DETECT_RANGE:
                c3_scored += 1
                dx = np.maximum(0.0, np.maximum(x0 - WX, WX - x1))
                dy = np.maximum(0.0, np.maximum(y0 - WY, WY - y1))
                dist = np.hypot(dx, dy)
                lethal = cells == 100
                away = ad < WALL_LETHAL - C2_TOL
                inside = (dist <= TIGHT) & (cells != 100)
                front_ex = (x0 - WX) - (TIGHT + FRONT_SLACK)
                front = lethal & away & (front_ex > 0) & (WY >= y0 - TIGHT) & (WY <= y1 + TIGHT)
                side_ex = np.maximum(y0 - WY, WY - y1) - (TIGHT + PAD + SIDE_SLACK)
                side = lethal & away & (side_ex > 0) & (WX >= x0 - TIGHT)
                c3_counts["inside tight not lethal"] += int(inside.sum())
                c3_counts["lethal too far in front"] += int(front.sum())
                c3_counts["lethal too far beside"] += int(side.sum())
                if front.any():
                    c3_worst_front = max(c3_worst_front, float(front_ex[front].max()))
                if side.any():
                    c3_worst_side = max(c3_worst_side, float(side_ex[side].max()))
                c3_fail += bool(inside.any() or front.any() or side.any())

    print(f"moving grids scored: {scored}, worst grid to pose gap {worst_gap * 1e3:.1f} ms\n")
    if scored == 0:
        print("no moving grids: nothing scored")
        return

    if obstacle is None:
        print(f"C1 centreline: {scored - c1_fail}/{scored} grids pass, "
              f"worst min-cost cell {c1_worst:.3f} m from the true centreline "
              f"(limit {0.25:.2f})  -> {'PASS' if c1_fail == 0 else 'FAIL'}")
    print(f"C2 walls:      {scored - c2_fail}/{scored} grids pass "
          f"-> {'PASS' if c2_fail == 0 else 'FAIL'}")
    for k, n in c2_counts.items():
        print(f"    {k}: {n} cells")
    if obstacle is not None:
        print(f"\nC3 inflation:  {c3_scored - c3_fail}/{c3_scored} grids pass "
              f"-> {'PASS' if c3_fail == 0 and c3_scored > 0 else 'FAIL'}")
        for k, n in c3_counts.items():
            print(f"    {k}: {n} cells")
        print(f"    worst excess in front {c3_worst_front:.3f} m, beside {c3_worst_side:.3f} m")


if __name__ == "__main__":
    main()
