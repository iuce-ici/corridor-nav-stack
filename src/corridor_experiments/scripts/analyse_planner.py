#!/usr/bin/env python3
"""Phase 3d-2 criteria P1 to P8 for the path planner, from a recorded bag.

Test apparatus: reads ground truth, which the system under test never may.
Every topic is sorted by header stamp. A path is paired with the grid it was
planned on by exact stamp: the planner copies the grid's stamp.

P1 validity     every path cell after the start is below 99 in its grid,
                every step goes one column forward and at most one row
                sideways, and a non-empty path reaches the last column
P2 optimality   path cost equals the cost an independent exact search finds
                on the same grid (relative difference under 1e-6); an empty
                path is correct only if that search finds no path either.
                Under the side rule (default; --no-side-rule for bags made
                before it) the search is constrained to the sides the rule
                picks, and the rule is re-applied here independently, with
                its own memory per track id: nearest track first, C_R and C_L
                by exact search, right unless left is cheaper by more than
                SIDE_E, a committed side kept unless the other is cheaper by
                more than SIDE_H or it has closed. The path must also pass
                every decided track on the decided side. Grids whose tracks
                are missing from the bag are not checked and are counted.
P3 centreline   (--centreline) every path cell from x 10 m to the end lies
                within 0.25 m of the true centreline
P4 side         (--obstacles X,Y ... --sides R|L|A ...) on every grid where the
                obstacle is confirmed, the path passes on the expected side,
                with no side flip between consecutive grids. A means either
                side is acceptable (a centred obstacle, a geometric tie): no
                wrong side is counted, the R and L counts are reported, and
                only the flips decide the verdict.
P5 no path      (--blocked X,Y) on every grid where the block is confirmed,
                the path is empty
P6 start        every non-empty path starts at the vehicle origin cell (0, 0)
P7 plan time    from the node's own log line, not from the bag
P8 silence      on every scan where the corridor geometry is invalid, no path
                is published; and no path is ever published without a grid.
                Reported NOT TESTED unless at least MIN_INVALID invalid scans
                are in the bag, so it cannot pass by never being exercised.

Path cost, the same rule as the node: each step into a cell costs
  step length * (1 + W * c / 98),  step length 0.25 m straight, 0.25 * sqrt(2) diagonal,
cells at 99 or 100 cannot be entered, the start cell is exempt.

Usage:
  python3 analyse_planner.py bags/NAME [--centreline] [--no-side-rule]
         [--obstacles X,Y [X,Y ...] --sides S [S ...]] [--blocked X,Y [X,Y ...]]
  Obstacle positions are world centres, as in generate_corridor.py.
"""
import argparse
import math
import sys

import numpy as np

W = 1.0
BLOCKED_FROM = 99
OBS_SIZE = 1.2
MATCH = 2.0            # metres, track near face to true near face, for "confirmed"
CENTRE_TOL = 0.25      # P3
CENTRE_FROM = 10.0     # P3, metres ahead
REL_TOL = 1e-6         # P2
MIN_INVALID = 20       # P8
SIDE_E = 0.03          # side rule: right unless left cheaper by more than this
SIDE_H = 0.10          # side rule: a committed side switches only beyond this


# ---------------------------------------------------------------- core checks

def cell_index(x, y, ox, oy, res):
    return int(round((x - ox) / res - 0.5)), int(round((y - oy) / res - 0.5))


def path_cost(cells, idx, res):
    """Cost of a path given as (i, j) indices; None and a reason if illegal."""
    diag = res * math.sqrt(2.0)
    total = 0.0
    for (i0, j0), (i1, j1) in zip(idx, idx[1:]):
        if i1 != i0 + 1 or abs(j1 - j0) > 1:
            return None, "illegal move"
        if not (0 <= j1 < cells.shape[0]):
            return None, "leaves the grid"
        c = int(cells[j1, i1])
        if c >= BLOCKED_FROM or c < 0:
            return None, "enters a blocked cell"
        total += (res if j1 == j0 else diag) * (1.0 + W * c / 98.0)
    return total, None


def exact_search(cells, si, sj, res):
    """Column by column dynamic programme over forward moves: exact for this
    move set, and written independently of the node's A*. Returns the minimum
    cost to any cell of the last column, or inf."""
    ny, nx = cells.shape
    diag = res * math.sqrt(2.0)
    cost = np.full(ny, np.inf)
    cost[sj] = 0.0
    for i in range(si, nx - 1):
        nxt = np.full(ny, np.inf)
        col = cells[:, i + 1].astype(float)
        enter = (col < BLOCKED_FROM) & (col >= 0)
        weight = 1.0 + W * col / 98.0
        for dj, step in ((0, res), (1, diag), (-1, diag)):
            src = np.full(ny, np.inf)
            if dj == 0:
                src = cost
            elif dj == 1:
                src[1:] = cost[:-1]         # from row j - 1 to row j
            else:
                src[:-1] = cost[1:]         # from row j + 1 to row j
            nxt = np.minimum(nxt, src + step * weight)
        cost = np.where(enter, nxt, np.inf)
        if not np.isfinite(cost).any():
            return np.inf
    return float(cost.min())


def shut_side(cells, col, y, side, oy, res):
    """Copy of cells with one side of a track closed in column col."""
    out = cells.copy()
    yrow = oy + (np.arange(cells.shape[0]) + 0.5) * res
    out[(yrow >= y) if side == "R" else (yrow <= y), col] = 100
    return out


def apply_side_rule(cells, ox, oy, res, si, sj, tracks, memory, stats):
    """Re-apply the planner's side rule. tracks: [(id, near_x, y)] in base_link.
    memory: {id: side}, updated in place. Returns the constrained cells and
    the decisions [(col, y, side)]."""
    decided = []
    for tid, x, y in sorted(tracks, key=lambda t: t[1]):
        col = int(round((x - ox) / res - 0.5))
        if col <= si or col >= cells.shape[1]:
            continue
        # A side with no open cell in the near face column is closed; if
        # only one side has any, it is forced without a search.
        yrow = oy + (np.arange(cells.shape[0]) + 0.5) * res
        colv = cells[:, col]
        open_cell = (colv < BLOCKED_FROM) & (colv >= 0)
        open_r, open_l = bool((open_cell & (yrow < y)).any()), bool((open_cell & (yrow > y)).any())
        r = shut_side(cells, col, y, "R", oy, res)
        l = shut_side(cells, col, y, "L", oy, res)
        if not open_r and not open_l:
            continue
        if open_r != open_l:
            side = "R" if open_r else "L"
            stats["forced"] = stats.get("forced", 0) + 1
            fr = fl = None
        else:
            cr, cl = exact_search(r, si, sj, res), exact_search(l, si, sj, res)
            fr, fl = np.isfinite(cr), np.isfinite(cl)
            if not fr and not fl:
                continue
            stats["searched"] = stats.get("searched", 0) + 1
        if fr is None:
            pass
        elif fr and not fl:
            side = "R"
        elif fl and not fr:
            side = "L"
        else:
            m = (cl - cr) / min(cl, cr)
            prev = memory.get(tid)
            if prev is None:
                side = "L" if m < -SIDE_E else "R"
            elif prev == "R":
                side = "L" if m < -SIDE_H else "R"
            else:
                side = "R" if m > SIDE_H else "L"
        if tid in memory and memory[tid] != side:
            stats["switches"] += 1
        memory[tid] = side
        stats[side] += 1
        cells = r if side == "R" else l
        decided.append((col, y, side))
    present = {t[0] for t in tracks}
    for tid in list(memory):
        if tid not in present:
            del memory[tid]
    return cells, decided


def evaluate(grids, paths, poses, tracks, geoms, centreline=False,
             obstacles=(), sides=(), blocked=(), rule_tracks=None):
    """grids:  {stamp: (cells (ny, nx) int array, ox, oy, res)}
    paths:  [(stamp, [(x, y), ...])] sorted by stamp
    poses:  (t, x, y, yaw) arrays, ground truth
    tracks: {stamp: [(near_x, y), ...]} confirmed tracks in base_link
    geoms:  [(stamp, valid)]
    Returns a list of report lines and a dict of verdicts."""
    out, verdict = [], {}
    pt, px, py, pyaw = poses

    def pose_at(s):
        i = int(np.clip(np.searchsorted(pt, s), 1, len(pt) - 1))
        if abs(pt[i - 1] - s) < abs(pt[i] - s):
            i -= 1
        return px[i], py[i], pyaw[i]

    def to_base(wx, wy, vx, vy, yaw):
        dx, dy = wx - vx, wy - vy
        c, s = math.cos(yaw), math.sin(yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def confirmed(stamp, wx, wy, vx, vy, yaw):
        nx_, ny_ = to_base(wx - 0.5 * OBS_SIZE, wy, vx, vy, yaw)
        return any(math.hypot(tx - nx_, ty - ny_) < MATCH for tx, ty in tracks.get(stamp, []))

    p1 = {"enters a blocked cell": 0, "illegal move": 0, "leaves the grid": 0,
          "does not reach the last column": 0}
    p1_fail = p2_fail = p6_fail = 0
    p2_worst = 0.0
    p2_empty_wrong = p2_path_when_none = 0
    p3_scored = p3_fail = 0
    p3_worst = 0.0
    p4 = [{"scored": 0, "wrong": 0, "flips": 0, "last": None, "R": 0, "L": 0}
          for _ in obstacles]
    p5_scored = p5_fail = 0
    no_grid = 0
    nonempty = 0
    memory = {}
    rule_stats = {"R": 0, "L": 0, "switches": 0, "no tracks": 0, "wrong side": 0}

    # The recorder subscribes topic by topic, so the first paths can predate
    # its subscription to /cost_grid: their grids were published but never
    # recorded. Those are not evidence of a path without a grid.
    first_grid = min(grids) if grids else None
    before_recording = 0
    for stamp, pts in paths:
        g = grids.get(stamp)
        if g is None:
            if first_grid is None or stamp < first_grid:
                before_recording += 1
            else:
                no_grid += 1
            continue
        cells, ox, oy, res = g
        ny, nx = cells.shape
        si, sj = cell_index(0.0, 0.0, ox, oy, res)
        # P1 and the path cost use the grid as published; the rule only
        # closes cells, it never changes a cost.
        decided = []
        check_p2 = True
        constrained = cells
        if rule_tracks is not None:
            if stamp not in rule_tracks:
                rule_stats["no tracks"] += 1
                check_p2 = False
            else:
                constrained, decided = apply_side_rule(cells, ox, oy, res, si, sj,
                                                       rule_tracks[stamp], memory, rule_stats)
        best = exact_search(constrained, si, sj, res)

        if not pts:
            if check_p2 and np.isfinite(best):
                p2_empty_wrong += 1
                p2_fail += 1
        else:
            nonempty += 1
            idx = [cell_index(x, y, ox, oy, res) for x, y in pts]
            if idx[0] != (si, sj):
                p6_fail += 1
            cost, why = path_cost(cells, idx, res)
            bad = why is not None
            if not bad and idx[-1][0] != nx - 1:
                why, bad = "does not reach the last column", True
            if not bad:
                for col, y, side in decided:
                    k = col - si
                    if 0 <= k < len(pts) and ((pts[k][1] < y) != (side == "R")):
                        rule_stats["wrong side"] += 1
                        p2_fail += 1
                        break
            if bad:
                p1[why] += 1
                p1_fail += 1
            elif not check_p2:
                pass
            elif not np.isfinite(best):
                p2_path_when_none += 1
                p2_fail += 1
            else:
                rel = abs(cost - best) / max(best, 1e-12)
                p2_worst = max(p2_worst, rel)
                if rel > REL_TOL:
                    p2_fail += 1

        vx, vy, yaw = pose_at(stamp)

        if centreline and pts:
            p3_scored += 1
            worst = 0.0
            for x, y in pts:
                if x < CENTRE_FROM:
                    continue
                # True centreline (world y = 0) in base_link at this x.
                yc = -(vy + math.sin(yaw) * x) / math.cos(yaw)
                worst = max(worst, abs(y - yc))
            p3_worst = max(p3_worst, worst)
            p3_fail += worst > CENTRE_TOL

        for k, ((wx, wy), want) in enumerate(zip(obstacles, sides)):
            if not pts or not confirmed(stamp, wx, wy, vx, vy, yaw):
                continue
            cx, cy = to_base(wx, wy, vx, vy, yaw)
            col = int(round((cx - ox) / res - 0.5)) - si
            if not (0 <= col < len(pts)):
                continue
            side = "R" if pts[col][1] < cy else "L"
            s = p4[k]
            s["scored"] += 1
            s[side] += 1
            s["wrong"] += want != "A" and side != want
            if s["last"] is not None and side != s["last"]:
                s["flips"] += 1
            s["last"] = side

        if blocked and any(confirmed(stamp, wx, wy, vx, vy, yaw) for wx, wy in blocked):
            p5_scored += 1
            p5_fail += bool(pts)

    out.append(f"paths: {len(paths)} ({nonempty} non-empty), grids: {len(grids)}"
               + (f"; {before_recording} paths stamped before the first recorded grid, "
                  "not checked" if before_recording else ""))
    verdict["P1"] = p1_fail == 0
    checked = len(paths) - no_grid - before_recording
    out.append(f"P1 validity:    {checked - p1_fail}/{checked} pass "
               f"-> {'PASS' if verdict['P1'] else 'FAIL'}")
    for k, n in p1.items():
        if n:
            out.append(f"    {k}: {n} paths")
    verdict["P2"] = p2_fail == 0
    out.append(f"P2 optimality:  worst relative cost difference {p2_worst:.2e} (limit {REL_TOL:.0e}), "
               f"empty when a path exists: {p2_empty_wrong}, path when none exists: "
               f"{p2_path_when_none} -> {'PASS' if verdict['P2'] else 'FAIL'}")
    if rule_tracks is not None:
        out.append(f"    side rule re-applied: decisions R {rule_stats['R']}, L {rule_stats['L']} "
                   f"(forced {rule_stats.get('forced', 0)}, searched {rule_stats.get('searched', 0)}), "
                   f"switches {rule_stats['switches']}; path on the wrong side of a decided "
                   f"track {rule_stats['wrong side']}; grids not checked, tracks missing from "
                   f"the bag {rule_stats['no tracks']}")
    if centreline:
        verdict["P3"] = p3_scored > 0 and p3_fail == 0
        out.append(f"P3 centreline:  {p3_scored - p3_fail}/{p3_scored} pass, worst {p3_worst:.3f} m "
                   f"(limit {CENTRE_TOL}) -> {'PASS' if verdict['P3'] else 'FAIL'}")
    for k, ((wx, wy), want) in enumerate(zip(obstacles, sides)):
        s = p4[k]
        ok = s["scored"] > 0 and s["wrong"] == 0 and s["flips"] == 0
        verdict[f"P4 ({wx},{wy})"] = ok
        out.append(f"P4 side ({wx}, {wy}) expect {want}: scored {s['scored']} "
                   f"(R {s['R']}, L {s['L']}), wrong side "
                   f"{s['wrong']}, flips {s['flips']} -> "
                   f"{'PASS' if ok else ('NOT TESTED' if s['scored'] == 0 else 'FAIL')}")
    if blocked:
        verdict["P5"] = p5_scored > 0 and p5_fail == 0
        out.append(f"P5 no path:     {p5_scored - p5_fail}/{p5_scored} empty while blocked -> "
                   f"{'PASS' if verdict['P5'] else ('NOT TESTED' if p5_scored == 0 else 'FAIL')}")
    verdict["P6"] = p6_fail == 0
    out.append(f"P6 start:       {nonempty - p6_fail}/{nonempty} start at (0, 0) -> "
               f"{'PASS' if verdict['P6'] else 'FAIL'}")
    out.append("P7 plan time:   read the node's 'plan time over N grids' log line")

    path_stamps = {s for s, _ in paths}
    invalid = [s for s, v in geoms if not v]
    leaked = sum(1 for s in invalid if s in path_stamps)
    tested = len(invalid) >= MIN_INVALID
    verdict["P8"] = tested and leaked == 0 and no_grid == 0
    state = "PASS" if verdict["P8"] else ("NOT TESTED" if not tested else "FAIL")
    out.append(f"P8 silence:     {len(invalid)} invalid geometry scans, paths published on them: "
               f"{leaked}, paths without a grid: {no_grid} -> {state}")
    return out, verdict


# ---------------------------------------------------------------- bag loading

def load(uri, with_ids=False):
    """with_ids also returns {stamp: [(id, near_x, y)]} for the side rule."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    topics = ["/cost_grid", "/planned_path", "/model/vehicle/pose",
              "/obstacle_tracks", "/corridor_geometry"]
    reader = None
    for sid in ("", "mcap", "sqlite3"):
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid),
                        rosbag2_py.ConverterOptions("", ""))
            break
        except Exception:
            reader = None
    if reader is None:
        sys.exit(f"could not open {uri}")
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        sys.exit(f"missing topics {missing}. Present: {sorted(types)}")
    cls = {t: get_message(types[t]) for t in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    raw = {t: [] for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = deserialize_message(data, cls[topic])
        # Integer nanoseconds as the key: exact pairing, no float rounding.
        raw[topic].append((m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, m))
    for t in topics:
        raw[t].sort(key=lambda p: p[0])

    grids = {}
    for s, g in raw["/cost_grid"]:
        cells = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        grids[s] = (cells, g.info.origin.position.x, g.info.origin.position.y,
                    float(g.info.resolution))
    paths = [(s, [(p.pose.position.x, p.pose.position.y) for p in m.poses])
             for s, m in raw["/planned_path"]]

    seen, rows = set(), []
    for s, m in raw["/model/vehicle/pose"]:
        if s in seen:
            continue
        seen.add(s)
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        rows.append((s, m.pose.position.x, m.pose.position.y, yaw))
    a = np.array(rows, dtype=float)
    poses = (a[:, 0], a[:, 1], a[:, 2], a[:, 3])

    tracks = {s: [(t.x, t.y) for t in m.tracks] for s, m in raw["/obstacle_tracks"]}
    geoms = [(s, m.valid) for s, m in raw["/corridor_geometry"]]
    if with_ids:
        ids = {s: [(t.id, t.x, t.y) for t in m.tracks] for s, m in raw["/obstacle_tracks"]}
        return grids, paths, poses, tracks, geoms, ids
    return grids, paths, poses, tracks, geoms


def parse_xy(items):
    return [tuple(float(v) for v in it.split(",")) for it in items]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--centreline", action="store_true")
    ap.add_argument("--obstacles", nargs="*", default=[])
    ap.add_argument("--sides", nargs="*", default=[])
    ap.add_argument("--blocked", nargs="*", default=[])
    ap.add_argument("--no-side-rule", action="store_true",
                    help="P2 without the side rule, for bags recorded before it")
    a = ap.parse_args()
    obstacles, blocked = parse_xy(a.obstacles), parse_xy(a.blocked)
    if len(obstacles) != len(a.sides):
        sys.exit("give one --sides entry (R, L or A) per --obstacles entry")
    if any(s not in ("R", "L", "A") for s in a.sides):
        sys.exit("--sides entries must be R, L or A")

    *data, ids = load(a.bag, with_ids=True)
    lines, _ = evaluate(*data, centreline=a.centreline,
                        obstacles=obstacles, sides=a.sides, blocked=blocked,
                        rule_tracks=None if a.no_side_rule else ids)
    print(f"bag: {a.bag}" + ("" if a.no_side_rule else
          f", side rule e {SIDE_E * 100:.0f} %, h {SIDE_H * 100:.0f} %"))
    print("\n".join(lines))


if __name__ == "__main__":
    main()
