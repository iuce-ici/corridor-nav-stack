#!/usr/bin/env python3
"""3d-3 criteria E1 to E4(a) for the free_space node.

Replays every recorded scan into its own voxel memory, written independently
of the node from the agreed rules, using the EKF pose the node reports it used
(/free_space/pose), and compares the result with the node's /drivable grid
scan by scan. The node numbers its scans; if the bag does not hold every one
from the first, E4a and E1 are reported NOT VALID instead of PASS or FAIL.

  E1  evidence only: no cell the node calls drivable lacks floor evidence
      (a floor return or a beam through the floor voxel), a granted zone, or
      the vehicle's own footprint. Drivable also needs all 8 neighbouring
      columns verified (one voxel of erosion); the replay applies that too
  E2  never inside an obstacle: no drivable cell whose centre lies inside a
      true obstacle footprint (ground truth pose, test apparatus)
  E3  proximity zone: never granted while a true obstacle footprint reaches
      into the half ellipse ahead of the bumper; granted on more than 95 % of
      the scans where none does
  E4a the node's grid equals this replay on every scan

Sensor mount and every threshold are written here from the agreed values,
not read from the node: sensor at base_link (4.2, 0, 1.0), pitched 0.20944 rad
nose down; voxel 0.25 m, 0 to 4.5 m, lateral window +-3.75 m (15 voxels each side); floor below
0.15 m; a pass ends 0.25 m before its return; missing returns ignored; body
from x -1.0 to 4.0, half width 0.9 m, its footprint free by definition; zone a
half ellipse centred on the middle of the front bumper, 5 m ahead by 1.5 m to
each side, in the vehicle frame.

Usage (from ~/ros2_ws):
  python3 src/corridor_experiments/scripts/analyse_free_space.py bags/NAME \\
      src/corridor_sim/worlds/corridor_SCENARIO.obstacles.json [--why] [--e2]
  --why  newly occupied voxels inside the zone, with the return they came from
  --e2   every drivable cell inside a true box: where, how deep, on what evidence
  --diff every cell where the node and the replay disagree (first 30)
Record: /scan/points /free_space/pose /free_space/mount /drivable /model/vehicle/pose
"""
import json
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

LIDAR = np.array([4.2, 0.0, 1.0])
PITCH = 0.20944
VOX, Z_MAX, HW = 0.25, 4.5, 3.75
FLOOR_Z, MARGIN = 0.15, 0.25
BEHIND, AHEAD = 20.0, 40.0
ZONE, ZONE_LAT, REAR, BUMPER, BODY_HW = 5.0, 1.5, -1.0, 4.0, 0.9


def in_zone(xb, yb):
    """Base_link points inside the half ellipse, boundary included; same
    arithmetic as the node."""
    u = (xb - BUMPER) / ZONE
    v = yb / ZONE_LAT
    return (xb >= BUMPER) & (u * u + v * v <= 1.0)


def in_footprint(xb, yb):
    return (xb >= REAR) & (xb <= BUMPER) & (np.abs(yb) <= BODY_HW)
RES, X_MAX, Y_HALF = 0.25, 30.0, 6.0
TOPICS = ["/scan/points", "/free_space/pose", "/drivable", "/model/vehicle/pose",
          "/free_space/mount"]


def stamp(m):
    return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec


def load(uri):
    for sid in ("", "mcap", "sqlite3"):
        try:
            r = rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid),
                   rosbag2_py.ConverterOptions("", ""))
            break
        except Exception:
            r = None
    if r is None:
        sys.exit(f"could not open {uri}")
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    miss = [t for t in TOPICS if t not in types]
    if miss:
        sys.exit(f"missing topics {miss}")
    cls = {t: get_message(types[t]) for t in TOPICS}
    r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    out = {t: {} for t in TOPICS}
    while r.has_next():
        topic, data, _ = r.read_next()
        m = deserialize_message(data, cls[topic])
        if topic == "/free_space/mount":
            out[topic].setdefault(len(out[topic]), list(m.data))
        else:
            out[topic].setdefault(stamp(m), m)
    return out


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Memory:
    """Voxel evidence in odom_ekf as dense arrays over the whole run: no
    rolling, so nothing is ever evicted."""

    def __init__(self, x_lo, x_hi):
        self.g0 = math.floor(x_lo / VOX)
        self.nx = math.floor(x_hi / VOX) - self.g0 + 1
        assert abs(HW / VOX - round(HW / VOX)) < 1e-9, "half width must be whole voxels"
        self.ny = int(round(2 * HW / VOX))
        self.nz = int(round(Z_MAX / VOX))
        self.free = np.zeros((self.nx, self.ny, self.nz), bool)
        self.occ = np.zeros((self.nx, self.ny, self.nz), bool)
        self.floor = np.zeros((self.nx, self.ny), bool)

    def insert(self, px, py, ptheta, pts_sensor, mount):
        # The mount exactly as the node received it from TF (already checked
        # against the agreed 12 degrees and (4.2, 0, 1.0) in main), evaluated
        # term by term in the node's order, so the replay matches bit for bit.
        R00, R01, R02, R10, R11, R12, R20, R21, R22, tx, ty, tz = mount
        sx_, sy_, sz_ = pts_sensor[:, 0], pts_sensor[:, 1], pts_sensor[:, 2]
        bx = R00 * sx_ + R01 * sy_ + R02 * sz_ + tx
        by = R10 * sx_ + R11 * sy_ + R12 * sz_ + ty
        bz = R20 * sx_ + R21 * sy_ + R22 * sz_ + tz
        cy, sy = math.cos(ptheta), math.sin(ptheta)
        wx = px + cy * bx - sy * by
        wy = py + sy * bx + cy * by
        wz = bz
        ox = px + cy * tx - sy * ty
        oy = py + sy * tx + cy * ty
        oz = tz
        lo = math.floor((px - BEHIND) / VOX) + 1
        hi = math.floor((px + AHEAD) / VOX) - 1

        d = np.stack([wx - ox, wy - oy, wz - oz], 1)
        r = np.sqrt(d[:, 0] * d[:, 0] + d[:, 1] * d[:, 1] + d[:, 2] * d[:, 2])
        keep = r >= 1e-6
        d, r = d[keep], r[keep]
        wx, wy, wz = wx[keep], wy[keep], wz[keep]
        u = d / r[:, None]
        step = 0.5 * VOX
        T = step * np.arange(1, int(31.0 / step) + 2)
        q = np.array([ox, oy, oz])[None, None, :] + T[None, :, None] * u[:, None, :]
        ok = (T[None, :] < (r[:, None] - MARGIN)) & (q[:, :, 2] >= 0.0)
        q = q[ok]
        gx = np.floor(q[:, 0] / VOX).astype(np.int64)
        iy = np.floor((q[:, 1] + HW) / VOX).astype(np.int64)
        iz = np.floor(q[:, 2] / VOX).astype(np.int64)
        v = ((gx >= lo) & (gx <= hi) & (iy >= 0) & (iy < self.ny) & (iz >= 0) & (iz < self.nz)
             & (gx - self.g0 >= 0) & (gx - self.g0 < self.nx))
        self.free[gx[v] - self.g0, iy[v], iz[v]] = True

        egx = np.floor(wx / VOX).astype(np.int64)
        eiy = np.floor((wy + HW) / VOX).astype(np.int64)
        eiz = np.floor(wz / VOX).astype(np.int64)
        inw = ((egx >= lo) & (egx <= hi) & (eiy >= 0) & (eiy < self.ny)
               & (egx - self.g0 >= 0) & (egx - self.g0 < self.nx))
        fl = inw & (wz < FLOOR_Z)
        self.floor[egx[fl] - self.g0, eiy[fl]] = True
        oc = inw & (wz >= FLOOR_Z) & (eiz >= 0) & (eiz < self.nz)
        new = oc.copy()
        new[oc] = ~self.occ[egx[oc] - self.g0, eiy[oc], eiz[oc]]
        self.occ[egx[oc] - self.g0, eiy[oc], eiz[oc]] = True
        # Where each newly occupied voxel came from, for --why: the return in
        # base_link and in the sensor frame.
        self.last_new = [(int(g), int(j), int(k), float(xb_), float(yb_), float(zb_),
                          float(xs_), float(ys_), float(zs_))
                         for g, j, k, xb_, yb_, zb_, xs_, ys_, zs_ in
                         zip(egx[new], eiy[new], eiz[new], bx[keep][new], by[keep][new],
                             bz[keep][new], sx_[keep][new], sy_[keep][new], sz_[keep][new])]

    def zone_ok(self, px, py, pth):
        """No occupied voxel (any height) with its centre in the ellipse."""
        c, s = math.cos(pth), math.sin(pth)
        col = self.occ.any(axis=2)
        gi, ji = np.nonzero(col)
        dx = (gi + self.g0 + 0.5) * VOX - px
        dy = -HW + (ji + 0.5) * VOX - py
        xb = c * dx + s * dy
        yb = -s * dx + c * dy
        return not in_zone(xb, yb).any()

    def cells(self, X, Y, px, py, pth, zone):
        """Arrays of world x, y: value 0 drivable, 50 not verified, 100
        occupied or outside; and, for the column each point falls in, whether
        it has floor evidence, a granted zone, or lies in the footprint
        (both judged at the column centre)."""
        c, s = math.cos(pth), math.sin(pth)
        gi, ji = np.meshgrid(np.arange(self.nx), np.arange(self.ny), indexing="ij")
        dx = (gi + self.g0 + 0.5) * VOX - px
        dy = -HW + (ji + 0.5) * VOX - py
        xb = c * dx + s * dy
        yb = -s * dx + c * dy
        footv = in_footprint(xb, yb)
        zonev = zone & in_zone(xb, yb)
        evv = self.floor | self.free[:, :, 0]
        raw = np.where(footv, 0, np.where(self.occ.any(axis=2), 100,
                                          np.where(evv | zonev, 0, 50)))
        # One voxel of erosion: drivable only if all 8 neighbours are
        # verified. Outside the lateral window counts as not verified.
        pad = np.full((self.nx + 2, self.ny + 2), 50)
        pad[:, 0] = pad[:, -1] = 100
        pad[1:-1, 1:-1] = raw
        bad = np.zeros_like(raw, bool)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di or dj:
                    bad |= pad[1 + di:self.nx + 1 + di, 1 + dj:self.ny + 1 + dj] != 0
        er = np.where(raw != 0, raw, np.where(bad, 50, 0))

        g = np.floor(X / VOX).astype(np.int64) - self.g0
        j = np.floor((Y + HW) / VOX).astype(np.int64)
        inside = (j >= 0) & (j < self.ny) & (g >= 0) & (g < self.nx)
        gc, jc = np.clip(g, 0, self.nx - 1), np.clip(j, 0, self.ny - 1)
        val = np.where(inside, er[gc, jc], 100)
        return val, evv[gc, jc] & inside, zonev[gc, jc] & inside, footv[gc, jc] & inside


def main():
    why = "--why" in sys.argv
    if why:
        sys.argv.remove("--why")
    e2why = "--e2" in sys.argv
    if e2why:
        sys.argv.remove("--e2")
    diffwhy = "--diff" in sys.argv
    if diffwhy:
        sys.argv.remove("--diff")
    diffs = []
    e2rows = []          # (scan s, truth x, EKF minus truth x, face, depth, evidence, u, w, voxel y span)
    drift = []
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    data = load(sys.argv[1])
    spec = json.load(open(sys.argv[2]))
    sx, sy, _ = spec["size"]
    boxes = [(o["x"], o["y"]) for o in spec["obstacles"]]
    truth = sorted((s, m.pose) for s, m in data["/model/vehicle/pose"].items())
    tt = np.array([s for s, _ in truth], float)

    xs_all = [m.pose.position.x for m in data["/free_space/pose"].values()]
    if not xs_all:
        sys.exit("no /free_space/pose messages")
    mem = Memory(min(xs_all) - BEHIND - 2.0, max(xs_all) + AHEAD + 2.0)

    mounts = list(data["/free_space/mount"].values())
    if not mounts:
        sys.exit("no /free_space/mount messages")
    mount = mounts[0]
    if any(m != mount for m in mounts):
        sys.exit("the node's mount changed during the run")
    c, s = math.cos(PITCH), math.sin(PITCH)
    agreed = [c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c, LIDAR[0], LIDAR[1], LIDAR[2]]
    dev = max(abs(a - b) for a, b in zip(mount, agreed))
    print(f"mount used by the node against the agreed 12 deg and (4.2, 0, 1.0): "
          f"max difference {dev:.2e}" + ("" if dev < 1e-9 else "  <- NOT the agreed mount"))
    gx_n = int(round(X_MAX / RES)) + 1
    gy_n = int(round(2 * Y_HALF / RES)) + 1
    xs = np.arange(gx_n) * RES
    ys = -Y_HALF + np.arange(gy_n) * RES

    n = mism = e1 = e2 = 0
    why_shown = [0]
    z_near = z_near_granted = z_clear = z_clear_granted = 0
    zone_disagree = 0
    scans = sorted(data["/scan/points"].items())
    # Scans the node used (it reported a pose) that the bag does not hold:
    # their evidence is in the node's map but can never be in the replay.
    have = set(data["/scan/points"])
    used_missing = sorted(t for t in data["/free_space/pose"] if t not in have)
    grids_missing = sum(1 for t in data["/free_space/pose"] if t not in data["/drivable"])
    # The node numbers the scans it inserts (orientation.y, 1 for the first).
    # The replay can only match if the bag holds every one of them from the
    # first: a recorder that subscribed late, or a lost pose message, leaves
    # evidence in the node's map that the replay can never have.
    counts = [int(round(m.pose.orientation.y)) for _, m in sorted(data["/free_space/pose"].items())]
    before = counts[0] - 1
    gaps = sum(1 for a, b in zip(counts, counts[1:]) if b != a + 1)
    complete = before == 0 and gaps == 0 and not used_missing
    for st, cl in scans:
        used = data["/free_space/pose"].get(st)
        if used is None:
            continue                     # the node skipped this scan
        P = used.pose
        px, py, pth = P.position.x, P.position.y, P.position.z    # test encoding
        raw = point_cloud2.read_points(cl, field_names=["x", "y", "z"], skip_nans=True)
        pts = np.column_stack([np.asarray(raw["x"], float), np.asarray(raw["y"], float),
                               np.asarray(raw["z"], float)])
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        mem.insert(px, py, pth, pts, mount)
        zone = mem.zone_ok(px, py, pth)
        if why:
            cw, sw = math.cos(pth), math.sin(pth)
            for (g, j, k, xb_, yb_, zb_, xs_, ys_, zs_) in mem.last_new:
                gw = (g + 0.5) * VOX
                ddx, ddy = gw - px, -HW + (j + 0.5) * VOX - py
                inz = bool(in_zone(np.float64(cw * ddx + sw * ddy), np.float64(-sw * ddx + cw * ddy)))
                if inz and why_shown[0] < 40:
                    why_shown[0] += 1
                    print(f"  scan {st / 1e9:8.3f} s  vehicle x {px:6.2f}: new occupied voxel at "
                          f"{gw - px:+6.2f} m ahead, y {-HW + (j + 0.5) * VOX:+5.2f}, "
                          f"z {(k + 0.5) * VOX:4.2f} | return in base_link ({xb_:+6.2f}, {yb_:+5.2f}, "
                          f"{zb_:+5.2f}), sensor frame ({xs_:+6.2f}, {ys_:+6.2f}, {zs_:+6.2f})")
        zone_disagree += int(zone != (P.orientation.x > 0.5))

        grid = data["/drivable"].get(st)
        if grid is None:
            continue
        g = np.array(grid.data, np.int16).reshape(grid.info.height, grid.info.width)
        n += 1
        c, s = math.cos(pth), math.sin(pth)

        # Ground truth pose of the vehicle at this scan, for E2 and E3.
        i = int(np.clip(np.searchsorted(tt, st), 1, len(tt) - 1))
        if abs(tt[i - 1] - st) < abs(tt[i] - st):
            i -= 1
        T = truth[i][1]
        vx, vy, vth = T.position.x, T.position.y, yaw(T.orientation)
        ct, stt = math.cos(vth), math.sin(vth)

        XB, YB = np.meshgrid(xs, ys)                    # rows y, columns x, as the grid
        foot = in_footprint(XB, YB)
        val, ev, zone_mask, foot_col = mem.cells(px + c * XB - s * YB, py + s * XB + c * YB,
                                                 px, py, pth, zone)
        val = np.where(foot, 0, val)
        mism += int((g != val).sum())
        if diffwhy and len(diffs) < 30:
            for r_, k_ in zip(*np.nonzero(g != val)):
                if len(diffs) < 30:
                    diffs.append((st / 1e9, px, XB[r_, k_], YB[r_, k_], int(g[r_, k_]), int(val[r_, k_]),
                                  bool(ev[r_, k_])))
        drv = g == 0
        e1 += int((drv & ~(ev | zone_mask | foot_col | foot)).sum())
        WX = vx + ct * XB - stt * YB
        WY = vy + stt * XB + ct * YB
        drift.append(px - vx)
        for bx, by in boxes:
            bad = drv & (np.abs(WX - bx) < sx / 2) & (np.abs(WY - by) < sy / 2)
            e2 += int(bad.sum())
            if e2why and bad.any():
                EX = px + c * XB - s * YB         # where the node looked, odom_ekf
                EY = py + s * XB + c * YB
                for r_, k_ in zip(*np.nonzero(bad)):
                    u, w = WX[r_, k_] - bx, WY[r_, k_] - by
                    dx_, dy_ = sx / 2 - abs(u), sy / 2 - abs(w)
                    if dx_ <= dy_:
                        face, depth = ("front" if u < 0 else "back"), dx_
                    else:
                        face, depth = ("right" if w < 0 else "left"), dy_
                    gi = math.floor(EX[r_, k_] / VOX) - mem.g0
                    ji = math.floor((EY[r_, k_] + HW) / VOX)
                    if foot[r_, k_] or foot_col[r_, k_]:
                        evd = "footprint"
                    elif mem.floor[gi, ji]:
                        evd = "floor return"
                    elif mem.free[gi, ji, 0]:
                        evd = "beam through floor voxel"
                    elif zone_mask[r_, k_]:
                        evd = "zone"
                    else:
                        evd = "none"
                    ylo = -HW + ji * VOX
                    e2rows.append((st / 1e9, vx, px - vx, face, depth, evd, u, w, (ylo, ylo + VOX)))

        # E3: does any true box footprint reach into the half ellipse, in the
        # true vehicle frame? Box sampled every 2 cm, edges included.
        near = False
        for bx, by in boxes:
            u = np.linspace(-sx / 2, sx / 2, max(2, int(sx / 0.02) + 1))
            w = np.linspace(-sy / 2, sy / 2, max(2, int(sy / 0.02) + 1))
            BU, BW = np.meshgrid(bx + u, by + w)
            bxl = ct * (BU - vx) + stt * (BW - vy)
            byl = -stt * (BU - vx) + ct * (BW - vy)
            if in_zone(bxl, byl).any():
                near = True
        if near:
            z_near += 1
            z_near_granted += int(P.orientation.x > 0.5)
        else:
            z_clear += 1
            z_clear_granted += int(P.orientation.x > 0.5)

    print(f"bag: {sys.argv[1]}, scans compared {n}")
    print(f"scans the node inserted before the recording caught its first: {before}; "
          f"gaps in its count: {gaps}")
    print(f"scans the node used but the bag lacks: {len(used_missing)}"
          + (f" (first at {used_missing[0] / 1e9:.3f} s)" if used_missing else "")
          + f"; node poses without a recorded grid: {grids_missing}")
    if diffwhy:
        for d_ in diffs:
            print(f"  scan {d_[0]:8.3f} s  EKF x {d_[1]:6.2f}  cell xb {d_[2]:5.2f} yb {d_[3]:+5.2f}: "
                  f"node {d_[4]:3d}, replay {d_[5]:3d}, replay floor evidence {d_[6]}")
    invalid = "NOT VALID: the bag lacks scans the node built its map from; record again"
    print(f"E4a node grid against replay: {mism} differing cells over {n} grids -> "
          f"{('PASS' if mism == 0 else 'FAIL') if complete else invalid}; "
          f"zone decision differs on {zone_disagree} scans")
    print(f"E1  drivable without floor evidence or zone: {e1} -> "
          f"{('PASS' if e1 == 0 else 'FAIL') if complete else invalid}")
    print(f"E2  drivable cells inside a true box: {e2} -> {'PASS' if e2 == 0 else 'FAIL'}")
    if e2why:
        d = np.array(drift)
        print(f"--e2: EKF x minus true x over the run: start {d[0]:+.3f}, end {d[-1]:+.3f}, "
              f"range {d.min():+.3f} to {d.max():+.3f} m")
        if e2rows:
            from collections import Counter
            print("  nearest face:", dict(Counter(r[3] for r in e2rows)))
            print("  evidence:    ", dict(Counter(r[5] for r in e2rows)))
            dep = np.array([r[4] for r in e2rows])
            edges = [0, 0.05, 0.10, 0.15, 0.20, 0.30, 1.0]
            hist = np.histogram(dep, edges)[0]
            print("  depth inside the box (m):", ", ".join(
                f"{a:.2f} to {b:.2f}: {h}" for a, b, h in zip(edges[:-1], edges[1:], hist)))
            us = np.array([r[6] for r in e2rows]); ws = np.array([r[7] for r in e2rows])
            print(f"  box-relative cell centres: u (along x) {us.min():+.2f} to {us.max():+.2f}, "
                  f"|w| (across) {np.abs(ws).min():.3f} to {np.abs(ws).max():.3f} m "
                  f"(box half sizes {sx / 2:.2f} by {sy / 2:.2f})")
            print("  first 15:")
            for r in e2rows[:15]:
                print(f"    scan {r[0]:8.3f} s  true x {r[1]:6.2f}  EKF-true {r[2]:+.3f}  {r[3]:5s} "
                      f"depth {r[4]:.3f}  u {r[6]:+.3f} w {r[7]:+.3f}  voxel y {r[8][0]:+.2f} to "
                      f"{r[8][1]:+.2f}  evidence: {r[5]}")
    share = z_clear_granted / z_clear if z_clear else float('nan')
    ok3 = z_near_granted == 0 and (z_clear == 0 or share > 0.95)
    print(f"E3  zone granted with a box in the ellipse: {z_near_granted} of {z_near} scans; "
          f"granted with none: {z_clear_granted} of {z_clear} ({100 * share:.1f} %) -> "
          f"{'PASS' if ok3 else 'FAIL'}")


if __name__ == "__main__":
    main()
