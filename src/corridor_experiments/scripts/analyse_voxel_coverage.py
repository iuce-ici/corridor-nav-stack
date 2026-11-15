#!/usr/bin/env python3
"""Can a 3D voxel memory, built only from LiDAR evidence, see far enough ahead?

Viability study for 3d-3, offline, before any node is written. Every recorded
scan is replayed into a world fixed 3D voxel grid, placed by the ground truth
pose (test apparatus; the node will use the EKF, whose lateral error is a
fraction of a millimetre):

  free      a beam passed through the voxel, ending more than MARGIN beyond it
  occupied  a beam ended in it (returns below FLOOR_Z count as floor, not hits)
  unknown   neither

A ground cell is DRIVABLE when every voxel of the vehicle's height envelope
above it is verified free and none is occupied; a floor return verifies the
floor voxel. Reported twice: strict, and with the no floating objects fill
(voxels between the lowest and highest verified voxel of a column count as
free when no beam ended in that span), since road vehicles do not meet
floating objects. A hanging object that no beam has hit is the case the fill
cannot see.

After each scan it measures, in a lane the width of the vehicle, the KNOWN
FREE HORIZON: the distance from the front bumper to the first cell of the lane
that is not drivable. That is the distance the vehicle could plan into, and so
what its speed would be allowed from. Also reported: per distance ahead, the
share of envelope voxels seen free (where coverage runs out), and any voxel
marked free inside the true obstacle footprint (must be 0).

Usage (from ~/ros2_ws; the bag needs /scan/points and /model/vehicle/pose):
  python3 src/corridor_experiments/scripts/analyse_voxel_coverage.py bags/NAME \\
      [--lane Y ...] [--box X,Y,W,D,H] [--every N]
"""
import argparse
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

CLOUD, POSE = "/scan/points", "/model/vehicle/pose"
LIDAR = np.array([4.2, 0.0, 1.0])     # base_link
PITCH = 0.20944                       # 12 deg nose down
BUMPER = 4.0                          # front face, base_link x
HALF_W = 0.9                          # vehicle half width, outer wheel faces
VOX = 0.25
X0, X1 = 0.0, 150.0
Y0, Y1 = -4.0, 4.0
Z_LO, Z_HI = 0.0, 4.5                 # envelope, floor bin included
FLOOR_Z = 0.15                        # returns below this are floor
WALL_MARGIN = 0.15                    # returns this close to a wall are wall
MARGIN = 0.25                         # a pass must end this far beyond a voxel
STEP_T = VOX / 2.0
MOVING = 0.1                          # m/s
ZONE = 5.0                            # m around the body verified by the other safety sensors
REAR = -1.0                           # rear face, base_link x


def open_reader(uri):
    for sid in ("", "mcap", "sqlite3"):
        try:
            r = rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid),
                   rosbag2_py.ConverterOptions("", ""))
            return r
        except Exception:
            pass
    sys.exit(f"could not open {uri}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--lane", nargs="*", type=float, default=[0.0, -2.2, 2.2],
                    help="lane centres, world y")
    ap.add_argument("--box", default="50,0,0.6,1.2,1.5",
                    help="true obstacle x,y,width,depth,height for the footprint check")
    ap.add_argument("--every", type=int, default=1, help="use every Nth scan")
    a = ap.parse_args()
    box = tuple(float(v) for v in a.box.split(","))

    reader = open_reader(a.bag)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for t in (CLOUD, POSE):
        if t not in types:
            sys.exit(f"missing {t}")
    cls = {t: get_message(types[t]) for t in (CLOUD, POSE)}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[CLOUD, POSE]))
    clouds, poses = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = deserialize_message(data, cls[topic])
        s = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        (clouds if topic == CLOUD else poses).append((s, m))
    clouds.sort(key=lambda p: p[0])
    poses.sort(key=lambda p: p[0])
    pt = np.array([s for s, _ in poses], float)
    pxy = np.array([[m.pose.position.x, m.pose.position.y] for _, m in poses])
    pyaw = np.array([math.atan2(2 * (m.pose.orientation.w * m.pose.orientation.z),
                                1 - 2 * m.pose.orientation.z ** 2) for _, m in poses])

    def xyz(cl):
        raw = point_cloud2.read_points(cl, field_names=["x", "y", "z"], skip_nans=True)
        return np.column_stack([np.asarray(raw["x"], float), np.asarray(raw["y"], float),
                                np.asarray(raw["z"], float)])

    run([(st, xyz(cl)) for st, cl in clouds], pt, pxy, pyaw, a, box)


def run(clouds, pt, pxy, pyaw, a, box):
    bx, by, bw, bd, bh = box

    nx, ny, nz = (int(round((X1 - X0) / VOX)), int(round((Y1 - Y0) / VOX)),
                  int(round((Z_HI - Z_LO) / VOX)))
    free = np.zeros((nx, ny, nz), bool)
    hits = np.zeros((nx, ny, nz), np.int32)
    floor = np.zeros((nx, ny), bool)
    print(f"bag {getattr(a, 'bag', '')}: {len(clouds)} scans, voxel grid {nx} x {ny} x {nz} at {VOX} m, "
          f"envelope {Z_LO} to {Z_HI} m")

    c, s = math.cos(PITCH), math.sin(PITCH)
    R_ls = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])   # sensor to base_link
    T = np.arange(STEP_T, 30.0, STEP_T)


    zone_granted = []
    rows = []   # vehicle x, speed, horizon per lane
    limit_z = {}   # height of the first non drivable voxel at each lane horizon
    limit_zf = {}  # the same, with the no floating objects fill
    prof_seen = np.zeros(121)
    prof_n = 0
    for k, (st, cl) in enumerate(clouds):
        if k % a.every:
            continue
        i = int(np.clip(np.searchsorted(pt, st), 0, len(pt) - 1))
        vx, vy = pxy[i]
        yaw = pyaw[i]
        j0 = int(np.clip(np.searchsorted(pt, st - 1e8), 0, len(pt) - 1))
        j1 = int(np.clip(np.searchsorted(pt, st + 1e8), 0, len(pt) - 1))
        spd = (np.hypot(*(pxy[j1] - pxy[j0])) / ((pt[j1] - pt[j0]) * 1e-9)
               if pt[j1] > pt[j0] else 0.0)

        p = cl[np.all(np.isfinite(cl), axis=1)]
        if not len(p):
            continue
        cy, sy = math.cos(yaw), math.sin(yaw)
        R_wb = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        origin = R_wb @ LIDAR + np.array([vx, vy, 0.0])
        pb = (R_ls @ p.T).T + LIDAR                               # base_link
        pw = (R_wb @ pb.T).T + np.array([vx, vy, 0.0])          # world endpoints
        d = pw - origin
        r = np.linalg.norm(d, axis=1)
        u = d / r[:, None]

        # Free: samples along each beam, ending MARGIN before the return.
        samp = origin[None, None, :] + T[None, :, None] * u[:, None, :]
        ok = T[None, :] < (r[:, None] - MARGIN)
        q = samp[ok]
        ix = ((q[:, 0] - X0) / VOX).astype(int)
        iy = ((q[:, 1] - Y0) / VOX).astype(int)
        iz = ((q[:, 2] - Z_LO) / VOX).astype(int)
        inb = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & (iz >= 0) & (iz < nz) & (q[:, 2] >= Z_LO)
        free[ix[inb], iy[inb], iz[inb]] = True

        # Endpoints: floor, wall, or a hit.
        ex = ((pw[:, 0] - X0) / VOX).astype(int)
        ey = ((pw[:, 1] - Y0) / VOX).astype(int)
        ez = ((pw[:, 2] - Z_LO) / VOX).astype(int)
        inb = (ex >= 0) & (ex < nx) & (ey >= 0) & (ey < ny)
        is_floor = pw[:, 2] < FLOOR_Z
        is_wall = np.abs(pw[:, 1]) > 4.0 - WALL_MARGIN
        f = inb & is_floor & ~is_wall
        floor[ex[f], ey[f]] = True
        h = inb & ~is_floor & ~is_wall & (ez >= 0) & (ez < nz)
        np.add.at(hits, (ex[h], ey[h], ez[h]), 1)

        # Drivable cells: envelope bins above the floor bin all free, no hits,
        # and floor evidenced (a floor return, or a pass through the floor bin).
        b0 = int((vx + BUMPER - X0) / VOX)
        b1 = min(nx, b0 + 121)
        sub_hits = hits[b0:b1]
        strict = free[b0:b1].copy()
        strict[:, :, 0] |= floor[b0:b1]          # a floor return verifies the floor voxel

        # Proximity zone (Aayush, 15 Nov 2026): ZONE m around the body is
        # verified by the vehicle's other safety sensors at all times, and is
        # granted only while the LiDAR has no occupied voxel inside it.
        zx1 = min(nx, int(math.ceil((vx + BUMPER + ZONE - X0) / VOX)))
        zx0 = max(0, int((vx + REAR - ZONE - X0) / VOX))
        zone_ok = not (hits[zx0:zx1] > 0).any()
        if zone_ok:
            k1 = min(b1, zx1) - b0
            if k1 > 0:
                strict[:k1, :, 0] = True
        zone_granted.append(zone_ok)

        # No floating objects (Aayush, 15 Nov 2026): in a column, the voxels
        # between the lowest and the highest verified free voxel are taken as
        # free too, provided no beam ended anywhere in that span. A hanging
        # object that was hit blocks the fill; one never hit would not.
        anyv = strict.any(axis=2)
        lo_i = np.argmax(strict, axis=2)
        hi_i = nz - 1 - np.argmax(strict[:, :, ::-1], axis=2)
        zi = np.arange(nz)[None, None, :]
        span = anyv[:, :, None] & (zi >= lo_i[:, :, None]) & (zi <= hi_i[:, :, None])
        span_clear = ~((sub_hits > 0) & span).any(axis=2)
        filled = strict | (span & span_clear[:, :, None])

        def lane_horizon(ver, ly, tally):
            y_lo = max(0, int(math.floor((ly - HALF_W - Y0) / VOX)))
            y_hi = min(ny, int(math.ceil((ly + HALF_W - Y0) / VOX)))
            ok = ver[:, y_lo:y_hi, :].all(axis=2) & (sub_hits[:, y_lo:y_hi, :].sum(axis=2) == 0)
            col_ok = ok.all(axis=1)
            bad = np.where(~col_ok)[0]
            if len(bad) and tally is not None:
                lacking = (~ver[bad[0], y_lo:y_hi, :]) | (sub_hits[bad[0], y_lo:y_hi, :] > 0)
                for jz in np.where(lacking.any(axis=0))[0]:
                    z = Z_LO + (jz + 0.5) * VOX
                    tally[z] = tally.get(z, 0) + 1
            return (bad[0] if len(bad) else len(col_ok)) * VOX

        # The fill taken to its end: with no floating objects, anything in a
        # column stands on its floor, so a verified floor voxel and no hit in
        # the column means nothing stands there at any height.
        ground = np.repeat(strict[:, :, :1], nz, axis=2)

        hz = [lane_horizon(strict, ly, limit_z) for ly in a.lane]
        hz += [lane_horizon(filled, ly, limit_zf) for ly in a.lane]
        hz += [lane_horizon(ground, ly, None) for ly in a.lane]
        rows.append((vx, spd, hz))

        if spd > MOVING and b1 - b0 == 121:
            # profile over the full corridor width minus wall bands
            band = slice(int((Y0 + 1.0 - Y0) / VOX), int((Y1 - 1.0 - Y0) / VOX))
            prof_seen += free[b0:b1][:, band, :].mean(axis=(1, 2))
            prof_n += 1

    # Voxels whose CENTRE lies inside the true box: edge voxels that only
    # partly overlap the box are legitimately seen free beside it.
    xc = X0 + (np.arange(nx) + 0.5) * VOX
    yc = Y0 + (np.arange(ny) + 0.5) * VOX
    zc = Z_LO + (np.arange(nz) + 0.5) * VOX
    inbox = ((np.abs(xc - bx) < bd / 2)[:, None, None] & (np.abs(yc - by) < bw / 2)[None, :, None]
             & (zc < bh)[None, None, :])
    print(f"voxels marked free whose centre is inside the true box: {int((free & inbox).sum())} "
          f"of {int(inbox.sum())} (must be 0); box voxels ever hit: {int(((hits > 0) & inbox).sum())}")
    print(f"proximity zone granted on {sum(zone_granted)} of {len(zone_granted)} scans")
    for name, tally in (("strict", limit_z), ("with the no floating objects fill", limit_zf)):
        print(f"heights of the voxels that ended a lane's horizon, {name} (m: count):")
        zs = sorted(tally.items())
        print("  " + ", ".join(f"{z:.3f}: {n}" for z, n in zs) if zs else "  none")

    arr = np.array([[r[0], r[1]] + r[2] for r in rows])
    nl = len(a.lane)
    for title, off in (("strict: every envelope voxel seen free", 0),
                       ("with the no floating objects fill", nl),
                       ("floor verified and no hit in the column", 2 * nl)):
        print(f"\nknown free horizon ahead of the bumper, m, {title} "
              "(lane = vehicle width around the given y):")
        print("  vehicle x     n   " + "".join(f"| lane y {ly:+.1f}: median  p10  " for ly in a.lane))
        for lo in (10, 11, 12, 13, 14, 15, 20, 25, 30, 35, 40, 45):
            hi_ = {10: 11, 11: 12, 12: 13, 13: 14, 14: 15}.get(lo, lo + 5)
            sel = (arr[:, 0] >= lo) & (arr[:, 0] < hi_) & (arr[:, 1] > MOVING)
            if not sel.any():
                continue
            line = f"  {lo:3d} to {hi_:3d}  {int(sel.sum()):4d}  "
            for j in range(nl):
                v = arr[sel, 2 + off + j]
                line += f"|            {np.median(v):6.2f} {np.percentile(v, 10):5.2f}  "
            print(line + " (moving)")
        sel = arr[:, 1] <= MOVING
        if sel.any():
            line = f"  stopped    {int(sel.sum()):4d}  "
            for j in range(nl):
                v = arr[sel, 2 + off + j]
                line += f"|            {np.median(v):6.2f} {np.percentile(v, 10):5.2f}  "
            print(line + f" (at x {arr[sel, 0].mean():.1f})")

    if prof_n:
        prof = prof_seen / prof_n
        print("\nshare of envelope voxels seen free, by distance ahead of the bumper "
              "(moving scans, corridor minus 1 m wall bands):")
        for dd in (0, 2, 4, 6, 8, 10, 12, 15, 20, 25, 30):
            kk = min(int(dd / VOX), len(prof) - 1)
            print(f"  {dd:3d} m   {prof[kk]:.3f}")


if __name__ == "__main__":
    main()
