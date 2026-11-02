#!/usr/bin/env python3
"""Every LiDAR beam in an empty corridor, against its exact true range.

An empty straight corridor is four planes: two walls, the floor and the
ceiling. For each returned point, the beam direction is the point's own unit
vector in the sensor frame; the true range is where that ray first meets a
plane. The measured range minus the true range is the beam's error.

Any (azimuth, ring) cell whose error exceeds TOL in a large share of scans is
a wrong column or a wrong beam, wherever it lands. This catches what a
"beyond the wall" test cannot: a corrupted return that lands on the floor or
inside the corridor.

Assumptions, all from the locked parameters:
  - sensor at base_link (4.2, 0, 1.0), pitched 12 degrees nose down about y
    (URDF rpy 0 0.20944 0: a forward beam tips toward -z)
  - floor z = 0 and ceiling z = 8.0 in base_link
  - walls at base_link y = -lateral_offset +- corridor_width / 2, heading
    taken as zero (measured under 0.001 rad)
  - range noise 3 cm, so TOL = 0.15 m is five sigma

Scans are subsampled (every STEP-th) for speed.

Usage:
  python3 analyse_column_errors.py bags/NAME [AZ_STEP_DEG]
  AZ_STEP_DEG is the horizontal sample step, default 0.5 (361 samples over
  180 degrees). For 360 samples pass 0.501393 (180/359), or azimuth bins and
  samples drift apart and every bin mixes two beams.
"""
import sys
import math
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

CLOUD = "/scan/points"
GEOM = "/corridor_geometry"
LIDAR = np.array([4.2, 0.0, 1.0])
PITCH = 0.20944
CEILING = 8.0
TOL = 0.15
# Returns meeting their surface at a shallower angle than this are skipped.
# A beam meeting the floor at angle a from height h travels r = h / sin(a);
# its range moves by r / tan(a) per radian of vehicle pitch, about 0.9 m per
# 0.1 degree for the far floor rings. The model assumes a level vehicle, so
# those beams test the model, not the sensor. At 10 degrees and 6 m the same
# 0.1 degree costs about 3 cm, inside TOL.
GRAZE_MIN = 10.0
STEP = 10
BAD_SHARE = 0.5          # a cell is reported if it is wrong in at least this share of scans


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


def cloud_xyzr(c):
    raw = point_cloud2.read_points(c, field_names=["x", "y", "z", "ring"], skip_nans=True)
    try:
        a = np.column_stack([np.asarray(raw["x"], float), np.asarray(raw["y"], float),
                             np.asarray(raw["z"], float), np.asarray(raw["ring"], float)])
    except (TypeError, IndexError, ValueError):
        a = np.array([tuple(float(v) for v in p) for p in raw])
    if a.size == 0:
        return a.reshape(0, 4)
    return a[np.all(np.isfinite(a[:, :3]), axis=1)]


def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__)
        sys.exit(1)
    az_step = float(sys.argv[2]) if len(sys.argv) == 3 else 0.5
    data = read_sorted(sys.argv[1], [CLOUD, GEOM])
    geo = data[GEOM]
    gt = np.array([s for s, _ in geo])

    c, s = math.cos(PITCH), math.sin(PITCH)
    R = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])   # sensor to base_link

    seen = defaultdict(int)
    bad = defaultdict(int)
    errs = defaultdict(list)
    surf = defaultdict(lambda: defaultdict(int))
    skipped = 0
    SURFACES = ("left wall", "right wall", "floor", "ceiling")
    scans = 0
    floor_resid = []

    for k, (st, cl) in enumerate(data[CLOUD]):
        if k % STEP:
            continue
        i = int(np.clip(np.searchsorted(gt, st), 1, len(gt) - 1))
        if abs(gt[i - 1] - st) < abs(gt[i] - st):
            i -= 1
        g = geo[i][1]
        if not g.valid:
            continue
        scans += 1
        centre = -g.lateral_offset
        y_left, y_right = centre + 0.5 * g.corridor_width, centre - 0.5 * g.corridor_width

        a = cloud_xyzr(cl)
        if not len(a):
            continue
        p, ring = a[:, :3], a[:, 3].astype(int)
        rng = np.linalg.norm(p, axis=1)
        ok = rng > 1e-6
        p, ring, rng = p[ok], ring[ok], rng[ok]
        u = (R @ (p / rng[:, None]).T).T          # beam directions in base_link

        # Distance along each ray to each plane; keep the nearest positive.
        t = np.full((len(u), 4), np.inf)
        with np.errstate(divide="ignore", invalid="ignore"):
            for j, (axis, val) in enumerate(((1, y_left), (1, y_right), (2, 0.0), (2, CEILING))):
                tt = (val - LIDAR[axis]) / u[:, axis]
                t[:, j] = np.where(tt > 0, tt, np.inf)
        true = t.min(axis=1)
        hit = np.argmin(t, axis=1)
        err = rng - true

        # Sanity check on the mount model: floor hits should land at z = 0.
        pb = (R @ p.T).T + LIDAR
        fsel = (hit == 2) & (np.abs(err) < TOL)
        if fsel.any():
            floor_resid.append(np.median(pb[fsel, 2]))

        # Angle between each beam and the surface it hits: asin of the beam
        # component along that surface's normal (y for walls, z otherwise).
        normal_comp = np.where(hit <= 1, np.abs(u[:, 1]), np.abs(u[:, 2]))
        graze = np.degrees(np.arcsin(np.clip(normal_comp, 0.0, 1.0)))
        use = graze >= GRAZE_MIN
        skipped += int((~use).sum())

        # One bin per horizontal sample, centred on the samples themselves.
        # Samples sit at -90 + k * step, so bin from -90, not from 0: with 360
        # samples they fall on half steps from 0, and bins counted from 0
        # would put every sample on a bin edge, where noise flips it.
        az_deg = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
        az = np.round((az_deg + 90.0) / az_step) * az_step - 90.0
        for a_, r_, e_, h_ in zip(az[use], ring[use], err[use], hit[use]):
            key = (float(a_), int(r_))
            seen[key] += 1
            if abs(e_) > TOL:
                bad[key] += 1
                errs[key].append(e_)
                surf[float(a_)][int(h_)] += 1

    print(f"bag: {sys.argv[1]}, scans checked: {scans} (every {STEP}th), "
          f"azimuth step {az_step} deg")
    if floor_resid:
        print(f"mount check, median floor height of clean floor hits: "
              f"{np.median(floor_resid):+.3f} m (should be near 0)")
    total = sum(seen.values())
    nbad = sum(bad.values())
    print(f"returns skipped as grazing (under {GRAZE_MIN:.0f} deg to their surface): {skipped}")
    print(f"beam returns checked: {total}, over {TOL} m from truth: {nbad} "
          f"({100.0 * nbad / max(total, 1):.3f} %)\n")

    flagged = sorted(k for k in seen if bad[k] >= BAD_SHARE * seen[k])
    cols = defaultdict(list)
    for a_, r_ in flagged:
        cols[a_].append(r_)
    print(f"(azimuth, ring) cells wrong in at least {int(BAD_SHARE * 100)} % of scans: "
          f"{len(flagged)}, in {len(cols)} azimuth columns")
    for a_ in sorted(cols):
        rs = cols[a_]
        e = np.concatenate([errs[(a_, r)] for r in rs])
        where = max(surf[a_].items(), key=lambda kv: kv[1])[0]
        print(f"  azimuth {a_:+7.1f} deg: rings {min(rs)} to {max(rs)} ({len(rs)}), "
              f"error median {np.median(e):+.3f} m, min {e.min():+.3f}, max {e.max():+.3f}, "
              f"truth surface mostly {SURFACES[where]}")

    stray = nbad - sum(bad[k] for k in flagged)
    print(f"\nwrong returns outside the flagged cells (sporadic): {stray}")


if __name__ == "__main__":
    main()
