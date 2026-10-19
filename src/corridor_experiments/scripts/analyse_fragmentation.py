#!/usr/bin/env python3
"""
Diagnostic for the slalom tracking failure: why one obstacle produces more
than one cluster.
 
Not a criterion script. It answers a single question: on scans where the
obstacle is in range, how many clusters does the detector report near it, and
what do they look like.
 
Three hypotheses it distinguishes:
 
  1. Wall margin clipping. The box is 1.2 m wide, so an obstacle at y 0.8
     spans y 0.2 to 1.4. If the corridor half width minus wall_margin is below
     1.4 the outer slice is discarded as wall points, which can leave two
     disconnected groups. Shown by the 'inner limit' column against the
     obstacle's far edge, and by clusters whose y extent is cut short on the
     wall side.
 
  2. Range sparsity. At 25 to 30 m the obstacle returns a handful of points on
     one or two layers, and if the azimuth gaps exceed cell_size the grid
     cannot connect them. Shown by fragmentation that appears only in the far
     bands and disappears as the obstacle closes.
 
  3. Something else, in which case the per-scan dump at the end is the place
     to look.
 
The distinction matters for the fix. Clipping is a geometry bug and is fixed
by comparing the obstacle against the wall rather than discarding a fixed
margin. Sparsity is a clustering parameter question, fixed by a range
dependent cell size or by connecting cells two apart at long range. Guessing
between them would mean tuning a parameter to hide a geometry bug.
 
Usage, from ~/ros2_ws:
  python3 analyse_fragmentation.py bags/track_slalom \\
      src/corridor_sim/worlds/corridor_slalom.obstacles.json
"""
 
import json
import math
import sys
 
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
 
NEAR = 3.0                # m, a cluster this close to the truth centre is part of it
WALL_MARGIN = 0.30        # m, must match the detector parameter
BANDS = [(0, 5), (5, 10), (10, 15), (15, 20), (20, 25), (25, 30), (30, 35)]
 
WANTED = ['/obstacles', '/corridor_geometry', '/model/vehicle/pose']
 
 
def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9
 
 
def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
 
 
def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {t: get_message(types[t]) for t in WANTED if t in types}
    out = {t: [] for t in WANTED}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in classes:
            out[topic].append(deserialize_message(data, classes[topic]))
    return out
 
 
def truth_in_base_link(ox, oy, size, px, py, yaw):
    sx, sy = size[0] / 2.0, size[1] / 2.0
    c, s = math.cos(-yaw), math.sin(-yaw)
    xs, ys = [], []
    for dx in (-sx, sx):
        for dy in (-sy, sy):
            wx, wy = ox + dx - px, oy + dy - py
            xs.append(c * wx - s * wy)
            ys.append(s * wx + c * wy)
    return {'min_x': min(xs), 'x': 0.5 * (min(xs) + max(xs)),
            'y': 0.5 * (min(ys) + max(ys)),
            'y_min': min(ys), 'y_max': max(ys)}
 
 
def main():
    if len(sys.argv) != 3:
        sys.exit('usage: analyse_fragmentation.py <bag directory> <obstacles json>')
    bag_path, json_path = sys.argv[1], sys.argv[2]
 
    scenario = json.load(open(json_path))
    size = scenario['size']
    truth_world = [(o['x'], o['y']) for o in scenario['obstacles']]
 
    b = read_bag(bag_path)
    gt = np.array([[stamp_s(m.header), m.pose.position.x, m.pose.position.y,
                    yaw_of(m.pose.orientation)] for m in b['/model/vehicle/pose']])
    gt = gt[np.argsort(gt[:, 0], kind='stable')]
 
    geo = sorted(b['/corridor_geometry'], key=lambda m: stamp_s(m.header))
    widths = np.array([m.corridor_width for m in geo if m.valid])
    print(f'Bag: {bag_path}')
    if len(widths):
        hw = float(np.mean(widths)) / 2.0
        inner = hw - WALL_MARGIN
        print(f'Corridor width {np.mean(widths):.3f} m, half width {hw:.3f} m, '
              f'inner limit after {WALL_MARGIN:.2f} m margin: {inner:+.3f} m')
        for k, (ox, oy) in enumerate(truth_world):
            far = abs(oy) + size[1] / 2.0
            flag = 'CLIPPED' if far > inner else 'clear'
            print(f'  obstacle {k} at y {oy:+.2f} m spans to {far:.2f} m '
                  f'from the centreline: {flag}')
    else:
        inner = None
        print('no valid corridor geometry in this bag')
 
    msgs = sorted(b['/obstacles'], key=lambda m: stamp_s(m.header))
    print(f'{len(msgs)} obstacle messages\n')
 
    rows = []          # range, n_clusters_near, points in the largest, y extent
    dump = []
    for m in msgs:
        t = stamp_s(m.header)
        px = float(np.interp(t, gt[:, 0], gt[:, 1]))
        py = float(np.interp(t, gt[:, 0], gt[:, 2]))
        yaw = float(np.interp(t, gt[:, 0], np.unwrap(gt[:, 3])))
 
        for k, (ox, oy) in enumerate(truth_world):
            tr = truth_in_base_link(ox, oy, size, px, py, yaw)
            if not (0.0 < tr['min_x'] < 35.0):
                continue
            near = [ob for ob in m.obstacles
                    if math.hypot(ob.x - tr['x'], ob.y - tr['y']) < NEAR]
            if not near:
                rows.append((k, tr['min_x'], 0, 0, 0.0, 0.0))
                continue
            big = max(near, key=lambda ob: ob.n_points)
            total = sum(ob.n_points for ob in near)
            rows.append((k, tr['min_x'], len(near), big.n_points,
                         big.width, float(total)))
            if len(near) > 1 and len(dump) < 12:
                dump.append((tr['min_x'], k, tr['y'], tr['y_min'], tr['y_max'],
                             [(round(ob.x, 2), round(ob.y, 2), ob.n_points,
                               round(ob.width, 2), round(ob.length, 2),
                               round(ob.min_x, 2)) for ob in near]))
 
    r = np.array(rows, dtype=float)
    for k in sorted(set(r[:, 0].astype(int))):
        rk = r[r[:, 0] == k]
        print(f'obstacle {k}')
        print(f'{"band m":>12} {"scans":>6} {"split":>6} {"frac":>6} '
              f'{"clusters":>9} {"big pts":>8} {"all pts":>8} {"big width":>10}')
        print('-' * 72)
        for lo, hi in BANDS:
            sel = rk[(rk[:, 1] >= lo) & (rk[:, 1] < hi)]
            if not len(sel):
                continue
            seen = sel[sel[:, 2] > 0]
            split = int((sel[:, 2] > 1).sum())
            print(f'{lo:5.0f} to {hi:4.0f} {len(sel):6d} {split:6d} '
                  f'{split / len(sel):6.2f} '
                  f'{np.mean(seen[:, 2]) if len(seen) else float("nan"):9.2f} '
                  f'{np.mean(seen[:, 3]) if len(seen) else float("nan"):8.1f} '
                  f'{np.mean(seen[:, 5]) if len(seen) else float("nan"):8.1f} '
                  f'{np.mean(seen[:, 4]) if len(seen) else float("nan"):10.2f}')
        print()
 
    if dump:
        print('First split scans, truth box against the clusters reported:')
        for min_x, k, ty, ymin, ymax, obs in dump:
            print(f'  obstacle {k} near face {min_x:6.2f} m, truth y {ty:+.2f} '
                  f'(spans {ymin:+.2f} to {ymax:+.2f})')
            # length and min_x matter as much as width: a fragment whose length
            # is also near zero is a vertical column at one azimuth, and the
            # distance from its min_x back to the main cluster is what the
            # merge threshold has to cover.
            for x, y, n, w, ln, mx in sorted(obs, key=lambda o: o[0]):
                print(f'      cluster x {x:+7.2f} y {y:+6.2f} '
                      f'{n:4d} points  width {w:.2f}  length {ln:.2f}  '
                      f'near face {mx:+.2f} m')
    else:
        print('No scan reported more than one cluster near an obstacle.')
 
    print('\nRead it this way: a split fraction that is high in the far bands '
          'and falls to zero as the obstacle closes is range sparsity. A split '
          'fraction that persists at close range, with the clusters separated '
          'in y and the truth box marked CLIPPED above, is wall margin '
          'clipping.')
    return 0
 
 
if __name__ == '__main__':
    sys.exit(main())
 