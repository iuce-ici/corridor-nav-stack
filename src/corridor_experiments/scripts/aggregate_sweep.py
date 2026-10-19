#!/usr/bin/env python3
"""
Phase 2, part 8: summarise the seed sweep.
 
Each bag is one noise realisation of the same 100 m corridor run. This reports
one row per seed and then the spread across seeds, which is what a headline
figure needs: a single run shows what happened once, not what the filter does.
 
Reuses analyse_ekf_update for reading and sorting, so both scripts share one
definition of publish order and of the motion window.
 
Usage, from ~/ros2_ws:
  python3 src/corridor_experiments/scripts/aggregate_sweep.py bags/ekf_sweep_s*
"""
 
import math
import os
import sys
 
import numpy as np
 
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyse_ekf_update import (B_TRUE, V_MOVING, read_bag,  # noqa: E402
                                sorted_rows, stamp_s, yaw)
 
 
def one_bag(path):
    """Key figures for one run. Returns None if the run is not usable."""
    b = read_bag(path)
    import io
    import contextlib
    with contextlib.redirect_stdout(io.StringIO()):   # sorted_rows prints
        st = sorted_rows([[stamp_s(m.header), *m.state, *m.covariance]
                          for m in b['/ekf/state']], 'state')
        up = sorted_rows([[stamp_s(m.header), *m.innovation, *m.innovation_covariance,
                           *m.state_after, m.stamp_gap] for m in b['/ekf/update']], 'update')
        geo = sorted_rows([[stamp_s(m.header), m.lateral_offset, m.heading_error,
                            float(m.valid)] for m in b['/corridor_geometry']], 'geo')
        gt = sorted_rows([[stamp_s(m.header), m.pose.position.x, m.pose.position.y,
                           yaw(m.pose.orientation)] for m in b['/model/vehicle/pose']], 'gt')
        dr = sorted_rows([[stamp_s(m.header), m.pose.pose.position.y,
                           yaw(m.pose.pose.orientation)] for m in b['/odom_dr']], 'dr')
    if len(st) == 0 or len(up) == 0:
        return None
 
    # Duplicate timestamps mean two publishers were alive at once, usually a
    # node left over from an earlier run. The bag is then a mixture of two
    # filters and nothing computed from it is meaningful.
    uniq = np.unique(st[:, 0]).size
    if uniq < len(st):
        raise ValueError(
            f'{len(st) - uniq} of {len(st)} /ekf/state messages share a timestamp '
            'with another: more than one ekf node was publishing. Re-record this run '
            'with nothing else running (ros2 node list must be empty before the launch)')
 
    t = st[:, 0]
    x, y, th, bb = st[:, 1], st[:, 2], st[:, 3], st[:, 4]
    P = st[:, 5:21].reshape(-1, 4, 4)
    gx = np.interp(t, gt[:, 0], gt[:, 1])
    gy = np.interp(t, gt[:, 0], gt[:, 2])
    gyaw = np.interp(t, gt[:, 0], np.unwrap(gt[:, 3]))
 
    moving = np.gradient(gx, t) > V_MOVING
    if not moving.any():
        return None
    i0, i1 = np.nonzero(moving)[0][[0, -1]]
    w = slice(i0, i1 + 1)
 
    valid = geo[geo[:, 3] > 0.5]
    x0 = float(np.interp(valid[0, 0], gt[:, 0], gt[:, 1]))
    d_true = gx[-1] - x0
 
    ey, eth = y - gy, th - gyaw
    ts = t[w]
    dr_y = np.interp(ts, dr[:, 0], dr[:, 1]) - np.interp(ts, gt[:, 0], gt[:, 2])
 
    innov = up[:, 1:3]
    S = up[:, 3:7].reshape(-1, 2, 2)
    nis = np.array([v @ np.linalg.solve(Sk, v) for v, Sk in zip(innov, S)])
 
    return {
        'n_updates': len(up),
        'lat_rmse': float(np.sqrt(np.mean(ey[w] ** 2))),
        'lat_max': float(np.max(np.abs(ey[w]))),
        'head_rmse': float(np.sqrt(np.mean(eth[w] ** 2))),
        'head_max': float(np.max(np.abs(eth[w]))),
        # The simulator negates the gyro bias sign depending on the seed, so a
        # run is compared against whichever sign it actually has. The sign is
        # reported alongside, and a b_hat near zero would show as a large error
        # against either sign rather than being hidden by this.
        'bias_sign': '+' if bb[-1] >= 0 else '-',
        'bias_err_pct': 100.0 * (bb[-1] - math.copysign(B_TRUE, bb[-1])) / B_TRUE,
        'bias_sigma': float(abs(bb[-1] - math.copysign(B_TRUE, bb[-1]))
                            / math.sqrt(P[-1, 3, 3])),
        'nis': float(np.mean(nis)),
        'x_err': float(x[-1] - d_true),
        'dist': float(d_true),
        'dr_lat_max': float(np.max(np.abs(dr_y))),
    }
 
 
def main():
    paths = [p for p in sys.argv[1:] if os.path.isdir(p)]
    if not paths:
        sys.exit('usage: aggregate_sweep.py bags/ekf_sweep_s*')
 
    rows = []
    for p in sorted(paths):
        try:
            r = one_bag(p)
        except Exception as exc:                     # a half written bag, say
            print(f'{os.path.basename(p):<24} unreadable: {exc}')
            continue
        if r is None:
            print(f'{os.path.basename(p):<24} no usable run')
            continue
        r['name'] = os.path.basename(p)
        rows.append(r)
 
    if not rows:
        sys.exit('no usable runs')
 
    hdr = (f"{'run':<22} {'upd':>5} {'lat RMSE':>9} {'lat max':>8} {'hdg RMSE':>9}"
           f" {'hdg max':>8} {'b':>2} {'bias':>8} {'sig':>5} {'NIS':>6} {'x err':>7}"
           f" {'DR max':>8}")
    print(f'\n{hdr}')
    print(f"{'':<22} {'':>5} {'mm':>9} {'mm':>8} {'mrad':>9} {'mrad':>8} {'sgn':>2}"
          f" {'pct':>8} {'':>5} {'':>6} {'m':>7} {'mm':>8}")
    print('-' * len(hdr))
    for r in rows:
        print(f"{r['name']:<22} {r['n_updates']:5d} {r['lat_rmse'] * 1e3:9.4f}"
              f" {r['lat_max'] * 1e3:8.4f} {r['head_rmse'] * 1e3:9.5f}"
              f" {r['head_max'] * 1e3:8.5f} {r['bias_sign']:>2}"
              f" {r['bias_err_pct']:+8.2f} {r['bias_sigma']:5.2f}"
              f" {r['nis']:6.3f} {r['x_err']:+7.3f} {r['dr_lat_max'] * 1e3:8.1f}")
 
    print(f'\nAcross {len(rows)} seeds: mean, standard deviation, worst')
    fields = [('lateral RMSE', 'lat_rmse', 1e3, 'mm', 4),
              ('lateral max', 'lat_max', 1e3, 'mm', 4),
              ('heading RMSE', 'head_rmse', 1e3, 'mrad', 5),
              ('heading max', 'head_max', 1e3, 'mrad', 5),
              ('bias error', 'bias_err_pct', 1.0, 'percent', 2),
              ('bias, sigma from P', 'bias_sigma', 1.0, '', 2),
              ('mean NIS', 'nis', 1.0, '', 3),
              ('x error', 'x_err', 1.0, 'm', 3),
              ('dead reckoning max', 'dr_lat_max', 1e3, 'mm', 1)]
    for label, key, scale, unit, dp in fields:
        v = np.array([r[key] for r in rows]) * scale
        worst = v[np.argmax(np.abs(v))]
        print(f'  {label:<20} mean {np.mean(v):+.{dp}f}   sd {np.std(v):.{dp}f}   '
              f'worst {worst:+.{dp}f} {unit}')
 
    nis = np.array([r['nis'] for r in rows])
    print(f'\n  NIS across seeds: {np.min(nis):.3f} to {np.max(nis):.3f} '
          '(2 if the filter is consistent)')
    d = np.array([r['dist'] for r in rows])
    print(f'  distance travelled: {np.min(d):.2f} to {np.max(d):.2f} m')
    signs = [r['bias_sign'] for r in rows]
    print(f"  gyro bias sign: {signs.count('+')} positive, {signs.count('-')} negative "
          '(the simulator negates it by seed; each run is scored against its own sign)')
 
 
if __name__ == '__main__':
    main()
 