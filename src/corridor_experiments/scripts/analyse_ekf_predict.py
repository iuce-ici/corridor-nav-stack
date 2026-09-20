#!/usr/bin/env python3
"""
Phase 2, step 2: check the C++ EKF node, predict only, against a recorded run.
 
Reads one bag recorded with
  /odom_dr /odom_ekf /ekf/covariance /imu /model/vehicle/pose /clock
 
Section 0, start alignment (diagnostic, no verdict)
  Message counts, first stamps, stationary time before motion, and how long
  the EKF had been integrating at its first recorded message, read from its
  own covariance. sigma_theta grows as sigma_b * t, so P works as a clock.
  That separates two explanations for a count mismatch: the EKF started
  late, or the recorder missed its first messages.
 
Check 1, plumbing
  Pair /odom_ekf with /odom_dr by header stamp.
  Heading identical within 1e-12 rad at every pair.
  Position within 5 mm at every pair.
 
Check 2, the maths in C++
  Replay P in Python from the node's own inputs: speed and yaw rate from the
  published twist, heading from the previous published pose, dt from stamps.
  Starts from the first recorded P, so it works whatever happened at start.
  Every entry within 1e-6, scaled by its own standard deviations.
 
Check 3, sanity (no verdict, one run)
  Final claimed sigma_y and sigma_theta against the actual errors from
  ground truth.
 
Usage
  python3 analyse_ekf_predict.py bags/ekf_predict_100m_b
"""
 
import math
import sys
 
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
 
SIGMA_W = 1.45e-3        # rad/s per sample, as believed by the node
SIGMA_B = 8.7266e-4      # rad/s, bias prior, as believed by the node
V_MOVING = 0.01          # m/s, the vehicle counts as moving above this
 
HEADING_TOL = 1e-12      # rad, check 1
POS_TOL = 0.005          # m, check 1
P_TOL = 1e-6             # scaled, check 2
 
WANTED = ['/odom_dr', '/odom_ekf', '/ekf/covariance', '/imu', '/model/vehicle/pose']
 
 
def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
 
 
def yaw_planar(q):
    # The nodes write z = sin(theta/2), w = cos(theta/2). This inverts that
    # exactly, over the full range, with no asin.
    return 2.0 * math.atan2(q.z, q.w)
 
 
def yaw_general(q):
    # Ground truth quaternion may carry small roll and pitch.
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
 
 
def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in WANTED if t not in types]
    if missing:
        sys.exit(f'bag lacks topics: {missing}')
    classes = {t: get_message(types[t]) for t in WANTED}
    out = {t: [] for t in WANTED}
    while reader.has_next():
        topic, data, t_recv = reader.read_next()
        if topic in classes:
            out[topic].append((t_recv, deserialize_message(data, classes[topic])))
    return out
 
 
def odom_arrays(msgs):
    return {
        'recv': np.array([t for t, _ in msgs], dtype=np.int64),
        'st': np.array([stamp_ns(m.header) for _, m in msgs], dtype=np.int64),
        'x': np.array([m.pose.pose.position.x for _, m in msgs]),
        'y': np.array([m.pose.pose.position.y for _, m in msgs]),
        'th': np.array([yaw_planar(m.pose.pose.orientation) for _, m in msgs]),
        'v': np.array([m.twist.twist.linear.x for _, m in msgs]),
        'w': np.array([m.twist.twist.angular.z for _, m in msgs]),
    }
 
 
def jacobians(th, v, w, dt):
    th_mid = th + 0.5 * w * dt
    s, c = math.sin(th_mid), math.cos(th_mid)
    F = np.eye(4)
    F[0, 2] = -v * s * dt
    F[0, 3] = 0.5 * v * s * dt * dt
    F[1, 2] = v * c * dt
    F[1, 3] = -0.5 * v * c * dt * dt
    F[2, 3] = -dt
    G = np.array([-0.5 * v * s * dt * dt, 0.5 * v * c * dt * dt, dt, 0.0])
    return F, G
 
 
def out_of_order(st):
    """Intervals that go backwards or stand still, in the order the bag returned them."""
    return int(np.sum(np.diff(st) <= 0))
 
 
def sort_by_stamp(a):
    idx = np.argsort(a['st'], kind='stable')
    out = {k: v[idx] for k, v in a.items()}
    if np.any(np.diff(out['st']) <= 0):
        sys.exit('  duplicate header stamps after sorting: a node published one stamp twice. Stop here.')
    return out
 
 
def load_ordered(b):
    """Put every topic back into the order its node published it.
 
    The bag returns messages in recorder receive order, which is not publish
    order: the nodes can never publish a stamp that goes backwards, yet the
    bag contains such steps. So never trust bag order.
      odom topics:      sort by header stamp, which is sim time.
      /ekf/covariance:  has no header. Sort by the node's own clock instead:
                        P_theta_b falls by dt * P_bb every step and P_bb is
                        constant, so -P_theta_b / P_bb is the node's summed dt,
                        which rises strictly with every step.
    Then verify the pairing: stamp minus P clock must be the same constant
    for every pair."""
    imu_st = np.array([stamp_ns(m.header) for _, m in b['/imu']], dtype=np.int64)
    dr_raw = odom_arrays(b['/odom_dr'])
    ek_raw = odom_arrays(b['/odom_ekf'])
    P_raw = np.array([np.array(m.data).reshape(4, 4) for _, m in b['/ekf/covariance']])
    clock_raw = -P_raw[:, 2, 3] / P_raw[:, 3, 3]
 
    print('\nBag order (intervals going backwards, as the bag returned them)')
    print(f'  imu {out_of_order(imu_st)}   odom_dr {out_of_order(dr_raw["st"])}   '
          f'odom_ekf {out_of_order(ek_raw["st"])}   ekf/covariance {out_of_order(clock_raw)}'
          '   -> all sorted back into publish order below')
 
    dr = sort_by_stamp(dr_raw)
    ek = sort_by_stamp(ek_raw)
    order = np.argsort(clock_raw, kind='stable')
    P, clock = P_raw[order], clock_raw[order]
    if np.any(np.diff(clock) <= 0):
        sys.exit('  covariance clock does not rise strictly after sorting. Stop here.')
 
    if len(P) != len(ek['st']):
        sys.exit(f'  counts differ after sorting: odom_ekf {len(ek["st"])}, '
                 f'covariance {len(P)}. Cannot pair. Stop here.')
    offset = (ek['st'] - ek['st'][0]) / 1e9 - (clock - clock[0])
    worst = float(np.max(np.abs(offset)))
    print(f'  covariance to odom_ekf pairing: stamp against P clock, worst mismatch '
          f'{worst * 1e6:.3f} us')
    if worst > 1e-6:
        sys.exit('  pairing fails: the two topics do not describe the same steps. Stop here.')
    return dr, ek, P
 
 
def verdict(ok):
    return 'PASS' if ok else 'FAIL'
 
 
def main():
    if len(sys.argv) != 2:
        sys.exit('usage: analyse_ekf_predict.py <bag directory>')
    path = sys.argv[1]
    b = read_bag(path)
 
    print(f'Bag: {path}')
    dr, ek, P = load_ordered(b)
 
    # Section 0: start alignment
    print('\nSection 0: start alignment (diagnostic)')
    print(f'  counts   imu {len(b["/imu"])}   odom_dr {len(dr["st"])}   '
          f'odom_ekf {len(ek["st"])}   ekf/covariance {len(P)}')
 
 
    dt_nom = float(np.median(np.diff(ek['st']))) / 1e9
    lead = (ek['st'][0] - dr['st'][0]) / 1e9
    print(f'  first odom_ekf stamp is {lead:+.3f} s after the first odom_dr stamp '
          f'({lead / dt_nom:+.0f} samples at {1.0 / dt_nom:.1f} Hz)')
 
    moving = np.nonzero(dr['v'] > V_MOVING)[0]
    if len(moving):
        t_pre = (dr['st'][moving[0]] - dr['st'][0]) / 1e9
        print(f'  stationary time before motion, from odom_dr: {t_pre:.3f} s')
    else:
        t_pre = None
        print('  the vehicle never moved in this bag')
 
    # sigma_theta^2 = sigma_b^2 t^2 + sigma_w^2 dt t   ->   solve for t
    A, B, C = SIGMA_B ** 2, SIGMA_W ** 2 * dt_nom, -P[0, 2, 2]
    t_int = (-B + math.sqrt(B * B - 4 * A * C)) / (2 * A)
    t_dr = lead + dt_nom
    print(f'  EKF integration time at its first recorded message, from its own P: {t_int:.3f} s')
    print(f'  odom_dr integration time at that same stamp:                          {t_dr:.3f} s')
    if abs(t_int - t_dr) < 0.05:
        print('  -> the EKF integrated from the same start as dead reckoning. '
              'Any missing messages were not recorded, not skipped by the node.')
    else:
        print(f'  -> the EKF started integrating about {t_dr - t_int:.3f} s after dead reckoning.')
 
    # Check 1: plumbing
    common, i_dr, i_ek = np.intersect1d(dr['st'], ek['st'], return_indices=True)
    dth = ek['th'][i_ek] - dr['th'][i_dr]
    dpos = np.hypot(ek['x'][i_ek] - dr['x'][i_dr], ek['y'][i_ek] - dr['y'][i_dr])
    ok1 = len(common) > 0 and np.max(np.abs(dth)) <= HEADING_TOL and np.max(dpos) <= POS_TOL
    print(f'\nCheck 1: EKF mean against dead reckoning   ->   {verdict(ok1)}')
    print(f'  paired stamps: {len(common)} of {len(ek["st"])} odom_ekf messages')
    if len(common):
        print(f'  heading difference   max |d| {np.max(np.abs(dth)):.3e} rad   '
              f'first {dth[0]:+.3e}   spread (max minus min) {np.ptp(dth):.3e}   '
              f'tolerance {HEADING_TOL:.0e}')
        print(f'  position difference  max {np.max(dpos) * 1e3:.3f} mm   '
              f'final {dpos[-1] * 1e3:.3f} mm   tolerance {POS_TOL * 1e3:.0f} mm')
 
    # Check 2: replay P
    dts = np.diff(ek['st']) / 1e9
    Pr = P[0].copy()
    worst, worst_at, resyncs, compared = 0.0, None, 0, 0
    names = ['x', 'y', 'theta', 'b']
    for k in range(1, len(ek['st'])):
        dt = dts[k - 1]
        if dt > 1.5 * dt_nom or dt <= 0.0:
            # A message is missing between k-1 and k, so one node step has no
            # record. Restart the replay from the node's own P at k.
            Pr = P[k].copy()
            resyncs += 1
            continue
        F, G = jacobians(ek['th'][k - 1], ek['v'][k], ek['w'][k], dt)
        Pr = F @ Pr @ F.T + SIGMA_W ** 2 * np.outer(G, G)
        Pr = 0.5 * (Pr + Pr.T)
        Pn = P[k]
        d = np.sqrt(np.outer(np.diag(Pn), np.diag(Pn)))
        mask = d > 0.0
        err = np.zeros((4, 4))
        err[mask] = np.abs(Pr - Pn)[mask] / d[mask]
        compared += 1
        if err.max() > worst:
            worst = float(err.max())
            i, j = np.unravel_index(np.argmax(err), err.shape)
            worst_at = (k, names[i], names[j])
    ok2 = compared > 0 and worst <= P_TOL
    print(f'\nCheck 2: C++ covariance against Python replay   ->   {verdict(ok2)}')
    print(f'  steps compared {compared}, resyncs over gaps {resyncs}')
    if worst_at:
        print(f'  worst scaled difference {worst:.3e} at step {worst_at[0]}, '
              f'entry ({worst_at[1]}, {worst_at[2]})   tolerance {P_TOL:.0e}')
    else:
        print(f'  worst scaled difference {worst:.3e}   tolerance {P_TOL:.0e}')
 
    # Check 3: claimed against actual
    gt = b['/model/vehicle/pose']
    g_t = np.array([stamp_ns(m.header) for _, m in gt], dtype=np.int64)
    g_x = np.array([m.pose.position.x for _, m in gt])
    g_y = np.array([m.pose.position.y for _, m in gt])
    g_h = np.unwrap(np.array([yaw_general(m.pose.orientation) for _, m in gt]))
 
    t0 = min(dr['st'][0], ek['st'][0])
    t1 = ek['st'][-1]
    x0, y0, h0 = (np.interp(t0, g_t, a) for a in (g_x, g_y, g_h))
    x1, y1, h1 = (np.interp(t1, g_t, a) for a in (g_x, g_y, g_h))
    # True displacement expressed in the frame the nodes start in.
    dx, dy = x1 - x0, y1 - y0
    tx = math.cos(h0) * dx + math.sin(h0) * dy
    ty = -math.sin(h0) * dx + math.cos(h0) * dy
    th_true = h1 - h0
 
    e_y = ty - ek['y'][-1]
    e_th = th_true - ek['th'][-1]
    s_y = math.sqrt(P[-1, 1, 1])
    s_th = math.sqrt(P[-1, 2, 2])
    print('\nCheck 3: claimed error against actual error, end of run (no verdict)')
    if t_pre is not None and t0 >= dr['st'][moving[0]]:
        print('  WARNING: reference taken after motion began; the numbers below are not meaningful')
    print(f'  distance travelled (truth) {math.hypot(tx, ty):.3f} m')
    print(f'  lateral  actual error {e_y:+.3f} m      claimed sigma_y     {s_y:.3f} m   '
          f'ratio {abs(e_y) / s_y:.2f}')
    print(f'  heading  actual error {math.degrees(e_th):+.3f} deg    claimed sigma_theta '
          f'{math.degrees(s_th):.3f} deg   ratio {abs(e_th) / s_th:.2f}')
 
    all_ok = ok1 and ok2
    print(f'\nChecks 1 and 2: {verdict(all_ok)}')
    return 0 if all_ok else 1
 
 
if __name__ == '__main__':
    sys.exit(main())