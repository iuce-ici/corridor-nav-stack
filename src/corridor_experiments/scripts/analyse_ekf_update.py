#!/usr/bin/env python3
"""
Phase 2, part 5b: the first EKF run with LiDAR updates, range noise off.
 
Reads one bag recorded with
  /ekf/state /ekf/update /corridor_geometry /odom_ekf /odom_dr /imu
  /model/vehicle/pose /clock
Every topic is sorted by header stamp first: bag order is not publish order.
 
Frames. The EKF frame is corridor aligned from its first valid scan, so its y
and theta compare directly with ground truth y and yaw (the corridor
centreline is world y = 0 along world +x). Its x origin is arbitrary, so x is
compared as distance from the pose at initialisation.
 
Exit criteria, fixed before the first run:
  C1 mechanics   updates applied / valid scans after initialisation >= 0.99
  C2 bias        |b_hat - 8.7266e-4| <= 3 sigma_b at the end, sigma_b from P
  C3 lateral     |y_hat - y_true| <= 1.0 mm throughout the motion
  C4 heading     |theta_hat - yaw_true| <= 0.2 mrad throughout the motion
  C5 x           x error at the end within 5 cm of 1 percent of the distance
                 travelled: x is unobservable, so it must drift exactly like
                 dead reckoning's scale error
 
Diagnostics, no verdict, with the expectation stated in advance:
  claimed against actual error at the end. Expected: lateral about nine times
    overconfident (a repeated 0.44 mm offset treated as independent noise);
    heading roughly matched; x hugely overconfident (radius error not in P).
  mean normalised innovation squared (NIS). A consistent filter gives 2 for a
    two element measurement. Expected well below 2, because R deliberately
    exceeds the scan to scan scatter.
  stamp gap between filter time and scan stamp, the cost of updating on arrival.
  time for the bias estimate to settle within 5 percent.
 
Usage
  python3 analyse_ekf_update.py bags/ekf_update_100m
"""
 
import math
import sys
 
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
 
B_TRUE = 8.7266e-4        # rad/s, configured gyro bias
LAT_TOL = 1.0e-3          # m, C3
HEAD_TOL = 0.2e-3         # rad, C4
X_TOL = 0.05              # m, C5
MECH_MIN = 0.99           # C1
V_MOVING = 0.1            # m/s, ground truth speed that counts as moving
 
WANTED = ['/ekf/state', '/ekf/update', '/corridor_geometry', '/odom_dr',
          '/model/vehicle/pose']
 
 
def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9
 
 
def yaw(q):
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
        topic, data, _ = reader.read_next()
        if topic in classes:
            out[topic].append(deserialize_message(data, classes[topic]))
    return out
 
 
def sorted_rows(rows, label):
    a = np.array(rows, dtype=float)
    back = int(np.sum(np.diff(a[:, 0]) <= 0))
    a = a[np.argsort(a[:, 0], kind='stable')]
    print(f'  {label:<22} {len(a):6d} messages, {back:4d} backward intervals in bag order')
    return a
 
 
def heading_excursions(geo, gt, t, eth, window):
    """Is a heading excursion in the measurement or in the filter?
 
    Compares the LiDAR's own heading error against ground truth with the
    filter's, at the same moments. If the measurement carries the excursions,
    the limit is the sensor's, not the estimator's."""
    valid = geo[geo[:, 3] > 0.5]
    t0, t1 = t[window][0], t[window][-1]
    m = (valid[:, 0] >= t0) & (valid[:, 0] <= t1)
    if m.sum() < 10:
        print('\nHeading excursions: too few valid scans during motion')
        return
    ts = valid[m, 0]
    meas_err = valid[m, 2] - np.interp(ts, gt[:, 0], np.unwrap(gt[:, 3]))
    filt_err = np.interp(ts, t, eth)
    c = float(np.corrcoef(meas_err, filt_err)[0, 1])
    print('\nHeading excursions, during motion (no verdict)')
    print(f'  measurement error  max |e| {np.max(np.abs(meas_err)) * 1e3:.4f} mrad   '
          f'mean {np.mean(meas_err) * 1e3:+.4f}   std {np.std(meas_err) * 1e3:.4f}')
    print(f'  filter error       max |e| {np.max(np.abs(filt_err)) * 1e3:.4f} mrad   '
          f'mean {np.mean(filt_err) * 1e3:+.4f}   std {np.std(filt_err) * 1e3:.4f}')
    print(f'  correlation between them {c:+.3f}   '
          f'(near 1: the filter is following the measurement)')
    k = int(np.argmax(np.abs(filt_err)))
    print(f'  worst filter error at {ts[k] - t0:.2f} s into motion '
          f'({100 * (ts[k] - t0) / (t1 - t0):.0f} percent through it): '
          f'filter {filt_err[k] * 1e3:+.4f} mrad, measurement {meas_err[k] * 1e3:+.4f} mrad')
    early = ts - t0 < 2.0
    late = t1 - ts < 2.0
    for name, sel in (('first 2 s of motion', early), ('last 2 s of motion', late),
                      ('the rest', ~(early | late))):
        if sel.sum():
            print(f'  {name:<20} n {int(sel.sum()):5d}   filter max |e| '
                  f'{np.max(np.abs(filt_err[sel])) * 1e3:.4f} mrad   measurement max |e| '
                  f'{np.max(np.abs(meas_err[sel])) * 1e3:.4f} mrad')
 
 
def error_table(b, gt, t, x, y, th, ey, eth, window, x0):
    """The headline comparison: EKF against dead reckoning over the same run.
 
    Dead reckoning starts at the spawn pose, so its frame matches ground truth
    only when the vehicle spawns on the centreline at zero yaw. Checked here
    rather than assumed."""
    print('\nError over the motion window, EKF against dead reckoning')
    gy0 = float(np.interp(t[window][0], gt[:, 0], gt[:, 2]))
    gyaw0 = float(np.interp(t[window][0], gt[:, 0], np.unwrap(gt[:, 3])))
    if abs(gy0) > 0.01 or abs(gyaw0) > 0.01:
        print(f'  spawn was not on the centreline at zero yaw (y {gy0:.3f} m, yaw {gyaw0:.4f} rad):'
              ' the dead reckoning rows would need its frame, skipped')
        rows = [('EKF', ey[window], eth[window])]
    else:
        dr = sorted_rows([[stamp_s(m.header), m.pose.pose.position.y,
                           yaw(m.pose.pose.orientation)] for m in b['/odom_dr']], '/odom_dr')
        ts = t[window]
        dr_y = np.interp(ts, dr[:, 0], dr[:, 1]) - np.interp(ts, gt[:, 0], gt[:, 2])
        dr_th = np.interp(ts, dr[:, 0], np.unwrap(dr[:, 2])) - np.interp(ts, gt[:, 0], np.unwrap(gt[:, 3]))
        rows = [('EKF', ey[window], eth[window]), ('dead reckoning', dr_y, dr_th)]
    print(f"  {'':<16} {'lateral RMSE':>13} {'lateral max':>12} {'heading RMSE':>14}"
          f" {'heading max':>12} {'lateral at end':>15}")
    for name, e_y, e_th in rows:
        print(f'  {name:<16} {np.sqrt(np.mean(e_y ** 2)) * 1e3:10.3f} mm '
              f'{np.max(np.abs(e_y)) * 1e3:9.3f} mm {np.degrees(np.sqrt(np.mean(e_th ** 2))):11.4f} deg '
              f'{np.degrees(np.max(np.abs(e_th))):9.4f} deg {e_y[-1] * 1e3:12.3f} mm')
 
 
def verdict(ok):
    return 'PASS' if ok else 'FAIL'
 
 
def main():
    if len(sys.argv) != 2:
        sys.exit('usage: analyse_ekf_update.py <bag directory>')
    b = read_bag(sys.argv[1])
    print(f'Bag: {sys.argv[1]}\n\nTopics, sorted by header stamp')
 
    st = sorted_rows([[stamp_s(m.header), *m.state, *m.covariance] for m in b['/ekf/state']],
                     '/ekf/state')
    up = sorted_rows([[stamp_s(m.header), *m.innovation, *m.innovation_covariance,
                       *m.state_after, m.stamp_gap] for m in b['/ekf/update']], '/ekf/update')
    geo = sorted_rows([[stamp_s(m.header), m.lateral_offset, m.heading_error, float(m.valid)]
                       for m in b['/corridor_geometry']], '/corridor_geometry')
    gt = sorted_rows([[stamp_s(m.header), m.pose.position.x, m.pose.position.y,
                       yaw(m.pose.orientation)] for m in b['/model/vehicle/pose']],
                     '/model/vehicle/pose')
    if len(st) == 0 or len(up) == 0:
        sys.exit('no EKF state or no updates in this bag: the filter never initialised')
 
    t = st[:, 0]
    x, y, th, bb = st[:, 1], st[:, 2], st[:, 3], st[:, 4]
    P = st[:, 5:21].reshape(-1, 4, 4)
    gx = np.interp(t, gt[:, 0], gt[:, 1])
    gy = np.interp(t, gt[:, 0], gt[:, 2])
    gyaw = np.interp(t, gt[:, 0], np.unwrap(gt[:, 3]))
 
    # Initialisation: the filter starts on its first valid scan.
    valid = geo[geo[:, 3] > 0.5]
    t_init = valid[0, 0]
    x0 = np.interp(t_init, gt[:, 0], gt[:, 1])
    speed = np.gradient(gx, t)
    moving = speed > V_MOVING
    if not moving.any():
        sys.exit('the vehicle never moved in this bag')
    i_first, i_last = np.nonzero(moving)[0][[0, -1]]
    window = slice(i_first, i_last + 1)
 
    print(f'\nInitialised from the scan at {t_init:.3f} s; first EKF state at {t[0]:.3f} s')
    print(f'motion from {t[i_first]:.2f} s to {t[i_last]:.2f} s, '
          f'{gx[i_last] - gx[i_first]:.2f} m of true travel')
 
    # C1 mechanics
    n_valid_after = int(np.sum(valid[:, 0] > t_init))
    ratio = len(up) / n_valid_after if n_valid_after else 0.0
    ok1 = ratio >= MECH_MIN
    print(f'\nC1 mechanics   updates {len(up)} / valid scans after init {n_valid_after} '
          f'= {ratio:.4f}   (>= {MECH_MIN})   {verdict(ok1)}')
 
    # C2 bias
    sb = math.sqrt(P[-1, 3, 3])
    eb = bb[-1] - B_TRUE
    ok2 = abs(eb) <= 3 * sb
    T = t[-1] - t[0]
    print(f'C2 bias        b_hat {bb[-1]:.4e} rad/s against {B_TRUE:.4e} '
          f'({100 * eb / B_TRUE:+.2f} percent), claimed sigma_b {sb:.2e} '
          f'({abs(eb) / sb:.2f} sigma, limit 3)   {verdict(ok2)}')
    print(f'               for scale: gyro white noise alone limits sigma_b to about '
          f'{1.45e-4 / math.sqrt(T):.2e} after {T:.0f} s')
    settled = np.abs(bb - B_TRUE) <= 0.05 * B_TRUE
    not_settled = np.nonzero(~settled)[0]
    if settled[-1]:
        k = not_settled[-1] + 1 if len(not_settled) else 0
        print(f'               within 5 percent from {t[k] - t[0]:.2f} s after initialisation onward')
 
    # C3, C4 lateral and heading against truth, during motion
    ey = y - gy
    eth = th - gyaw
    ok3 = np.max(np.abs(ey[window])) <= LAT_TOL
    ok4 = np.max(np.abs(eth[window])) <= HEAD_TOL
    print(f'C3 lateral     max |error| {np.max(np.abs(ey[window])) * 1e3:.3f} mm, '
          f'mean {np.mean(ey[window]) * 1e3:+.3f} mm   (<= {LAT_TOL * 1e3:.1f} mm)   {verdict(ok3)}')
    print(f'C4 heading     max |error| {np.max(np.abs(eth[window])) * 1e3:.4f} mrad, '
          f'mean {np.mean(eth[window]) * 1e3:+.4f} mrad   (<= {HEAD_TOL * 1e3:.1f} mrad)   '
          f'{verdict(ok4)}')
 
    # C5 longitudinal: unobservable, so it must drift like dead reckoning
    d_true = gx[-1] - x0
    ex = x[-1] - d_true
    ok5 = abs(ex - 0.01 * d_true) <= X_TOL
    print(f'C5 x           x error at end {ex:+.3f} m over {d_true:.2f} m, '
          f'expected {0.01 * d_true:+.3f} m from the radius error   (within {X_TOL:.2f} m)   '
          f'{verdict(ok5)}')
 
    # Diagnostics
    print('\nDiagnostics (no verdict)')
    for name, err, var, unit, scale in (('lateral', ey[-1], P[-1, 1, 1], 'mm', 1e3),
                                        ('heading', eth[-1], P[-1, 2, 2], 'mrad', 1e3),
                                        ('x', ex, P[-1, 0, 0], 'm', 1.0)):
        s = math.sqrt(var)
        r = abs(err) / s if s > 0 else float('inf')
        print(f'  {name:<8} actual {err * scale:+10.4f} {unit}   claimed sigma {s * scale:10.4f} {unit}'
              f'   actual / claimed {r:8.2f}')
    innov = up[:, 1:3]
    S = up[:, 3:7].reshape(-1, 2, 2)
    nis = np.array([v @ np.linalg.solve(Sk, v) for v, Sk in zip(innov, S)])
    print(f'  NIS mean {np.mean(nis):.4f} (2 if consistent; expected well below)')
    gap = up[:, 11]
    print(f'  stamp gap, filter time minus scan stamp: median {np.median(gap) * 1e3:.1f} ms, '
          f'max {np.max(gap) * 1e3:.1f} ms, min {np.min(gap) * 1e3:.1f} ms')
 
    heading_excursions(geo, gt, t, eth, window)
    error_table(b, gt, t, x, y, th, ey, eth, window, x0)
 
    all_ok = ok1 and ok2 and ok3 and ok4 and ok5
    print(f'\nC1 to C5: {verdict(all_ok)}')
    return 0 if all_ok else 1
 
 
if __name__ == '__main__':
    sys.exit(main())
 