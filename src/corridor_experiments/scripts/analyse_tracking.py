#!/usr/bin/env python3
"""
Phase 3c: obstacle tracking against truth, and against the raw detections.
 
Reads a bag with /obstacle_tracks /obstacles /model/vehicle/pose, and the
scenario's JSON sidecar.
 
Test apparatus: reads Gazebo ground truth, which the perception nodes never
may. Truth boxes are transformed into base_link at each scan, as in 3b, so
tracks, detections and truth are all compared in the detector's own frame.
 
The point of this step is not accuracy. The detector is already accurate. It
is identity and continuity: that one obstacle produces one track, that the
track keeps its id, and that the estimate is steadier than the raw measurement
it is built from.
 
Exit criteria, fixed before the first run:
  C1 identity     each obstacle is matched by exactly one confirmed track id
                  over the whole run
  C2 no phantoms  no confirmed track matches no obstacle
  C3 accuracy     track error no worse than raw detection error, within 10
                  percent, in both axes
  C4 steadiness   track jitter below raw jitter in both axes
 
Jitter is the standard deviation of the scan to scan change in the error.
It measures how much the estimate twitches, which is what a controller feels,
and is a different question from how far it sits from truth. A filter that
smooths will improve C4 while leaving C3 roughly unchanged, since smoothing
removes noise but not bias.
 
  C5 coasting     with the extraction hook active, a track survives every
                  outage with its id intact, and its error while coasting
                  stays within COAST_TOL of truth
 
C5 only reports when the bag actually contains outages, so the same script
serves both the normal bags and the dropout bag. It is the only criterion that
tests the ego-motion transform itself: while measurements are absent the track
is moved purely by the EKF delta pose, so a sign error in that transform makes
the track drift the wrong way at twice the vehicle speed, which nothing else
here would notice.
 
Usage, from ~/ros2_ws:
  python3 analyse_tracking.py bags/track_single_left \\
      src/corridor_sim/worlds/corridor_single_left.obstacles.json
"""
 
# A coasting track is dead reckoning on the obstacle: the EKF carries it for
# the length of the outage. At 2 m/s a 0.5 s outage is 1 m of travel, and the
# EKF's own lateral error over that distance is well under a millimetre, so
# anything above a few centimetres is a transform error rather than accumulated
# noise. 0.25 m is deliberately loose: the purpose is to catch a wrong sign,
# which shows up as metres, not to re-measure the EKF.
 
import json
import math
import sys
 
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
 
MATCH_RADIUS = 2.0
ACCURACY_SLACK = 1.10     # C3: track error may be up to this times the raw error
COAST_TOL = 0.25          # m, C5, see the note above
NEAR_BAND = 20.0          # m, the band C5 requires a track to survive in
MAX_RANGE = 30.0          # m, the detector's max_x: beyond it nothing is seen.
                          # Taken from the node's own startup line, not from
                          # the value I remembered it being set to
DETECT_RANGE = 28.0        # m, measured first detection range, a little inside
                           # the 29.7 m onset so marginal scans are not counted
                           # as outages
 
WANTED = ['/obstacle_tracks', '/obstacles', '/model/vehicle/pose']
 
 
def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9
 
 
def yaw_of(q):
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
            'y': 0.5 * (min(ys) + max(ys))}
 
 
def match(items, truths, getx, gety):
    """Greedy nearest assignment of items to truths. Returns index per truth
    and the set of unmatched item indices."""
    out = [None] * len(truths)
    used = set()
    for k, tr in enumerate(truths):
        best, best_d = None, MATCH_RADIUS
        for j, it in enumerate(items):
            if j in used:
                continue
            d = math.hypot(getx(it) - tr['x'], gety(it) - tr['y'])
            if d < best_d:
                best, best_d = j, d
        if best is not None:
            used.add(best)
            out[k] = best
    return out, [j for j in range(len(items)) if j not in used]
 
 
def jitter(series):
    """Standard deviation of the scan to scan change, ignoring gaps."""
    a = np.array([v for v in series if v is not None], dtype=float)
    return float(np.std(np.diff(a))) if len(a) > 2 else float('nan')
 
 
def main():
    if len(sys.argv) != 3:
        sys.exit('usage: analyse_tracking.py <bag directory> <obstacles json>')
    bag_path, json_path = sys.argv[1], sys.argv[2]
 
    scenario = json.load(open(json_path))
    size = scenario['size']
    truth_world = [(o['x'], o['y']) for o in scenario['obstacles']]
    print(f'Bag: {bag_path}')
    print(f'Scenario: {len(truth_world)} obstacles\n')
 
    b = read_bag(bag_path)
    gt = np.array([[stamp_s(m.header), m.pose.position.x, m.pose.position.y,
                    yaw_of(m.pose.orientation)] for m in b['/model/vehicle/pose']])
    gt = gt[np.argsort(gt[:, 0], kind='stable')]
 
    tracks_by_t = {round(stamp_s(m.header), 6): m for m in b['/obstacle_tracks']}
    raw_by_t = {round(stamp_s(m.header), 6): m for m in b['/obstacles']}
    # Iterate over track stamps, not the intersection: during a forced outage
    # the extraction node publishes nothing at all, so those scans have a track
    # message and no detection message. Intersecting would silently discard
    # exactly the scans C5 exists to examine.
    stamps = sorted(tracks_by_t)
    n_missing = sum(1 for t in stamps if t not in raw_by_t)
    n_empty = sum(1 for m in b['/obstacles'] if not m.obstacles)
    print(f'{len(b["/obstacle_tracks"])} track messages, {len(b["/obstacles"])} '
          f'detection messages, {n_empty} of them empty, '
          f'{n_missing} scans with no detection message')
    # A plain count of empty messages says nothing about the dropout hook. The
    # detector publishes an empty array on every scan where nothing clusters,
    # which is every scan before the first obstacle comes into range, and in a
    # 30 m run that is well over a hundred scans. An empty message is evidence
    # of an outage only when an obstacle was inside detection range at the
    # time, which is what the count below measures. Reported here rather than
    # left to inference, because the raw count of 155 was read once as proof
    # the hook was working when it was proof of nothing.
    n_suspicious = 0
 
    ids_per_truth = [set() for _ in truth_world]
    phantoms = 0
    first_confirmed = None
    # per truth: lists of errors and of raw errors, aligned by scan
    t_ex = [[] for _ in truth_world]
    t_ey = [[] for _ in truth_world]
    r_ex = [[] for _ in truth_world]
    r_ey = [[] for _ in truth_world]
    coast_err = [[] for _ in truth_world]     # C5: error while no detection
    coast_lost = [0] * len(truth_world)       # outage scans with no track at all
    coast_ids = [set() for _ in truth_world]
    # An obstacle beyond the detector's max_x was never available to be
    # tracked, so requiring a track for it scores the scenario rather than the
    # tracker. This caught a real apparatus bug: the slalom obstacle at 70 m
    # with target_x 30 m is never inside sensor range, and C1 failed it as a
    # lost track.
    observable = [False] * len(truth_world)
 
    for t in stamps:
        px = float(np.interp(t, gt[:, 0], gt[:, 1]))
        py = float(np.interp(t, gt[:, 0], gt[:, 2]))
        yaw = float(np.interp(t, gt[:, 0], np.unwrap(gt[:, 3])))
        truths = [truth_in_base_link(ox, oy, size, px, py, yaw)
                  for ox, oy in truth_world]
 
        have_raw = t in raw_by_t
        tk = list(tracks_by_t[t].tracks)
        rw = list(raw_by_t[t].obstacles) if have_raw else []
        tk_idx, unmatched = match(tk, truths, lambda o: o.x, lambda o: o.y)
        rw_idx, _ = match(rw, truths, lambda o: o.min_x, lambda o: o.y)
        phantoms += len(unmatched)
 
        if have_raw and not rw and any(0.0 < tr['min_x'] < DETECT_RANGE
                                       for tr in truths):
            n_suspicious += 1
 
        for k, tr in enumerate(truths):
            if 0.0 < tr['min_x'] < MAX_RANGE:
                observable[k] = True
            j = tk_idx[k]
            if j is not None:
                ids_per_truth[k].add(tk[j].id)
                t_ex[k].append(tk[j].x - tr['min_x'])
                t_ey[k].append(tk[j].y - tr['y'])
                if first_confirmed is None or tr['min_x'] > first_confirmed:
                    first_confirmed = tr['min_x']
            i = rw_idx[k] if have_raw else None
            if i is not None:
                r_ex[k].append(rw[i].min_x - tr['min_x'])
                r_ey[k].append(rw[i].y - tr['y'])
            elif tr['min_x'] < NEAR_BAND:
                # No detection for this obstacle on this scan, while it is
                # close enough that a track should exist. Covers both an empty
                # array, which is what the dropout hook publishes, and a
                # missing message. Either the track coasted, which is
                # measurable, or it vanished, which is a failure.
                #
                # Keyed on the absence of a matching DETECTION, not on the
                # absence of a message. The first version of this test looked
                # for stamps present in tracks and absent from detections,
                # which can never happen: the tracker's only tick is the
                # arrival of a detection message, so a suppressed scan removes
                # one message from each topic and leaves the two in step.
                if j is None:
                    coast_lost[k] += 1
                else:
                    coast_ids[k].add(tk[j].id)
                    coast_err[k].append(math.hypot(tk[j].x - tr['min_x'],
                                                   tk[j].y - tr['y']))
 
    print(f'{n_suspicious} empty messages with an obstacle inside '
          f'{DETECT_RANGE:.0f} m: this is the dropout count, and zero means '
          f'the hook did not fire')
 
    print(f'\n{"obstacle":>9} {"track ids":>22} {"scans":>6} {"raw x":>8} {"trk x":>8}'
          f' {"raw y":>8} {"trk y":>8} {"jit x":>8} {"jit y":>8}')
    print(f'{"":>9} {"":>22} {"":>6} {"rms mm":>8} {"rms mm":>8} {"rms mm":>8}'
          f' {"rms mm":>8} {"r / t":>8} {"r / t":>8}')
    print('-' * 92)
 
    def rms(a):
        return float(np.sqrt(np.mean(np.square(a)))) if len(a) else float('nan')
 
    ok3 = ok4 = True
    for k in range(len(truth_world)):
        if not observable[k]:
            print(f'{k:>9} {"never in range":>22} {"":>6}   (not scored)')
            continue
        jx_r, jx_t = jitter(r_ex[k]), jitter(t_ex[k])
        jy_r, jy_t = jitter(r_ey[k]), jitter(t_ey[k])
        print(f'{k:>9} {str(sorted(ids_per_truth[k])):>22} {len(t_ex[k]):6d}'
              f' {rms(r_ex[k]) * 1e3:8.1f} {rms(t_ex[k]) * 1e3:8.1f}'
              f' {rms(r_ey[k]) * 1e3:8.1f} {rms(t_ey[k]) * 1e3:8.1f}'
              f' {jx_r * 1e3:3.0f}/{jx_t * 1e3:<4.0f} {jy_r * 1e3:3.0f}/{jy_t * 1e3:<4.0f}')
        if len(t_ex[k]):
            ok3 &= rms(t_ex[k]) <= ACCURACY_SLACK * rms(r_ex[k])
            ok3 &= rms(t_ey[k]) <= ACCURACY_SLACK * rms(r_ey[k])
            ok4 &= jx_t < jx_r and jy_t < jy_r
        else:
            ok3 = ok4 = False
 
    ok1 = all(len(ids_per_truth[k]) == (1 if observable[k] else 0)
              for k in range(len(truth_world)))
    ok2 = phantoms == 0
 
    def v(ok):
        return 'PASS' if ok else 'FAIL'
 
    print(f'\nC1 identity      ids per obstacle {[len(s) for s in ids_per_truth]},'
          f' observable {observable}   {v(ok1)}')
    print(f'C2 no phantoms   unmatched confirmed tracks {phantoms}   {v(ok2)}')
    print(f'C3 accuracy      track error within {ACCURACY_SLACK:.2f} times raw   {v(ok3)}')
    print(f'C4 steadiness    track jitter below raw jitter   {v(ok4)}')
 
    n_coast = sum(len(c) for c in coast_err)
    if n_coast == 0 and sum(coast_lost) == 0:
        ok5 = None
        print('C5 coasting      no outages in this bag, not tested')
    else:
        worst = max((max(c) for c in coast_err if c), default=float('nan'))
        lost = sum(coast_lost)
        drifted = sum(1 for k in range(len(truth_world))
                      if coast_ids[k] - ids_per_truth[k])
        ok5 = lost == 0 and worst <= COAST_TOL and drifted == 0
        print(f'C5 coasting      {n_coast} coasting scans, {lost} lost, '
              f'worst error {worst * 1e3:.0f} mm   (<= {COAST_TOL * 1e3:.0f})'
              f'   {v(ok5)}')
 
    if first_confirmed is not None:
        print(f'\nFirst confirmed track at {first_confirmed:.2f} m')
 
    all_ok = ok1 and ok2 and ok3 and ok4 and (ok5 is not False)
    print(f'\nOverall: {v(all_ok)}')
    return 0 if all_ok else 1
 
 
if __name__ == '__main__':
    sys.exit(main())
 