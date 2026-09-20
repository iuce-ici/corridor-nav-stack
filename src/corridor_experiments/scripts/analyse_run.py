#!/usr/bin/env python3
"""Compare extracted corridor geometry against Gazebo ground truth.
 
Test apparatus. Reads ground truth, which corridor_perception must never do.
 
Changes from the Part 5 version in scratch/, and only these:
  1. Both topics are sorted by header stamp before anything else. The bag
     returns messages in recorder receive order, which is not publish order,
     and np.interp silently returns wrong values when its x values are not
     increasing. Every line the old version printed is otherwise computed the
     same way, so the two outputs can be compared line for line.
  2. Added output only:
       backward intervals found in the bag, per topic;
       how far each geometry message's ground truth x moves between the
         unsorted and the sorted interpolation, and how many messages change
         bucket or regime because of it;
       a regime table with the boundaries used in the published Part 5 table,
         so that table can be reproduced from this script.
 
Known flaw kept deliberately for comparability: no idle trim. The summary lines
include stationary and degraded samples. The bucket and regime tables are the
trustworthy output.
"""
 
import sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
 
# Regime boundaries in base_link x, as in the published Part 5 table.
# 115.8 is analytic: end wall at 150.0, minus LiDAR x 4.2, minus max range 30.0.
# 128.0 is the published boundary; how it was derived is not recorded.
ONSET_X = 115.8
DOMINANT_X = 128.0
 
 
def stamp_to_sec(s):
    return s.sec + s.nanosec * 1e-9
 
 
def read(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id='mcap'),
                ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
 
    geo, truth = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/corridor_geometry':
            m = deserialize_message(data, get_message(types[topic]))
            geo.append((stamp_to_sec(m.header.stamp), m.lateral_offset,
                        m.heading_error, m.corridor_width,
                        m.left_points, m.right_points,
                        m.left_residual, m.right_residual, m.valid))
        elif topic == '/model/vehicle/pose':
            m = deserialize_message(data, get_message(types[topic]))
            q = m.pose.orientation
            yaw = 2.0 * np.arctan2(q.z, q.w)
            truth.append((stamp_to_sec(m.header.stamp),
                          m.pose.position.x, m.pose.position.y, yaw))
    return np.array(geo), np.array(truth)
 
 
def bucket_edges(tx):
    return np.linspace(tx.min(), tx.max(), 11)
 
 
def regime_of(tx):
    return np.where(tx < ONSET_X, 0, np.where(tx < DOMINANT_X, 1, 2))
 
 
def main(path):
    geo_raw, truth_raw = read(path)
    print(f"geometry msgs {len(geo_raw)}, truth msgs {len(truth_raw)}")
 
    back_g = int(np.sum(np.diff(geo_raw[:, 0]) <= 0))
    back_t = int(np.sum(np.diff(truth_raw[:, 0]) <= 0))
    print(f"backward intervals in bag order: geometry {back_g}, truth {back_t}"
          "   -> both sorted by stamp below")
    geo = geo_raw[np.argsort(geo_raw[:, 0], kind='stable')]
    truth = truth_raw[np.argsort(truth_raw[:, 0], kind='stable')]
 
    # Nearest-in-time truth for each geometry message.
    tx = np.interp(geo[:, 0], truth[:, 0], truth[:, 1])
    ty = np.interp(geo[:, 0], truth[:, 0], truth[:, 2])
    tyaw = np.interp(geo[:, 0], truth[:, 0], truth[:, 3])
 
    # What the unsorted interpolation gave for the same geometry messages.
    tx_old = np.interp(geo[:, 0], truth_raw[:, 0], truth_raw[:, 1])
    ty_old = np.interp(geo[:, 0], truth_raw[:, 0], truth_raw[:, 2])
    dx = tx_old - tx
    e_old, e_new = bucket_edges(tx_old), bucket_edges(tx)
    b_old = np.clip(np.searchsorted(e_old, tx_old, side='right') - 1, 0, 9)
    b_new = np.clip(np.searchsorted(e_new, tx, side='right') - 1, 0, 9)
    print(f"damage from the unsorted interpolation: truth x max |dx| "
          f"{np.max(np.abs(dx)):.3f} m, messages with |dx| > 1 cm "
          f"{int(np.sum(np.abs(dx) > 0.01))}; truth y max |dy| "
          f"{np.max(np.abs(ty_old - ty)) * 1000:.3f} mm")
    print(f"  bucket edges moved by up to {np.max(np.abs(e_old - e_new)):.3f} m; "
          f"messages changing bucket {int(np.sum(b_old != b_new))}, "
          f"changing regime {int(np.sum(regime_of(tx_old) != regime_of(tx)))}")
 
    off_err = geo[:, 1] - ty
    head_err = geo[:, 2] - tyaw
    width_err = geo[:, 3] - 8.0
 
    def stats(name, e, unit, scale=1.0):
        e = e * scale
        print(f"{name:22} mean {np.mean(e):9.4f}  "
              f"rms {np.sqrt(np.mean(e**2)):9.4f}  "
              f"max|e| {np.max(np.abs(e)):9.4f}  {unit}")
 
    print()
    stats("lateral offset error", off_err, "mm", 1000.0)
    stats("heading error", head_err, "mdeg", np.degrees(1.0) * 1000.0)
    stats("width error", width_err, "mm", 1000.0)
 
    print(f"\nvalid fraction {np.mean(geo[:, 8]):.4f}")
    print(f"points L min {geo[:,4].min():.0f} max {geo[:,4].max():.0f}")
    print(f"points R min {geo[:,5].min():.0f} max {geo[:,5].max():.0f}")
    print(f"residual L max {geo[:,6].max():.4f}  R max {geo[:,7].max():.4f}")
 
    # Behaviour against distance travelled, ten buckets.
    print(f"\n{'x range':>16} {'n':>6} {'off err mm':>12} {'widtherr mm':>14}"
          f" {'ptsL':>7} {'ptsR':>7} {'resR':>8}")
    edges = bucket_edges(tx)
    for i in range(10):
        m = (tx >= edges[i]) & (tx < edges[i + 1])
        if m.sum() == 0:
            continue
        print(f"{edges[i]:7.1f}-{edges[i+1]:6.1f} {m.sum():6d} "
              f"{np.mean(off_err[m])*1000:12.2f} {np.mean(width_err[m])*1000:14.2f}"
              f" {np.mean(geo[m,4]):7.0f} {np.mean(geo[m,5]):7.0f}"
              f" {np.mean(geo[m,7]):8.4f}")
 
    # Regimes, as in the published table. Mean absolute error, since the
    # published columns are magnitudes; signed means are in the bucket table.
    print(f"\n{'regime (base_link x)':>24} {'n':>6} {'|off err| mm':>13}"
          f" {'|width err| mm':>15} {'valid':>7}")
    names = [f"< {ONSET_X}", f"{ONSET_X} to {DOMINANT_X}", f">= {DOMINANT_X}"]
    reg = regime_of(tx)
    for r in range(3):
        m = reg == r
        if m.sum() == 0:
            print(f"{names[r]:>24} {0:6d}")
            continue
        print(f"{names[r]:>24} {m.sum():6d} {np.mean(np.abs(off_err[m]))*1000:13.2f}"
              f" {np.mean(np.abs(width_err[m]))*1000:15.2f} {np.mean(geo[m, 8]):7.3f}")
 
 
main(sys.argv[1])
 