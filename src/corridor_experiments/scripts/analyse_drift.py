#!/usr/bin/env python3
"""Dead reckoning drift against Gazebo ground truth.
 
Test apparatus. Reads ground truth, which corridor_perception must never do.
Trims idle samples before motion begins: the recorder is started before the
launch, so every bag opens with stationary samples that would otherwise be
counted as zero-error travel.
 
Changes from the Part 6 version in scratch/, and only these:
  1. Both topics are sorted by header stamp before anything else. The bag
     returns messages in recorder receive order, which is not publish order,
     and np.interp silently returns wrong values when its x values are not
     increasing.
  2. Added output only: backward intervals found in the bag, the time the bag
     runs on after the vehicle stops, and dead reckoned heading in each row.
     Every line the old version printed is computed the same way, so the two
     outputs can be compared line for line.
"""
import sys
import math
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
 
 
def yaw_from_quat(z, w):
    return 2.0 * math.atan2(z, w)
 
 
def read_bag(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id='mcap'),
                ConverterOptions('', ''))
    dr, gt = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/odom_dr':
            m = deserialize_message(data, Odometry)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            dr.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                       yaw_from_quat(m.pose.pose.orientation.z,
                                     m.pose.pose.orientation.w),
                       m.twist.twist.linear.x))
        elif topic == '/model/vehicle/pose':
            m = deserialize_message(data, PoseStamped)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            gt.append((t, m.pose.position.x, m.pose.position.y,
                       yaw_from_quat(m.pose.orientation.z,
                                     m.pose.orientation.w)))
    return np.array(dr), np.array(gt)
 
 
def to_publish_order(a, label):
    back = int(np.sum(np.diff(a[:, 0]) <= 0))
    a = a[np.argsort(a[:, 0], kind='stable')]
    print(f'{label:<20} {back:4d} backward intervals in bag order, sorted by stamp')
    return a
 
 
def main(path):
    dr, gt = read_bag(path)
    if len(dr) == 0 or len(gt) == 0:
        print('empty: dr', len(dr), 'gt', len(gt))
        return
    dr = to_publish_order(dr, '/odom_dr')
    gt = to_publish_order(gt, '/model/vehicle/pose')
 
    moving = np.nonzero(dr[:, 4] > 0.01)[0]
    if len(moving) == 0:
        print('no motion found in /odom_dr')
        return
    t_stop = dr[moving[-1], 0]
    print(f'bag runs on {dr[-1, 0] - t_stop:.2f} s after the vehicle stops '
          '(the final heading line keeps integrating bias during this time)')
    i0 = moving[0]
    dr = dr[i0:]
    print(f'trimmed {i0} idle samples, {len(dr)} remain')
 
    # Both frames start at the vehicle's pose when motion begins. The dead
    # reckoning node starts at the origin, so ground truth is referred to its
    # own first moving sample rather than to the corridor origin.
    gt0 = np.interp(dr[0, 0], gt[:, 0], gt[:, 1]), np.interp(dr[0, 0], gt[:, 0], gt[:, 2])
    dr0 = dr[0, 1], dr[0, 2]
 
    gx = np.interp(dr[:, 0], gt[:, 0], gt[:, 1]) - gt0[0]
    gy = np.interp(dr[:, 0], gt[:, 0], gt[:, 2]) - gt0[1]
    ex = (dr[:, 1] - dr0[0]) - gx
    ey = (dr[:, 2] - dr0[1]) - gy
 
    dist = np.sqrt(gx**2 + gy**2)          # true distance travelled
    err = np.sqrt(ex**2 + ey**2)
 
    print(f'true distance travelled   {dist[-1]:.3f} m')
    print(f'reported distance         {np.sqrt((dr[-1,1]-dr0[0])**2 + (dr[-1,2]-dr0[1])**2):.3f} m')
    print(f'longitudinal error        {ex[-1]:+.3f} m')
    print(f'lateral error             {ey[-1]:+.3f} m')
    print(f'total position error      {err[-1]:.3f} m')
    print(f'final heading, dr {math.degrees(dr[-1,3]):+.3f} deg')
    print()
    print(' dist_m   err_m   long_m   lat_m   hdg_deg')
    for d in range(0, int(dist[-1]) + 1, 10):
        k = int(np.argmin(np.abs(dist - d)))
        print(f'{dist[k]:7.1f} {err[k]:7.3f} {ex[k]:+8.3f} {ey[k]:+7.3f} {math.degrees(dr[k, 3]):+8.3f}')
 
 
if __name__ == '__main__':
    main(sys.argv[1])
 