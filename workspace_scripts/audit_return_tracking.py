#!/usr/bin/env python3
"""Inspect odometry and local position setpoints around a recorded return abort."""
import sys
import rosbag

bag, start, stop = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
odom = None
last_print = 0
for topic, msg, stamp in rosbag.Bag(bag).read_messages(topics=[
        '/rflysim/uav1/local/odom', '/mavros/setpoint_raw/local']):
    t = stamp.to_sec()
    if t < start or t > stop:
        continue
    if topic.endswith('/odom'):
        p = msg.pose.pose.position
        odom = (p.x, p.y, p.z)
    elif odom and t - last_print >= 0.16 and not (msg.type_mask & 1):
        # Existing MavrosConnect convention: ENU=(NED.y,-NED.x,NED.z).
        enu = (msg.position.y, -msg.position.x, msg.position.z)
        err = sum((a-b)**2 for a,b in zip(odom,enu))**0.5
        print(f'{t:.2f} odom={odom[0]:.2f},{odom[1]:.2f},{odom[2]:.2f} '
              f'set={enu[0]:.2f},{enu[1]:.2f},{enu[2]:.2f} err={err:.2f}')
        last_print = t
