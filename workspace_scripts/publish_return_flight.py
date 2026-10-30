#!/usr/bin/env python3
"""Development-only frame out-and-back flight, without balloon impact."""
import json
import time
import rospy
from std_msgs.msg import String

rospy.init_node('cicc_return_flight_test', anonymous=True, disable_signals=True)
pub = rospy.Publisher('/mission_command', String, queue_size=1, latch=True)
deadline = time.monotonic() + 10
while pub.get_num_connections() < 1 and time.monotonic() < deadline:
    time.sleep(0.1)
if pub.get_num_connections() < 1:
    raise RuntimeError('mission node is not subscribed')
payload = json.dumps({'commands': ['takeoff', 'cross_frame', 'detect_aruco', 'return_cross_frame', 'land']}, separators=(',', ':'))
pub.publish(String(data=payload))
print('published', payload, flush=True)
time.sleep(3.0)
