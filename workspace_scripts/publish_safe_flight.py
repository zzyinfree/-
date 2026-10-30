#!/usr/bin/env python3
"""Publish one fixed simulator-only takeoff-and-land mission."""

import json
import time

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('cicc_safe_flight_test', anonymous=True, disable_signals=True)
    pub = rospy.Publisher('/mission_command', String, queue_size=1, latch=True)
    deadline = time.monotonic() + 10
    while pub.get_num_connections() < 1 and time.monotonic() < deadline:
        time.sleep(0.1)
    if pub.get_num_connections() < 1:
        raise RuntimeError('mission node is not subscribed')
    payload = json.dumps({'commands': ['takeoff', 'land']}, separators=(',', ':'))
    pub.publish(String(data=payload))
    print(f'published {payload}', flush=True)
    # Keep the transient publisher alive until TCPROS has flushed the message.
    time.sleep(3.0)


if __name__ == '__main__':
    main()
