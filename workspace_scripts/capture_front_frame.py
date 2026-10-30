#!/usr/bin/env python3
"""Capture exactly one front RGB camera frame; does not command the vehicle."""

import argparse
import json
from pathlib import Path

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/rflysim/sensor1/img_rgb')
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=20)
    args = parser.parse_args()
    rospy.init_node('cicc_frame_capture', anonymous=True, disable_signals=True)
    msg = rospy.wait_for_message(args.topic, Image, timeout=args.timeout)
    bgr = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), bgr):
        raise OSError(path)
    print(json.dumps({'topic': args.topic, 'encoding': msg.encoding,
                      'shape': list(bgr.shape), 'seq': msg.header.seq,
                      'stamp': msg.header.stamp.to_sec(), 'output': str(path)}))


if __name__ == '__main__':
    main()
