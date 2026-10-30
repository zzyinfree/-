#!/usr/bin/env python3
"""Persistent raw downward ROS camera viewer with ArUco status overlay.

Observation only: publishes no ROS topics or vehicle commands. The detector
may subscribe/unsubscribe independently; this window remains visible.
"""
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

TOPIC = '/rflysim/sensor2/img_rgb'
TITLE = 'Downward Camera - ArUco / QR (sensor2)'


class Viewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.seq = -1
        self.last_id = None
        self.last_id_time = 0.0
        rospy.Subscriber(TOPIC, Image, self.on_image, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/aruco/detected_id', Int32, self.on_id, queue_size=10)

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logwarn_throttle(3, 'downward view conversion failed: %s', exc)
            return
        with self.lock:
            self.frame = frame
            self.seq = msg.header.seq

    def on_id(self, msg):
        with self.lock:
            self.last_id = msg.data
            self.last_id_time = time.monotonic()

    def run(self):
        cv2.namedWindow(TITLE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(TITLE, 640, 520)
        # DISPLAY=:20 uses a high-DPI desktop. x=1550 put the decorated
        # window beyond the right edge on cloned instances.
        cv2.moveWindow(TITLE, 1200, 70)
        last_report = 0.0
        while not rospy.is_shutdown():
            with self.lock:
                frame = None if self.frame is None else self.frame.copy()
                seq, marker, marker_time = self.seq, self.last_id, self.last_id_time
            if frame is None:
                frame = np.full((480, 640, 3), 15, dtype=np.uint8)
                status = 'Waiting for downward camera...'
            else:
                status = f'{TOPIC}  frame {seq}'
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 58), (15, 15, 15), -1)
            cv2.putText(frame, status, (10, 23), cv2.FONT_HERSHEY_SIMPLEX,
                        .55, (255, 255, 255), 1, cv2.LINE_AA)
            marker_text = (f'ArUco ID {marker} detected' if marker is not None and
                           time.monotonic() - marker_time < 20 else 'ArUco: waiting')
            cv2.putText(frame, marker_text, (10, 47), cv2.FONT_HERSHEY_SIMPLEX,
                        .58, (80, 255, 80), 2, cv2.LINE_AA)
            cv2.imshow(TITLE, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            if time.monotonic() - last_report > 3:
                print(f'downward_view seq={seq} shape={frame.shape} mean={frame.mean():.1f}', flush=True)
                last_report = time.monotonic()
            # Wall-clock pacing: /use_sim_time may pause after a landing/reset.
            time.sleep(1.0 / 15.0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('cicc_downward_camera_view', anonymous=False)
    Viewer().run()
