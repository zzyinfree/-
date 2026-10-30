#!/usr/bin/env python3
"""Live downward view + separate, source-labelled FM9G4B-V snapshot evidence."""
import json
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QHBoxLayout, QLabel, QVBoxLayout, QWidget
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String


def set_rgb(label, rgb):
    h, w = rgb.shape[:2]
    image = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy()
    label.setPixmap(QPixmap.fromImage(image).scaled(
        640, 480, Qt.KeepAspectRatio, Qt.SmoothTransformation))


class Viewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Downward QR evidence: live OpenCV | FM9G4B-V independent')
        self.setGeometry(800, 65, 1320, 590)
        self.live = QLabel('Waiting for /rflysim/sensor2/img_rgb')
        self.model = QLabel('FM9G4B-V: waiting for ArUco trigger')
        for label in (self.live, self.model):
            label.setAlignment(Qt.AlignCenter)
            label.setFixedSize(640, 480)
        self.status = QLabel('Left: UI OpenCV ArUco bbox/ID. Right: FM9G4B-V raw-image bbox; ID remains classical.')
        layout = QVBoxLayout(self)
        panes = QHBoxLayout()
        panes.addWidget(self.live)
        panes.addWidget(self.model)
        layout.addLayout(panes)
        layout.addWidget(self.status)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.seq = -1
        self.marker = None
        self.marker_time = 0.0
        self.model_result = None
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.params = cv2.aruco.DetectorParameters_create()
        rospy.Subscriber('/rflysim/sensor2/img_rgb', Image, self.on_image,
                         queue_size=1, buff_size=2**24)
        rospy.Subscriber('/aruco/detected_id', Int32, self.on_marker, queue_size=10)
        rospy.Subscriber('/jiuge/down_marker_result', String, self.on_model, queue_size=2)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(120)

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8').copy()
            with self.lock:
                self.frame = frame
                self.seq = msg.header.seq
        except Exception as exc:
            rospy.logwarn_throttle(3, 'downward Qt conversion failed: %s', exc)

    def on_marker(self, msg):
        with self.lock:
            self.marker = msg.data
            self.marker_time = time.monotonic()

    def on_model(self, msg):
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.model_result = data
        except Exception as exc:
            rospy.logwarn('FM9G result JSON invalid: %s', exc)

    def refresh(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
            seq = self.seq
            marker = self.marker
            marker_time = self.marker_time
            result = self.model_result
        if frame is None:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary,
                                                   parameters=self.params)
        current = []
        if ids is not None:
            for pts, marker_id in zip(corners, ids.flatten()):
                poly = np.round(pts.reshape(-1, 2)).astype(np.int32)
                cv2.polylines(frame, [poly], True, (0, 255, 0), 3)
                x, y = poly.min(axis=0)
                cv2.putText(frame, f'OpenCV ID {int(marker_id)}', (int(x), max(20, int(y)-7)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                current.append(int(marker_id))
        set_rgb(self.live, frame)
        status = f'LIVE down camera seq={seq}; OpenCV visible={current or "none"}'
        if marker is not None and time.monotonic()-marker_time < 20:
            status += f'; mission ArUco ID={marker}'
        if result:
            path = Path(result.get('image', ''))
            snapshot = cv2.imread(str(path)) if path.is_file() else None
            if snapshot is not None:
                snap_rgb = cv2.cvtColor(snapshot, cv2.COLOR_BGR2RGB)
                snap_corners, snap_ids, _ = cv2.aruco.detectMarkers(
                    cv2.cvtColor(snap_rgb, cv2.COLOR_RGB2GRAY), self.dictionary,
                    parameters=self.params)
                if snap_ids is not None:
                    for pts, marker_id in zip(snap_corners, snap_ids.flatten()):
                        poly = np.round(pts.reshape(-1, 2)).astype(np.int32)
                        cv2.polylines(snap_rgb, [poly], True, (0, 255, 0), 3)
                        x, y = poly.min(axis=0)
                        cv2.putText(snap_rgb, f'OpenCV ID {int(marker_id)}',
                                    (int(x), max(20, int(y)-7)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                box = result.get('bbox_px')
                if result.get('valid') and box:
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(snap_rgb, (x1, y1), (x2, y2), (255, 0, 255), 3)
                    cv2.putText(snap_rgb, 'FM9G bbox UNVERIFIED', (8, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                set_rgb(self.model, snap_rgb)
            status += (f' | FM9G seq={result.get("frame_seq")} '
                       f'valid={result.get("valid")} box={result.get("bbox_1000")} '
                       f'raw={result.get("raw", result.get("error", ""))} '
                       f'({result.get("latency_sec")}s); NOT flight input')
        self.status.setText(status)


if __name__ == '__main__':
    rospy.init_node('cicc_downward_camera_qt', anonymous=False)
    app = QApplication(sys.argv)
    viewer = Viewer()
    viewer.show()
    sys.exit(app.exec_())
