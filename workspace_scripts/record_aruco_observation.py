#!/usr/bin/env python3
"""Record down-camera evidence near ArUco hover; never commands the aircraft."""
import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


def decode(msg):
    enc = msg.encoding.lower()
    if enc not in ('rgb8', 'bgr8', 'mono8'):
        raise ValueError(f'unsupported image encoding {msg.encoding}')
    channels = 1 if enc == 'mono8' else 3
    raw = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = raw[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if enc == 'mono8':
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if enc == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img.copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output-dir', required=True)
    ap.add_argument('--timeout', type=float, default=125)
    ap.add_argument('--interval', type=float, default=0.5)
    ap.add_argument('--max-frames', type=int, default=80)
    ap.add_argument('--hover-x', type=float, default=4.5)
    ap.add_argument('--hover-y', type=float, default=0.5)
    ap.add_argument('--hover-z', type=float, default=1.2)
    ap.add_argument('--radius', type=float, default=0.85)
    args = ap.parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    rospy.init_node('cicc_aruco_observation', anonymous=True, disable_signals=True)
    state = {'odom': None, 'reported_ids': [], 'last': 0.0}
    records = []
    dictionary = cv2.aruco.getPredefinedDictionary(16)
    params = cv2.aruco.DetectorParameters_create()

    def odom_cb(msg):
        p = msg.pose.pose.position
        state['odom'] = {'stamp': msg.header.stamp.to_sec(),
                         'xyz': [p.x, p.y, p.z]}

    def id_cb(msg):
        state['reported_ids'].append({'time': time.time(), 'id': int(msg.data)})

    def img_cb(msg):
        if len(records) >= args.max_frames or state['odom'] is None:
            return
        xyz = state['odom']['xyz']
        dist = sum((x-y)**2 for x,y in zip(xyz, [args.hover_x,args.hover_y,args.hover_z]))**0.5
        now = time.monotonic()
        if dist > args.radius or now-state['last'] < args.interval:
            return
        try:
            bgr = decode(msg)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
            name = f'down_{len(records):04d}.png'
            path = out/name
            if not cv2.imwrite(str(path), bgr):
                raise OSError(path)
            detected = [] if ids is None else [int(x) for x in ids.ravel()]
            rec = {'image': name, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                   'stamp': msg.header.stamp.to_sec(), 'seq': msg.header.seq,
                   'encoding': msg.encoding, 'width': msg.width, 'height': msg.height,
                   'odom': state['odom'], 'hover_distance_m': dist,
                   'gray_mean': float(gray.mean()), 'gray_std': float(gray.std()),
                   'offline_aruco_ids': detected, 'rejected_candidates': len(rejected),
                   'label_status': 'unverified'}
            records.append(rec)
            state['last'] = now
            print(json.dumps({'frame': name, 'ids': detected, 'rejected': len(rejected),
                              'xyz': xyz}, ensure_ascii=False), flush=True)
        except Exception as exc:
            rospy.logerr('ArUco observation capture failed: %s', exc)

    rospy.Subscriber('/rflysim/uav1/local/odom', Odometry, odom_cb, queue_size=1)
    rospy.Subscriber('/aruco/detected_id', Int32, id_cb, queue_size=10)
    rospy.Subscriber('/rflysim/sensor2/img_rgb', Image, img_cb, queue_size=1,
                     buff_size=8*1024*1024)
    started = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic()-started < args.timeout and len(records) < args.max_frames:
        time.sleep(0.1)
    manifest = {'topic': '/rflysim/sensor2/img_rgb', 'hover_xyz': [args.hover_x,args.hover_y,args.hover_z],
                'radius_m': args.radius, 'frames': records, 'reported_ids': state['reported_ids'],
                'label_status': 'unverified'}
    (out/'aruco_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'saved':len(records),'reported_ids':state['reported_ids'],
                      'output_dir':str(out)}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
