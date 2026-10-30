#!/usr/bin/env python3
"""Record real simulation camera frames for offline review; no vehicle commands."""

import argparse
import hashlib
import json
import signal
import time
from pathlib import Path

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from common_msgs.msg import Objects


def main():
    stopping = [False]
    def request_stop(_signum, _frame):
        stopping[0] = True
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--count', type=int, default=40)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--timeout', type=float, default=80.0)
    parser.add_argument('--allow-partial', action='store_true',
                        help='Write manifest and exit successfully when a time-boxed audit capture ends')
    parser.add_argument('--group-id', required=True,
                        help='Independent capture session ID; never split one group across train/test')
    parser.add_argument('--scene-note', default='', help='Scene and flight context, not a claimed random seed')
    parser.add_argument('--topic', default='/rflysim/sensor1/img_rgb')
    args = parser.parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    bridge = CvBridge()
    records = []
    last_saved = [0.0]
    latest_pose = [None]
    latest_sim_odom = [None]
    latest_objects = [None]
    topic = args.topic

    def callback(msg):
        if len(records) >= args.count:
            return
        now = time.monotonic()
        if now - last_saved[0] < args.interval:
            return
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        name = f'frame_{len(records):04d}.png'
        if not cv2.imwrite(str(out / name), frame):
            raise OSError(out / name)
        sha256 = hashlib.sha256((out / name).read_bytes()).hexdigest()
        records.append({'image': name, 'sha256': sha256, 'seq': msg.header.seq,
                        'stamp': msg.header.stamp.to_sec(),
                        'host_time': time.time(),
                        'encoding': msg.encoding, 'shape': list(frame.shape),
                        'pose': latest_pose[0],
                        'sim_odom': latest_sim_odom[0],
                        'objects': latest_objects[0],
                        'label_status': 'unverified'})
        last_saved[0] = now
        if len(records) % 10 == 0:
            print(f'saved {len(records)}/{args.count}', flush=True)

    rospy.init_node('cicc_training_capture', anonymous=True, disable_signals=True)
    def pose_callback(msg):
        p = msg.pose.position
        q = msg.pose.orientation
        latest_pose[0] = {'stamp': msg.header.stamp.to_sec(),
                          'position': [p.x,p.y,p.z],
                          'orientation_xyzw': [q.x,q.y,q.z,q.w]}

    pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                pose_callback, queue_size=1)
    def odom_callback(msg):
        p=msg.pose.pose.position
        q=msg.pose.pose.orientation
        latest_sim_odom[0]={'stamp':msg.header.stamp.to_sec(),
                            'frame_id':msg.header.frame_id,
                            'position':[p.x,p.y,p.z],
                            'orientation_xyzw':[q.x,q.y,q.z,q.w]}

    odom_sub=rospy.Subscriber('/rflysim/uav1/local/odom',Odometry,
                              odom_callback,queue_size=1)
    def objects_callback(msg):
        latest_objects[0] = {
            'stamp': msg.header.stamp.to_sec(),
            'received_at': time.time(),
            'source_id': msg.source_id,
            'boxes': [
                {'class_name': obj.class_name,
                 'xyxy': [obj.left_top_x, obj.left_top_y,
                          obj.right_bottom_x, obj.right_bottom_y],
                 'center': [obj.center_x, obj.center_y],
                 'score': obj.score}
                for obj in msg.objects
            ],
        }

    objects_sub = rospy.Subscriber('/objects', Objects, objects_callback, queue_size=1)
    subscriber = rospy.Subscriber(topic, Image, callback, queue_size=1)
    started = time.monotonic()
    while not stopping[0] and len(records) < args.count and time.monotonic() - started < args.timeout:
        time.sleep(0.1)
    subscriber.unregister()
    pose_sub.unregister()
    odom_sub.unregister()
    objects_sub.unregister()
    (out / 'capture_manifest.json').write_text(json.dumps({
        'source_topic': topic, 'group_id': args.group_id,
        'scene_note': args.scene_note, 'label_status': 'unverified',
        'frames': records,
    }, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'saved': len(records), 'output_dir': str(out)}), flush=True)
    if len(records) != args.count and not args.allow_partial:
        raise TimeoutError(f'captured {len(records)}/{args.count}')


if __name__ == '__main__':
    main()
