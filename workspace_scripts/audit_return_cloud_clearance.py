#!/usr/bin/env python3
"""Read-only cloud/odom proximity audit for a recorded simulation sortie.

This does not prove absence of UE collisions: the depth camera has blind spots,
and the point cloud can include target objects. It makes dangerous near-surface
segments visible for comparison across planner changes.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import rosbag
from sensor_msgs import point_cloud2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', type=Path)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--start', type=float, default=0.0)
    ap.add_argument('--stop', type=float, default=1e20)
    args = ap.parse_args()
    samples = []
    pose = None
    pose_t = None
    for topic, msg, stamp in rosbag.Bag(str(args.bag)).read_messages(
            topics=['/rflysim/uav1/local/odom', '/cloud_registered']):
        t = stamp.to_sec()
        if topic.endswith('/odom'):
            p = msg.pose.pose.position
            pose = np.array([p.x, p.y, p.z], dtype=float)
            pose_t = t
            continue
        if not args.start <= t <= args.stop or pose is None or abs(t-pose_t)>0.3:
            continue
        if msg.width == 0:
            continue
        pts = np.array(list(point_cloud2.read_points(
            msg, field_names=('x','y','z'), skip_nans=True)), dtype=np.float32)
        if not len(pts):
            continue
        delta = pts - pose
        # Exclude ground and ceiling, evaluate the body-height band only.
        band = np.abs(delta[:,2]) < 0.30
        if not band.any():
            continue
        xy = np.linalg.norm(delta[band,:2], axis=1)
        d = float(np.min(xy))
        samples.append({'t': round(t,3), 'position': pose.round(3).tolist(),
                        'min_horizontal_cloud_m': round(d,3),
                        'points_body_band': int(band.sum()),
                        'points_within_0_35m': int(np.sum(xy<0.35))})
    vals = np.array([s['min_horizontal_cloud_m'] for s in samples])
    result = {'bag': str(args.bag), 'start': args.start, 'stop': args.stop,
              'interpretation': 'observed front-depth clearance only, not collision score',
              'samples': len(samples), 'min_m': float(vals.min()) if len(vals) else None,
              'p05_m': float(np.quantile(vals,.05)) if len(vals) else None,
              'within_0_35m': int(np.sum(vals<.35)) if len(vals) else 0,
              'nearest_20': sorted(samples,key=lambda r:r['min_horizontal_cloud_m'])[:20]}
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='nearest_20'},ensure_ascii=False))


if __name__ == '__main__':
    main()
