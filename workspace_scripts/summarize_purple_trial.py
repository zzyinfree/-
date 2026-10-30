#!/usr/bin/env python3
"""Summarize a recorded purple flight without equating BT success with a hit."""
import argparse
import json
import math
import re
from pathlib import Path

import rosbag

BOUNDS = (-1.0, 6.0, -3.0, 3.0)
SPRINT = re.compile(r'\[(\d+\.\d+)\].*Area ([\d.]+)%.*-> SPRINT! err=\(([-\d.]+),([-\d.]+)\).*pos=\(([-\d.]+),([-\d.]+),([-\d.]+)\).*sprint_vel=([\d.]+), time=([\d.]+)s')
PULSE = re.compile(r'SPRINT front-center.*err_x=([-\d.]+)px vx:([\d.]+) vy:([-\d.]+)')
DOWN = re.compile(r'purple down verify frames=(\d+) target=(\d+) other=(\d+) motion=(\d+) attempt=(\d+)')


def nearest(samples, when):
    if not samples:
        return None
    t, x, y, z = min(samples, key=lambda row: abs(row[0] - when))
    return {'stamp': round(t, 3), 'xyz': [round(x, 3), round(y, 3), round(z, 3)]}


def summarize(recording, mission_log):
    lines = mission_log.read_text(encoding='utf-8', errors='replace').splitlines()
    sprints = []
    down = []
    for line in lines:
        m = SPRINT.search(line)
        if m:
            stamp, area, ex, ey, x, y, z, speed, duration = map(float, m.groups())
            sprints.append({'stamp': stamp, 'area_percent': area,
                            'pixel_error': [ex, ey], 'start_xyz': [x, y, z],
                            'command_speed_mps': speed, 'command_duration_s': duration,
                            'pulse_samples': []})
        m = PULSE.search(line)
        if m and sprints:
            ex, vx, vy = map(float, m.groups())
            sprints[-1]['pulse_samples'].append({'error_x_px': ex, 'vx_mps': vx, 'vy_mps': vy})
        m = DOWN.search(line)
        if m:
            frames, target, other, motion, attempt = map(int, m.groups())
            down.append({'frames': frames, 'purple_frames': target, 'other_frames': other,
                         'motion_frames': motion, 'attempt': attempt})

    poses = []
    detections = []
    with rosbag.Bag(str(recording / 'flight.bag')) as bag:
        for topic, msg, t in bag.read_messages(topics=['/rflysim/uav1/local/odom', '/objects']):
            stamp = t.to_sec()
            if topic == '/rflysim/uav1/local/odom':
                p = msg.pose.pose.position
                poses.append((stamp, p.x, p.y, p.z))
            elif msg.source_id == 0:
                for obj in msg.objects:
                    if obj.class_name == 'purple_balloon':
                        w = max(0, obj.right_bottom_x - obj.left_top_x)
                        h = max(0, obj.right_bottom_y - obj.left_top_y)
                        detections.append((stamp, obj.center_x, obj.center_y, w * h, obj.score))
    for sprint in sprints:
        start = sprint['stamp']
        end = start + sprint['command_duration_s']
        sprint['measured_start'] = nearest(poses, start)
        sprint['measured_command_end'] = nearest(poses, end)
        sprint['measured_after_coast_0p5s'] = nearest(poses, end + 0.5)
        a, b = sprint['measured_start'], sprint['measured_command_end']
        if a and b:
            sprint['actual_horizontal_command_m'] = round(math.dist(a['xyz'][:2], b['xyz'][:2]), 3)
        sprint['front_detections_during_pulse'] = sum(start <= d[0] <= end for d in detections)
        nearest_detection = min(detections, key=lambda d: abs(d[0] - start)) if detections else None
        if nearest_detection and abs(nearest_detection[0] - start) < 0.3:
            sprint['front_at_trigger'] = {'error_x_px': nearest_detection[1] - 320,
                                          'center_y_px': nearest_detection[2],
                                          'box_area_px': nearest_detection[3],
                                          'confidence': round(nearest_detection[4], 3)}
    xmin, xmax, ymin, ymax = BOUNDS
    margins = [min(x - xmin, xmax - x, y - ymin, ymax - y) for _, x, y, _ in poses]
    target_persisted = any(row['purple_frames'] >= 3 for row in down)
    return {
        'recording': str(recording), 'mission_log': str(mission_log),
        'sprints': sprints, 'down_verification': down,
        'min_horizontal_boundary_margin_m': round(min(margins), 3) if margins else None,
        'out_of_boundary_logged': any('DRONE OUT OF BOUNDARY' in s or 'servo safety envelope reached' in s for s in lines),
        'front_detections': len(detections), 'odom_samples': len(poses),
        'front_frames': len(list((recording / 'front').glob('frame_*.png'))),
        'down_frames': len(list((recording / 'down').glob('frame_*.png'))),
        'visual_verdict': 'purple_persisted_after_sprint' if target_persisted else 'unverified',
        'judge_hit': None, 'judge_event_available': False,
        'bt_success_seen': any('LLMSequence' in s and 'SUCCESS' in s for s in lines),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('recording', type=Path)
    parser.add_argument('mission_log', type=Path)
    args = parser.parse_args()
    result = summarize(args.recording, args.mission_log)
    path = args.recording / 'metrics.json'
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'metrics': str(path), 'sprints': len(result['sprints']),
                      'visual_verdict': result['visual_verdict'],
                      'min_margin_m': result['min_horizontal_boundary_margin_m'],
                      'judge_hit': result['judge_hit']}))


if __name__ == '__main__':
    main()
