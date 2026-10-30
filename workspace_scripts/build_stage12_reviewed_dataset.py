#!/usr/bin/env python3
"""Manual, assistant-reviewed boxes from full 640x480 simulator frames.

These are not organizer ground truth. The entries below were checked against
original un-cropped PNGs, not accepted directly from YOLO/HSV proposals.
"""
import json
from collections import Counter
from pathlib import Path
from PIL import Image

ROOT = Path('/root/gpufree-data/cicc_workspace')
OUT = ROOT / 'annotations/stage12_reviewed_bbox.jsonl'
EXP = {
    'red_balloon': '请框出图中的红色气球。只输出一个 <box>。',
    'blue_balloon': '请框出图中的蓝色气球。只输出一个 <box>。',
    'purple_balloon': '请框出图中的紫色气球。只输出一个 <box>。',
    'pillar_obstacle': '请框出图中的砖墙障碍柱。只输出一个 <box>。',
    'square_frame': '请框出图中的方形穿越框。只输出一个 <box>。',
    'ground_marker': '请框出图中的地面二维码标识。只输出一个 <box>。',
}

# (capture, frame, category, visible xyxy). No duplicate class per image.
TRAIN = [
    ('run_014_red_restored_clone', 0, 'red_balloon', [200, 202, 225, 237]),
    ('run_014_red_restored_clone', 0, 'pillar_obstacle', [242, 0, 426, 304]),
    ('run_014_red_restored_clone', 51, 'red_balloon', [211, 206, 234, 238]),
    ('run_014_red_restored_clone', 51, 'pillar_obstacle', [242, 0, 426, 304]),
    ('run_014_red_restored_clone', 127, 'pillar_obstacle', [123, 0, 299, 480]),
    ('run_014_red_restored_clone', 127, 'square_frame', [312, 175, 461, 343]),
    ('run_014_red_restored_clone', 127, 'blue_balloon', [396, 258, 414, 286]),
    ('run_014_red_restored_clone', 127, 'purple_balloon', [479, 261, 503, 295]),
    ('run_014_red_restored_clone', 153, 'square_frame', [214, 123, 480, 438]),
    ('run_014_red_restored_clone', 153, 'blue_balloon', [336, 265, 359, 301]),
    ('run_014_red_restored_clone', 153, 'ground_marker', [119, 326, 233, 359]),
    ('run_014_red_restored_clone', 255, 'blue_balloon', [338, 322, 390, 391]),
    ('run_022_blue_p_only_tight_gate', 0, 'pillar_obstacle', [216, 0, 413, 306]),
    ('run_022_blue_p_only_tight_gate', 0, 'square_frame', [412, 90, 507, 261]),
    ('run_022_blue_p_only_tight_gate', 111, 'pillar_obstacle', [215, 0, 413, 307]),
    ('run_022_blue_p_only_tight_gate', 111, 'purple_balloon', [573, 196, 610, 238]),
    ('run_022_blue_p_only_tight_gate', 221, 'square_frame', [242, 181, 402, 354]),
    ('run_022_blue_p_only_tight_gate', 221, 'blue_balloon', [202, 263, 224, 291]),
    ('run_022_blue_p_only_tight_gate', 221, 'ground_marker', [166, 305, 256, 331]),
    ('run_022_blue_p_only_tight_gate', 258, 'blue_balloon', [470, 288, 505, 334]),
    ('run_022_blue_p_only_tight_gate', 369, 'red_balloon', [78, 318, 134, 384]),
    ('run_022_blue_p_only_tight_gate', 369, 'ground_marker', [329, 379, 510, 480]),
    ('run_022_blue_p_only_tight_gate', 553, 'blue_balloon', [195, 110, 281, 224]),
]
VAL = [
    ('run_019_blue_slow_telemetry', 0, 'red_balloon', [203, 203, 227, 237]),
    ('run_019_blue_slow_telemetry', 0, 'purple_balloon', [505, 208, 527, 239]),
    ('run_019_blue_slow_telemetry', 0, 'pillar_obstacle', [261, 0, 438, 301]),
    ('run_019_blue_slow_telemetry', 0, 'square_frame', [438, 90, 509, 260]),
    ('run_019_blue_slow_telemetry', 46, 'red_balloon', [95, 168, 151, 234]),
    ('run_019_blue_slow_telemetry', 182, 'square_frame', [269, 171, 425, 352]),
    ('run_019_blue_slow_telemetry', 182, 'blue_balloon', [350, 256, 371, 286]),
    ('run_019_blue_slow_telemetry', 182, 'ground_marker', [199, 300, 270, 325]),
    ('run_019_blue_slow_telemetry', 410, 'blue_balloon', [292, 244, 352, 325]),
]


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for split, entries in [('train', TRAIN), ('val', VAL)]:
        for run, frame, category, box in entries:
            image = f'{run}/frame_{frame:04d}.png'
            path = ROOT / 'recordings' / image
            with Image.open(path) as im:
                width, height = im.size
            x1, y1, x2, y2 = box
            assert 0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height, image
            rows.append({'image': image, 'category': category,
                         'expression': EXP[category], 'bbox': box, 'split': split,
                         'group_id': run, 'review_status': 'assistant_visual_review',
                         'source': 'original_simulator_png_manual_review_not_yolo_pseudolabel'})
    assert {r['group_id'] for r in rows if r['split'] == 'train'}.isdisjoint(
        {r['group_id'] for r in rows if r['split'] == 'val'})
    assert len({(r['image'], r['category']) for r in rows}) == len(rows)
    OUT.write_text(''.join(json.dumps(r, ensure_ascii=False) + '\n' for r in rows), encoding='utf-8')
    print(json.dumps({'output': str(OUT), 'train': len(TRAIN), 'val': len(VAL),
                      'by_category': dict(Counter(r['category'] for r in rows)),
                      'groups_disjoint': True}, ensure_ascii=False))


if __name__ == '__main__':
    main()
