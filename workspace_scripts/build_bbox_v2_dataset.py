#!/usr/bin/env python3
"""Extend v1 with visually reviewed, original simulator frames.

Boxes below were inspected on 640x480 originals. This is assistant visual
review, not organizer ground truth. Color-mask proposals are NOT imported.
"""
import json
from pathlib import Path

ROOT = Path('/root/gpufree-data/cicc_workspace')
OLD = ROOT / 'annotations/provisional_bbox_v1.jsonl'
OUT = ROOT / 'annotations/reviewed_bbox_v2.jsonl'
PROMPT = {
    'red_balloon': '请定位画面中的红色气球，只输出目标的 <box> 边界框，坐标归一化到0到1000。',
    'blue_balloon': '找出图像里的蓝色气球。仅返回一个 <box>[x1,y1,x2,y2]</box>。',
    'purple_balloon': '请框选紫色气球，只给出一个归一化的 <box>。',
    'square_frame': '请框出可穿越的方形框架，仅输出一个 <box>。',
    'pillar_obstacle': '请指出砖墙障碍柱的位置，只输出一个 <box>。',
}

# (capture group, frame index, category, pixel xyxy, split)
EXTRA = [
    ('run_004_prompt_red', 45, 'red_balloon', [331, 266, 364, 311], 'train'),
    ('run_004_prompt_red', 45, 'pillar_obstacle', [188, 7, 255, 324], 'train'),
    ('run_005_prompt_blue', 85, 'blue_balloon', [298, 279, 325, 316], 'train'),
    ('run_005_prompt_blue', 85, 'square_frame', [31, 36, 432, 440], 'train'),
    ('run_005_prompt_blue', 135, 'blue_balloon', [274, 281, 305, 324], 'train'),
    ('run_005_prompt_blue', 145, 'blue_balloon', [130, 291, 184, 356], 'train'),
    ('run_005_prompt_blue', 150, 'blue_balloon', [181, 325, 265, 433], 'train'),
    ('run_008_purple_corrected', 15, 'purple_balloon', [525, 201, 557, 236], 'train'),
    ('run_008_purple_corrected', 15, 'pillar_obstacle', [183, 0, 390, 311], 'train'),
    ('run_008_purple_corrected', 75, 'purple_balloon', [432, 284, 464, 326], 'train'),
    ('run_008_purple_corrected', 75, 'square_frame', [208, 165, 381, 414], 'train'),
    ('run_008_purple_corrected', 115, 'purple_balloon', [99, 308, 132, 347], 'train'),
    ('run_008_purple_corrected', 115, 'pillar_obstacle', [157, 29, 243, 414], 'train'),
    ('run_007_blue_guarded', 95, 'blue_balloon', [366, 283, 397, 326], 'val'),
    ('run_007_blue_guarded', 100, 'blue_balloon', [210, 297, 251, 349], 'val'),
    ('run_007_blue_guarded', 155, 'blue_balloon', [541, 298, 604, 365], 'val'),
    ('run_009_purple_center_fix', 210, 'purple_balloon', [248, 237, 292, 294], 'val'),
    ('run_009_purple_center_fix', 215, 'purple_balloon', [145, 146, 287, 330], 'val'),
]


def main():
    rows = [json.loads(line) for line in OLD.read_text(encoding='utf-8').splitlines() if line.strip()]
    from PIL import Image
    for group, index, category, box, split in EXTRA:
        image = f'{group}/frame_{index:04d}.png'
        path = ROOT / 'recordings' / image
        with Image.open(path) as im:
            assert im.size == (640, 480), (path, im.size)
        assert 0 <= box[0] < box[2] <= 640 and 0 <= box[1] < box[3] <= 480
        rows.append(dict(image=image, category=category, expression=PROMPT[category],
                         bbox=box, split=split, group_id=group,
                         review_status='assistant_visual_review',
                         source='original_simulation_png_manual_visual_bbox_not_yolo'))
    train_groups = {r['group_id'] for r in rows if r['split'] == 'train'}
    val_groups = {r['group_id'] for r in rows if r['split'] == 'val'}
    assert not train_groups & val_groups, train_groups & val_groups
    assert all(r['review_status'] in ('assistant_visual_review', 'human_verified') for r in rows)
    OUT.write_text(''.join(json.dumps(row, ensure_ascii=False) + '\n' for row in rows), encoding='utf-8')
    print(json.dumps({'out': str(OUT), 'train': sum(r['split'] == 'train' for r in rows),
                      'val': sum(r['split'] == 'val' for r in rows),
                      'train_groups': sorted(train_groups), 'val_groups': sorted(val_groups)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
