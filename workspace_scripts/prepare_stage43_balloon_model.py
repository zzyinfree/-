#!/usr/bin/env python3
"""Add visually inspected blue/purple raw frames; preserve stage42 val exactly."""
import collections
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ANN = ROOT / 'annotations'
# Assistant-verified pixel boxes on original 640x480 front-camera PNGs.
# Deliberately exclude heavily occluded/clipped proposals.
EXTRA = [
    ('run_071_purple_demo', 96, 'purple', (492,397,533,447)),
    ('run_071_purple_demo', 120, 'purple', (4,317,60,374)),
    ('run_071_purple_demo', 156, 'purple', (375,257,426,329)),
    ('run_071_purple_demo', 180, 'purple', (331,262,416,371)),
    ('run_075_purple_smooth40', 144, 'purple', (245,220,295,287)),
    ('run_075_purple_smooth40', 168, 'purple', (309,317,340,360)),
    ('run_075_purple_smooth40', 180, 'purple', (266,320,290,354)),
    ('run_032_blue_userwatch', 70, 'blue', (230,293,263,338)),
    ('run_032_blue_userwatch', 168, 'blue', (557,279,611,338)),
    ('run_032_blue_userwatch', 182, 'blue', (339,256,370,300)),
    ('run_032_blue_userwatch', 210, 'blue', (234,323,314,434)),
    ('run_068_blue_full_return', 72, 'blue', (119,298,164,351)),
    ('run_068_blue_full_return', 144, 'blue', (259,249,288,287)),
    ('run_068_blue_full_return', 156, 'blue', (290,238,315,271)),
    ('run_078_blue_postbrake', 60, 'blue', (202,267,224,294)),
    ('run_078_blue_postbrake', 72, 'blue', (423,279,448,310)),
]


def main():
    oldpath = ANN / 'stage42_balanced_reviewed_bbox.jsonl'
    old = [json.loads(s) for s in oldpath.read_text(encoding='utf-8').splitlines() if s]
    new = []
    for group, frame, color, box in EXTRA:
        new.append(dict(image=f'{group}/front/frame_{frame:04}.png',
            category=f'{color}_balloon',
            expression=f'请框出图中的{"蓝" if color == "blue" else "紫"}色气球。只输出一个 <box>。',
            bbox=list(box), split='train', group_id=group,
            review_status='assistant_visual_review',
            source='original_simulator_png_assistant_manual_review_not_yolo_pseudolabel'))
    combined = old + new
    assert len({(r['image'],r['category']) for r in combined}) == len(combined)
    assert {r['group_id'] for r in combined if r['split']=='train'}.isdisjoint(
        {r['group_id'] for r in combined if r['split']=='val'})
    for row in combined:
        assert row['group_id'] == row['image'].split('/')[0]
        assert (ROOT / 'recordings' / row['image']).is_file(), row['image']
        x1,y1,x2,y2 = row['bbox']
        assert 0 <= x1 < x2 <= 640 and 0 <= y1 < y2 <= 480
    assert [r for r in combined if r['split']=='val'] == [r for r in old if r['split']=='val']
    for name, rows in [('stage43_new_blue_purple_reviewed_bbox.jsonl',new),
                       ('stage43_balanced_reviewed_bbox.jsonl',combined)]:
        payload = ''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows)
        dest = ANN / name
        dest.write_text(payload,encoding='utf-8')
        print(json.dumps({'path':str(dest),'sha256':hashlib.sha256(payload.encode()).hexdigest(),
            'rows':len(rows),'counts':{f'{s}:{c}':n for (s,c),n in
            collections.Counter((r['split'],r['category']) for r in rows).items()}},
            ensure_ascii=False))


if __name__ == '__main__':
    main()
