#!/usr/bin/env python3
"""Prepare manually inspected, sortie-disjoint FM9G4B-V bbox experiment.

Boxes are assistant visual reviews of raw simulator PNGs, not human GT or YOLO
pseudo-labels. Keep the flight stack and base model untouched.
"""
import collections
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ANN = ROOT / 'annotations'
BALLOONS = [
    ('run_073_purple_sprint_return', 140, 'purple', (76,323,139,395), 'train'),
    ('run_073_purple_sprint_return', 160, 'purple', (317,239,352,287), 'train'),
    ('run_073_purple_sprint_return', 170, 'purple', (308,335,404,469), 'train'),
    ('run_073_purple_sprint_return', 200, 'purple', (217,317,280,401), 'train'),
    ('run_079_red30_search_chase', 130, 'red', (585,327,640,400), 'train'),
    ('run_079_red30_search_chase', 140, 'red', (310,304,346,352), 'train'),
    ('run_080_blue_xy_hold', 70, 'blue', (478,291,517,338), 'train'),
    ('run_080_blue_xy_hold', 170, 'blue', (219,227,285,316), 'train'),
    ('run_080_blue_xy_hold', 200, 'blue', (323,324,359,375), 'train'),
    ('run_074_purple_sprint_return', 180, 'purple', (323,255,366,313), 'val'),
    ('run_074_purple_sprint_return', 195, 'purple', (131,358,231,480), 'val'),
    ('run_074_purple_sprint_return', 255, 'purple', (282,157,337,231), 'val'),
    ('run_076_blue_smooth40', 75, 'blue', (325,285,355,325), 'val'),
    ('run_081_red30_fixed_anchor', 195, 'red', (460,199,517,267), 'val'),
    ('run_081_red30_fixed_anchor', 225, 'red', (67,181,195,334), 'val'),
]


def main():
    rows = []
    for group, frame, color, box, split in BALLOONS:
        image = f'{group}/front/frame_{frame:04}.png'
        rows.append(dict(image=image, category=f'{color}_balloon',
            expression=f'请框出图中的{"红" if color == "red" else "蓝" if color == "blue" else "紫"}色气球。只输出一个 <box>。',
            bbox=list(box), split=split, group_id=group,
            review_status='assistant_visual_review',
            source='original_simulator_png_assistant_manual_review_not_yolo_pseudolabel'))
    newfile = ANN / 'stage42_new_balloon_reviewed_bbox.jsonl'
    mergedfile = ANN / 'stage42_balanced_reviewed_bbox.jsonl'
    old = [json.loads(x) for x in (ANN / 'stage41_merged_reviewed_bbox.jsonl').read_text(encoding='utf-8').splitlines() if x]
    missing_old = [r['image'] for r in old if not (ROOT / 'recordings' / r['image']).is_file()]
    if missing_old:
        print(json.dumps({'missing_old_frames_excluded': missing_old}, ensure_ascii=False))
        old = [r for r in old if (ROOT / 'recordings' / r['image']).is_file()]
    markers = [r for r in old if r['split'] == 'train' and r['category'] == 'ground_marker']
    # Cap the dominant marker class while retaining multiple distinct sorties.
    by_group = collections.defaultdict(list)
    for row in markers:
        by_group[row['group_id']].append(row)
    selected = []
    while len(selected) < 12 and any(by_group.values()):
        for group in sorted(by_group):
            if by_group[group] and len(selected) < 12:
                selected.append(by_group[group].pop(0))
    combined = [r for r in old if r['split'] == 'val' or (r['split'] == 'train' and r['category'] != 'ground_marker')]
    combined += selected + rows
    keys = [(r['image'], r['category']) for r in combined]
    assert len(keys) == len(set(keys)), 'duplicate image-category'
    train_groups = {r['group_id'] for r in combined if r['split'] == 'train'}
    val_groups = {r['group_id'] for r in combined if r['split'] == 'val'}
    assert train_groups.isdisjoint(val_groups)
    for r in combined:
        assert r['image'].split('/')[0] == r['group_id']
        assert (ROOT / 'recordings' / r['image']).is_file(), r['image']
        x1,y1,x2,y2 = r['bbox']
        assert 0 <= x1 < x2 <= 640 and 0 <= y1 < y2 <= 480
    for path, data in ((newfile, rows), (mergedfile, combined)):
        payload = ''.join(json.dumps(r, ensure_ascii=False) + '\n' for r in data)
        path.write_text(payload, encoding='utf-8')
        print(json.dumps(dict(file=str(path), sha256=hashlib.sha256(payload.encode()).hexdigest(),
            split=dict(collections.Counter(r['split'] for r in data)),
            categories={f'{s}:{c}': n for (s,c),n in collections.Counter((r['split'],r['category']) for r in data).items()},
            train_groups=sorted(train_groups), val_groups=sorted(val_groups)), ensure_ascii=False, default=str))


if __name__ == '__main__':
    main()
