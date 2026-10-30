#!/usr/bin/env python3
"""Build visually reviewed down-camera bbox data, separated by sortie.

Dark-square contour boxes were visually checked against the full-frame montage;
they are assistant-reviewed labels, not human ground truth. FM9G inference and
evaluation must always consume the original full RGB image, never contours.
"""
import json
from pathlib import Path

ROOT = Path('/root/gpufree-data/cicc_workspace')
SPEC = {
    'run_027_red_clone_stage14': (91, 92, 93, 94, 95, 97, 98, 99, 100, 101, 102, 103, 104,
                                  106, 107, 108, 109, 110, 111),
    'run_028_purple_left_stage14': (92, 94, 95, 96, 97, 98, 99, 105, 108),
}
MANUAL_BOXES = {
    # Full-resolution 640x480 PNGs inspected individually, 2026-10-17.
    ('run_027_red_clone_stage14', 97): [228, 94, 449, 314],
    ('run_027_red_clone_stage14', 98): [225, 106, 428, 309],
    ('run_027_red_clone_stage14', 99): [228, 124, 401, 298],
    ('run_027_red_clone_stage14', 100): [221, 131, 393, 300],
    ('run_027_red_clone_stage14', 101): [198, 121, 399, 318],
}

def main():
    source = json.loads((ROOT / 'outputs' / 'stage14_down_proposals.json').read_text())
    proposed = {(r['run'], r['frame']): r for r in source}
    # Keep previous reviewed non-down training images so the model does not
    # learn to predict the marker for every possible image.
    previous = [json.loads(s) for s in (ROOT / 'annotations' / 'stage12_reviewed_bbox.jsonl').read_text().splitlines() if s.strip()]
    rows = [r for r in previous if r['split'] == 'train']
    for run, numbers in SPEC.items():
        for num in numbers:
            item = proposed[(run, num)]
            box = MANUAL_BOXES.get((run, num), item['bbox_candidate'])
            if box is None:
                raise ValueError((run, num))
            rel = f'{run}/down/frame_{num:04d}.png'
            rows.append({'image': rel, 'category': 'ground_marker',
                         'expression': '请框出图中的地面二维码标识。只输出一个 <box>。',
                         'bbox': box,
                         'split': 'train' if run.startswith('run_027') else 'val',
                         'group_id': run,
                         'review_status': 'assistant_visual_review',
                         'source': 'full_simulator_png_manual_or_dark_contour_visual_review_not_human_gt'})
    out = ROOT / 'annotations' / 'stage14_down_reviewed_bbox.jsonl'
    out.write_text(''.join(json.dumps(r, ensure_ascii=False)+'\n' for r in rows), encoding='utf-8')
    print(json.dumps({'output': str(out), 'train': sum(r['split']=='train' for r in rows),
                      'val': sum(r['split']=='val' for r in rows)}, ensure_ascii=False))

if __name__ == '__main__':
    main()
