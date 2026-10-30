"""Combine stage12 reviews with visually checked down-camera marker examples.

The selected square proposals were inspected on stage13_square_proposals.jpg.
Wrong candidates (blue wall outlines etc.) are excluded. Expand the dark-square
proposal to include its visible white border.
No YOLO pseudo-labels and no sortie leakage between train and val.
"""
import json
from pathlib import Path

HERE = Path(__file__).parent
ROOT = Path('/root/gpufree-data/cicc_workspace')
old = [json.loads(s) for s in (HERE/'annotations/stage12_reviewed_bbox.jsonl').read_text(
    encoding='utf-8').splitlines() if s.strip()]
data = json.loads((HERE/'stage13_square_proposals.json').read_text(encoding='utf-8'))
reviewed = {
    'run_024_purple_stage13': {78,79,80,81,82,88,89,90,91,92,93,94},
    'run_025_purple_stage13': {70,71,72,73,74,81,82,85,86,87},
}
added = []
for item in data:
    if item['frame'] not in reviewed[item['run']]:
        continue
    x1, y1, x2, y2 = item['bbox_candidate']
    pad = round(max(x2-x1, y2-y1)*0.075)
    box = [max(0, x1-pad), max(0, y1-pad), min(640, x2+pad), min(480, y2+pad)]
    run = item['run']
    added.append({'image': f'{run}/down/{Path(item["image"]).name}',
                  'category': 'ground_marker',
                  'expression': '请框出图中的地面二维码标识。只输出一个 <box>。',
                  'bbox': box,
                  'split': 'train' if run.startswith('run_024') else 'val',
                  'group_id': run,
                  'review_status': 'assistant_visual_review',
                  'source': 'original_down_camera_png_visual_review_of_square_proposal'})
rows = old+added
assert len(added) == 22
assert {r['group_id'] for r in rows if r['split']=='train'}.isdisjoint(
    {r['group_id'] for r in rows if r['split']=='val'})
out = HERE/'annotations/stage13_down_reviewed_bbox.jsonl'
out.write_text(''.join(json.dumps(r, ensure_ascii=False)+'\n' for r in rows), encoding='utf-8')
print(out, len(rows), len(added))
