#!/usr/bin/env python3
"""Quality-gate simulator captures. Never promotes YOLO proposals to truth."""
import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--directory', required=True)
    args = ap.parse_args()
    root = Path(args.directory)
    manifest = json.loads((root/'capture_manifest.json').read_text(encoding='utf-8'))
    pseudo_path = root/'yolo_pseudo_labels.json'
    pseudo = {x['image']: x['detections'] for x in json.loads(pseudo_path.read_text(encoding='utf-8'))} if pseudo_path.exists() else {}
    rows = []
    for record in manifest['frames']:
        path = root/record['image']
        raw = path.read_bytes()
        checksum = hashlib.sha256(raw).hexdigest()
        if 'sha256' in record and checksum != record['sha256']:
            raise ValueError(f'checksum mismatch: {path}')
        image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'unreadable: {path}')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_std = float(gray.std())
        edge_fraction = float((cv2.Canny(gray,80,160)>0).mean())
        # A uniform wall/floor view is ambiguous: neither a clean negative nor a usable positive.
        suspect_clipped = gray_std < 15 and edge_fraction < 0.003
        h,w = gray.shape
        detections = pseudo.get(record['image'],[])
        bad_boxes = []
        for det in detections:
            x1,y1,x2,y2 = det['bbox']
            if not (0 <= x1 < x2 <= w and 0 <= y1 < y2 <= h):
                bad_boxes.append(det)
        rows.append({'image':record['image'],'sha256':checksum,
                     'group_id':manifest.get('group_id','unknown_legacy'),
                     'pose':record.get('pose'), 'gray_std':round(gray_std,3),
                     'edge_fraction':round(edge_fraction,5),
                     'quality':'suspect_clipped_or_uniform' if suspect_clipped else 'review_candidate',
                     'pseudo_detection_count':len(detections),
                     'pseudo_invalid_box_count':len(bad_boxes),
                     'label_status':'unverified'})
    counts = Counter(x['quality'] for x in rows)
    summary = {'group_id':manifest.get('group_id','unknown_legacy'),
               'total':len(rows),'quality_counts':dict(counts),
               'unique_hashes':len({x['sha256'] for x in rows}),
               'pseudo_invalid_boxes':sum(x['pseudo_invalid_box_count'] for x in rows),
               'first_suspect':next((x['image'] for x in rows if x['quality'] != 'review_candidate'),None),
               'note':'Heuristic quality gate only. All object labels remain unverified.'}
    (root/'quality_audit.json').write_text(json.dumps({'summary':summary,'frames':rows},ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False))


if __name__ == '__main__':
    main()
