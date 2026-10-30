#!/usr/bin/env python3
"""Render every manual label for QA; no model predictions or auto-labeling."""
import json
from pathlib import Path
import cv2
import numpy as np

root = Path('/root/gpufree-data/cicc_workspace')
rows = [json.loads(line) for line in (root/'annotations/stage12_reviewed_bbox.jsonl').read_text(encoding='utf-8').splitlines()]
for split in ('train', 'val'):
    tiles = []
    for row in rows:
        if row['split'] != split:
            continue
        frame = cv2.imread(str(root/'recordings'/row['image']))
        x1, y1, x2, y2 = row['bbox']
        cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
        cv2.putText(frame, row['category'], (max(0,x1), max(17,y1-4)), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(frame, row['category'], (max(0,x1), max(17,y1-4)), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, (255,255,255), 1, cv2.LINE_AA)
        frame = cv2.resize(frame, (320,240))
        title = row['image'].split('/')[0].split('_')[1] + '/' + row['image'].split('/')[-1]
        cv2.putText(frame, title, (5,232), cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1,cv2.LINE_AA)
        tiles.append(frame)
    while len(tiles) % 4:
        tiles.append(np.zeros((240,320,3), np.uint8))
    sheet = np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,len(tiles),4)])
    target = root/'outputs'/f'stage12_{split}_label_review.jpg'
    cv2.imwrite(str(target), sheet, [cv2.IMWRITE_JPEG_QUALITY,93])
    print(target)
