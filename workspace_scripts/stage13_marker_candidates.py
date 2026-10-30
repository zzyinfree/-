#!/usr/bin/env python3
"""Generate candidate labels/contact sheet; detections are NOT ground truth until reviewed."""
import cv2
import glob
import json
import os
import sys

root = sys.argv[1]
out = sys.argv[2]
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
params = cv2.aruco.DetectorParameters_create()
rows = []
for run in ['run_024_purple_stage13', 'run_025_purple_stage13']:
    for path in sorted(glob.glob(f'{root}/recordings/{run}/down/frame*.png')):
        image = cv2.imread(path)
        corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=params)
        if ids is None:
            continue
        for poly, mid in zip(corners, ids.flatten()):
            pts = poly.reshape(4, 2)
            x1, y1 = pts.min(axis=0)
            x2, y2 = pts.max(axis=0)
            if x2-x1 < 25 or y2-y1 < 25:
                continue
            rows.append({'run': run, 'image': path, 'id': int(mid),
                         'bbox_candidate': [round(float(x1)), round(float(y1)),
                                            round(float(x2)), round(float(y2))]})

selected = []
for run in ['run_024_purple_stage13', 'run_025_purple_stage13']:
    sub = [r for r in rows if r['run'] == run and r['id'] == 35]
    step = max(1, len(sub)//18)
    selected.extend(sub[::step][:18])
tiles = []
for i, row in enumerate(selected):
    im = cv2.imread(row['image'])
    x1, y1, x2, y2 = row['bbox_candidate']
    cv2.rectangle(im, (x1, y1), (x2, y2), (0, 255, 0), 3)
    im = cv2.resize(im, (320, 240))
    cv2.putText(im, f'{i} {row["run"][4:7]} {os.path.basename(row["image"])}',
                (4, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    tiles.append(im)
while tiles and len(tiles)%4:
    tiles.append(tiles[-1].copy())
if tiles:
    sheet = cv2.vconcat([cv2.hconcat(tiles[i:i+4]) for i in range(0, len(tiles), 4)])
    cv2.imwrite(out+'.jpg', sheet)
with open(out+'.json', 'w', encoding='utf-8') as f:
    json.dump({'detected': len(rows), 'selected': selected,
               'label_status': 'cv2_proposals_unreviewed'}, f, indent=2)
print('detected', len(rows), 'selected', len(selected))
