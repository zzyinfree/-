#!/usr/bin/env python3
"""Read-only overview of recorded original camera frames for manual labeling."""
from pathlib import Path
import cv2
import numpy as np

root = Path('/root/gpufree-data/cicc_workspace/recordings')
out = Path('/root/gpufree-data/cicc_workspace/outputs/training_contact_sheets')
out.mkdir(parents=True, exist_ok=True)
for run in sorted(root.iterdir()):
    if not run.is_dir():
        continue
    frames = sorted(run.glob('frame_*.png'))
    if not frames:
        continue
    indices = sorted(set(round((len(frames) - 1) * i / 15) for i in range(16)))
    tiles = []
    for idx in indices:
        frame = cv2.imread(str(frames[idx]))
        if frame is None:
            continue
        frame = cv2.resize(frame, (320, 240))
        cv2.putText(frame, frames[idx].stem, (8, 23), cv2.FONT_HERSHEY_SIMPLEX,
                    .65, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, frames[idx].stem, (8, 23), cv2.FONT_HERSHEY_SIMPLEX,
                    .65, (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(frame)
    while len(tiles) < 16:
        tiles.append(np.zeros((240, 320, 3), np.uint8))
    sheet = np.vstack([np.hstack(tiles[i:i+4]) for i in range(0, 16, 4)])
    path = out / (run.name + '.jpg')
    cv2.imwrite(str(path), sheet, [cv2.IMWRITE_JPEG_QUALITY, 90])
    print(run.name, len(frames), path)
