#!/usr/bin/env python3
"""Print HSV component candidates for manual full-frame bbox review only."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

RANGES = {
    'red_balloon': [(0, 11), (170, 179)],
    'blue_balloon': [(100, 130)],
    'purple_balloon': [(130, 170)],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--recordings', type=Path, required=True)
    ap.add_argument('--images', nargs='+', required=True)
    args = ap.parse_args()
    for rel in args.images:
        img = cv2.imread(str(args.recordings / rel))
        if img is None:
            raise FileNotFoundError(rel)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        for name, ranges in RANGES.items():
            mask = np.zeros(h.shape, dtype=np.uint8)
            for lo, hi in ranges:
                mask |= ((h >= lo) & (h <= hi) & (s >= 80) & (v >= 80)).astype(np.uint8)
            n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
            comps = []
            for i in range(1, n):
                x, y, w, hh, area = map(int, stats[i])
                if area >= 50:
                    comps.append({'bbox': [x, y, x + w, y + hh], 'pixels': area})
            comps.sort(key=lambda c: c['pixels'], reverse=True)
            if comps:
                print(json.dumps({'image': rel, 'class': name, 'candidates': comps[:3]}, ensure_ascii=False))


if __name__ == '__main__':
    main()
