#!/usr/bin/env python3
"""Propose color-mask bbox candidates from raw simulator frames; never ground truth.

This script is for annotation triage only. Its output requires visual review before
training/evaluation, and must never be represented as model detections or score.
"""
import argparse
import json
from pathlib import Path

import cv2

HSV = {
    "red_balloon": [((0, 95, 80), (12, 255, 255)), ((170, 95, 80), (179, 255, 255))],
    "blue_balloon": [((95, 95, 70), (132, 255, 255))],
    "purple_balloon": [((132, 75, 60), (169, 255, 255))],
}


def candidates(path):
    image = cv2.imread(str(path))
    if image is None:
        return []
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    height, width = image.shape[:2]
    rows = []
    for category, ranges in HSV.items():
        mask = None
        for lo, hi in ranges:
            partial = cv2.inRange(hsv, lo, hi)
            mask = partial if mask is None else cv2.bitwise_or(mask, partial)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            ratio = w / h
            fill = area / (w * h)
            if w < 10 or h < 10 or area < 70 or not 0.55 <= ratio <= 1.7 or fill < 0.42:
                continue
            if x <= 0 or y <= 0 or x + w >= width or y + h >= height:
                continue
            rows.append({"image": str(path), "category": category,
                         "proposal_bbox": [x, y, x + w, y + h],
                         "area": round(area), "fill": round(fill, 3)})
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recordings", type=Path, required=True)
    ap.add_argument("--groups", nargs="+", required=True)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    result = []
    for group in args.groups:
        frames = sorted((args.recordings / group).glob("frame_*.png"))
        for path in frames[::args.stride]:
            for row in candidates(path):
                row["image"] = str(path.relative_to(args.recordings))
                row["group_id"] = group
                row["review_status"] = "unverified_color_proposal"
                result.append(row)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in result), encoding="utf-8")
    counts = {c: sum(r["category"] == c for r in result) for c in HSV}
    print(json.dumps({"total": len(result), "by_category": counts, "output": str(args.output)}))


if __name__ == "__main__":
    main()
