#!/usr/bin/env python3
"""Create a target image and JSON from a real simulator frame, offline."""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

PKG = Path('/home/ubuntu/catkin_ws/src/object_det')
sys.path.insert(0, str(PKG / 'src'))
from object_det.detectors import YoloeDetector

RANGES = {
    'red': [((0, 100, 80), (10, 255, 255)), ((170, 100, 80), (180, 255, 255))],
    'blue': [((100, 100, 80), (130, 255, 255))],
    'purple': [((130, 60, 80), (170, 255, 255))],
}
DRAW = {'red': (0, 0, 255), 'blue': (255, 0, 0),
        'purple': (255, 0, 255), 'square_frame': (0, 255, 0)}


def classify_balloon(bgr, box):
    x1, y1, x2, y2 = box
    roi = bgr[y1:y2, x1:x2]
    if roi.size == 0:
        return 'unknown', None, 0
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    best = ('unknown', None, 0)
    for name, bounds in RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in bounds:
            mask |= cv2.inRange(hsv, low, high)
        count = cv2.countNonZero(mask)
        if count < 30 or count <= best[2]:
            continue
        moment = cv2.moments(mask)
        center = [round(x1 + moment['m10']/moment['m00']),
                  round(y1 + moment['m01']/moment['m00'])]
        best = (name, center, count)
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True)
    parser.add_argument('--output-image', required=True)
    parser.add_argument('--output-json', required=True)
    parser.add_argument('--conf', type=float, default=0.3)
    args = parser.parse_args()
    bgr = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(args.image)
    detector = YoloeDetector(str(PKG / 'models/best.pt'),
                             class_names=['square_frame', 'red_balloon'],
                             conf_thres=args.conf, device='cuda:0',
                             use_half=True, max_det=20)
    rows = detector.predict(bgr)
    annotated = bgr.copy()
    objects = []
    if rows is not None:
        for x1, y1, x2, y2, score, class_id in rows:
            box = [round(float(v)) for v in (x1, y1, x2, y2)]
            native_class = str(detector.names.get(int(class_id), 'unknown'))
            if native_class == 'square_frame':
                color, center = None, [(box[0]+box[2])//2, (box[1]+box[3])//2]
                label = 'square_frame'
            elif 'balloon' in native_class:
                color, center, _ = classify_balloon(bgr, box)
                if color == 'unknown':
                    continue
                label = f'{color}_balloon'
            else:
                continue
            obj = {'class_name': label, 'color': color, 'bbox': box,
                   'center': center, 'score': float(score)}
            objects.append(obj)
            draw_color = DRAW.get(color or label, (255, 255, 255))
            cv2.rectangle(annotated, tuple(box[:2]), tuple(box[2:]), draw_color, 2)
            cv2.drawMarker(annotated, tuple(center), draw_color,
                           cv2.MARKER_CROSS, 16, 2)
            cv2.putText(annotated, f'{label} {score:.2f}',
                        (box[0], max(18, box[1]-5)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, draw_color, 2)
    out_image = Path(args.output_image)
    out_json = Path(args.output_json)
    out_image.parent.mkdir(parents=True, exist_ok=True)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out_image), annotated):
        raise OSError(out_image)
    result = {'source_image': args.image, 'status': 'targets_found' if objects else 'no_target',
              'objects': objects, 'annotated_image': str(out_image)}
    out_json.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
