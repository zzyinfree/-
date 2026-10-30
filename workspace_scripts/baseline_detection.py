#!/usr/bin/env python3
"""Run the new instance's untouched competition detector on one saved image.

This is an offline diagnostic: no ROS, simulator, or flight commands.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2

PKG = Path('/home/ubuntu/catkin_ws/src/object_det')
sys.path.insert(0, str(PKG / 'src'))
from object_det.detectors import YoloeDetector


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--image', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--conf', type=float, default=0.5)
    args = p.parse_args()
    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(args.image)
    detector = YoloeDetector(
        str(PKG / 'models/best.pt'),
        class_names=['square_frame', 'red_balloon'],
        conf_thres=args.conf,
        device='cuda:0',
        use_half=True,
        max_det=20,
    )
    rows = detector.predict(image)
    objects = []
    if rows is not None:
        for x1, y1, x2, y2, score, class_id in rows:
            box = [round(float(x)) for x in (x1, y1, x2, y2)]
            objects.append({'bbox': box, 'score': float(score),
                            'class_id': int(class_id),
                            'class_name': str(detector.names.get(int(class_id), 'unknown'))})
            cv2.rectangle(image, tuple(box[:2]), tuple(box[2:]), (0, 255, 0), 2)
            cv2.putText(image, objects[-1]['class_name'], (box[0], max(20, box[1]-5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), image):
        raise OSError(output)
    result = {'image': args.image, 'conf': args.conf, 'names': detector.names,
              'objects': objects, 'annotated_image': str(output)}
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
