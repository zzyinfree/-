#!/usr/bin/env python3
"""Evaluate captured simulator frames with the new instance's native YOLOE."""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

PKG = Path('/home/ubuntu/catkin_ws/src/object_det')
sys.path.insert(0, str(PKG / 'src'))
from object_det.detectors import YoloeDetector


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', required=True)
    args = parser.parse_args()
    directory = Path(args.directory)
    detector = YoloeDetector(str(PKG / 'models/best.pt'),
                             class_names=['square_frame', 'red_balloon'],
                             conf_thres=0.3, device='cuda:0', use_half=True,
                             max_det=20)
    thumbnails, results = [], []
    for path in sorted(directory.glob('frame_*.png')):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        rows = detector.predict(image)
        detections = []
        if rows is not None:
            for x1, y1, x2, y2, score, class_id in rows:
                box = [round(float(v)) for v in (x1, y1, x2, y2)]
                label = str(detector.names.get(int(class_id), 'unknown'))
                detections.append({'bbox': box, 'score': float(score), 'class_name': label})
                cv2.rectangle(image, tuple(box[:2]), tuple(box[2:]), (0, 255, 0), 2)
                cv2.putText(image, f'{label} {score:.2f}',
                            (box[0], max(18, box[1]-4)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 255, 0), 2)
        cv2.putText(image, path.stem, (6, image.shape[0]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        thumbnails.append(cv2.resize(image, (320, 240)))
        results.append({'image': path.name, 'detections': detections,
                        'label_status': 'pseudo_unverified'})
    if not thumbnails:
        raise RuntimeError('no frames')
    cols = 4
    rows = (len(thumbnails) + cols - 1) // cols
    sheet = np.zeros((rows * 240, cols * 320, 3), dtype=np.uint8)
    for i, thumb in enumerate(thumbnails):
        y, x = divmod(i, cols)
        sheet[y*240:(y+1)*240, x*320:(x+1)*320] = thumb
    cv2.imwrite(str(directory / 'contact_sheet.jpg'), sheet)
    (directory / 'yolo_pseudo_labels.json').write_text(
        json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'frames': len(results),
                      'frame_detected': sum(any(d['class_name']=='square_frame' for d in r['detections']) for r in results),
                      'balloon_detected': sum(any('balloon' in d['class_name'] for d in r['detections']) for r in results),
                      'contact_sheet': str(directory / 'contact_sheet.jpg')}))


if __name__ == '__main__':
    main()
