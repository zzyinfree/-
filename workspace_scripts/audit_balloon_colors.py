#!/usr/bin/env python3
"""Flag YOLO balloon class/color disagreements; proposals remain unverified."""
import argparse
import json
from collections import Counter
from pathlib import Path

import cv2

RANGES = {'red': [(0,10),(170,179)], 'blue':[(100,130)],
          'purple':[(130,170)]}


def classify(roi_bgr):
    hsv = cv2.cvtColor(roi_bgr,cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    scores = {}
    for color, ranges in RANGES.items():
        scores[color] = int(sum(((h>=lo)&(h<=hi)&(s>=80)&(v>=75)).sum()
                                for lo,hi in ranges))
    color = max(scores,key=scores.get)
    return (color if scores[color]>=30 else 'unknown'), scores


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--directory',required=True)
    args=ap.parse_args()
    root=Path(args.directory)
    rows=json.loads((root/'yolo_pseudo_labels.json').read_text(encoding='utf-8'))
    out=[]
    counts=Counter()
    for row in rows:
        image=cv2.imread(str(root/row['image']))
        if image is None: raise FileNotFoundError(row['image'])
        detections=[]
        for d in row['detections']:
            d=dict(d)
            if 'balloon' in d['class_name']:
                x1,y1,x2,y2=d['bbox']
                color,scores=classify(image[y1:y2,x1:x2])
                d['color_heuristic']=color
                d['color_pixel_counts']=scores
                d['proposed_class_name']=color+'_balloon' if color!='unknown' else 'unknown_balloon'
                d['color_disagrees_with_native_class']=d['proposed_class_name']!=d['class_name']
                counts[d['proposed_class_name']]+=1
            detections.append(d)
        out.append({'image':row['image'],'detections':detections,
                    'label_status':'pseudo_unverified'})
    (root/'color_audited_proposals.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'frames':len(out),'color_proposal_counts':dict(counts),
                      'note':'HSV suggestions only; no automatic ground-truth promotion'},ensure_ascii=False))


if __name__=='__main__':main()
