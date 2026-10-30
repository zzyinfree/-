#!/usr/bin/env python3
"""Validate human-reviewed pixel-xyxy JSONL before PDF-style COCO conversion."""
import argparse
import hashlib
import json
from collections import Counter,defaultdict
from pathlib import Path

from PIL import Image

CATEGORIES={'pillar_obstacle','square_frame','aruco_marker','landing_pad',
            'red_balloon','blue_balloon','purple_balloon','smoke_projectile'}


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--input',required=True)
    ap.add_argument('--image-root',required=True)
    ap.add_argument('--allow-assistant-reviewed',action='store_true',
                    help='PROVISIONAL training only; default remains human_verified-only')
    args=ap.parse_args()
    root=Path(args.image_root).resolve()
    groups=defaultdict(set)
    seen={}
    counts=Counter()
    for line_num,line in enumerate(Path(args.input).read_text(encoding='utf-8').splitlines(),1):
        if not line.strip():continue
        row=json.loads(line)
        needed={'image','category','expression','bbox','split','group_id','review_status'}
        missing=needed-set(row)
        if missing:raise ValueError(f'line {line_num}: missing {sorted(missing)}')
        accepted={'human_verified'}
        if args.allow_assistant_reviewed:accepted.add('assistant_visual_review')
        if row['review_status'] not in accepted:
            raise ValueError(f'line {line_num}: review status {row["review_status"]} not accepted')
        counts[('review_status',row['review_status'])]+=1
        if row['category'] not in CATEGORIES:raise ValueError(f'line {line_num}: unknown category')
        if row['split'] not in ('train','val'):raise ValueError(f'line {line_num}: bad split')
        if not isinstance(row['expression'],str) or not row['expression'].strip():
            raise ValueError(f'line {line_num}: empty expression')
        path=(root/row['image']).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise ValueError(f'line {line_num}: unsafe/missing image {path}')
        with Image.open(path) as im:w,h=im.size
        box=row['bbox']
        if not isinstance(box,list) or len(box)!=4 or any(type(x) not in (int,float) for x in box):
            raise ValueError(f'line {line_num}: bad xyxy')
        x1,y1,x2,y2=box
        if not(0<=x1<x2<=w and 0<=y1<y2<=h):
            raise ValueError(f'line {line_num}: out-of-bounds xyxy')
        if row['category'].endswith('_balloon') and (x2-x1)*(y2-y1)<64:
            raise ValueError(f'line {line_num}: tiny balloon requires explicit review')
        groups[row['group_id']].add(row['split'])
        img_hash=hashlib.sha256(path.read_bytes()).hexdigest()
        if img_hash in seen and seen[img_hash]!=row['split']:
            raise ValueError(f'line {line_num}: identical image across splits')
        seen[img_hash]=row['split']
        counts[(row['split'],row['category'])]+=1
    leaking={g:s for g,s in groups.items() if len(s)>1}
    if leaking:raise ValueError(f'group split leakage: {leaking}')
    print(json.dumps({'groups':{g:list(s)[0] for g,s in groups.items()},
                      'counts':{f'{s}/{c}':n for (s,c),n in counts.items()},
                      'unique_images':len(seen),'validation':'PASS' if counts else 'EMPTY'},
                     ensure_ascii=False,indent=2))


if __name__=='__main__':main()
