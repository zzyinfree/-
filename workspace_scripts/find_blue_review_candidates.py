#!/usr/bin/env python3
"""Rank blue color blobs for manual image review; never creates labels."""
import json
from pathlib import Path
import cv2
import numpy as np

root=Path('/root/gpufree-data/cicc_workspace/recordings/run_002_takeoff_land')
rows=[]
for path in sorted(root.glob('frame_*.png')):
    bgr=cv2.imread(str(path))
    if bgr is None:continue
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([100,90,70]),np.array([135,255,255]))
    n,labels,stats,centroids=cv2.connectedComponentsWithStats(mask)
    blobs=[]
    for x,y,w,h,area in stats[1:]:
        if 4<=w<=90 and 5<=h<=100 and area>=8:
            blobs.append({'xyxy':[int(x),int(y),int(x+w),int(y+h)],'area':int(area)})
    blobs.sort(key=lambda b:-b['area'])
    rows.append({'image':path.name,'top':blobs[:3]})
rows.sort(key=lambda r:-r['top'][0]['area'] if r['top'] else 0)
print(json.dumps(rows[:20],ensure_ascii=False,indent=2))
