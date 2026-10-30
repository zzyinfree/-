#!/usr/bin/env python3
"""Visual QA montage of provisional boxes; no label modification."""
import json
import math
from collections import defaultdict
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont

ROOT=Path('/root/gpufree-data/cicc_workspace')
rows=[json.loads(x) for x in (ROOT/'annotations/provisional_bbox_v1.jsonl').read_text(encoding='utf-8').splitlines() if x.strip()]
by_image=defaultdict(list)
for row in rows:by_image[row['image']].append(row)
thumb_w,thumb_h=640,510
cols=3
count=len(by_image)
canvas=Image.new('RGB',(cols*thumb_w,math.ceil(count/cols)*thumb_h),'white')
palette={'red_balloon':'red','blue_balloon':'blue','purple_balloon':'magenta',
         'square_frame':'lime','pillar_obstacle':'yellow','aruco_marker':'cyan'}
for i,(name,objects) in enumerate(sorted(by_image.items())):
    img=Image.open(ROOT/'recordings'/name).convert('RGB')
    draw=ImageDraw.Draw(img)
    for row in objects:
        color=palette[row['category']]
        box=row['bbox']
        draw.rectangle(box,outline=color,width=3)
        draw.text((box[0],max(0,box[1]-15)),row['category'],fill=color,stroke_width=1,stroke_fill='black')
    x=(i%cols)*thumb_w;y=(i//cols)*thumb_h
    canvas.paste(img,(x,y))
    ImageDraw.Draw(canvas).text((x+4,y+483),name+' '+objects[0]['split'],fill='black')
out=ROOT/'annotations/provisional_bbox_v1_montage.jpg'
canvas.save(out,quality=90)
print(out)
