#!/usr/bin/env python3
"""Transcribed, visually reviewed assistant annotations; NOT human-verified GT.

All coordinates are pixel xyxy in original 640x480 images. They were inspected
against the corresponding PNGs, not copied from YOLO pseudo-labels. Ambiguous
or heavily occluded objects are intentionally omitted.
"""
import json
from pathlib import Path

OUT = Path('/root/gpufree-data/cicc_workspace/annotations/provisional_bbox_v1.jsonl')
EXP = {
    'pillar_obstacle': '请框出图中的砖墙障碍柱。只输出一个 <box>。',
    'square_frame': '请框出图中的方形穿越框。只输出一个 <box>。',
    'aruco_marker': '请框出地面的ArUco二维码标记。只输出一个 <box>。',
    'red_balloon': '请框出图中的红色气球。只输出一个 <box>。',
    'blue_balloon': '请框出图中的蓝色气球。只输出一个 <box>。',
    'purple_balloon': '请框出图中的紫色气球。只输出一个 <box>。',
}

# Each tuple is (frame index, category, visually inspected pixel box).
RUN1 = [
    (0,'red_balloon',[19,148,92,226]),
    (0,'pillar_obstacle',[187,0,396,311]),
    (0,'square_frame',[395,90,517,226]),
    (12,'red_balloon',[24,151,97,227]),
    (12,'purple_balloon',[573,198,611,237]),
    (12,'square_frame',[395,90,517,226]),
    (36,'red_balloon',[78,165,134,228]),
]
RUN3 = [
    (0,'red_balloon',[72,161,133,230]),
    (0,'pillar_obstacle',[209,0,399,306]),
    (24,'red_balloon',[61,156,122,226]),
    (36,'blue_balloon',[454,246,476,272]),
    (36,'square_frame',[372,157,513,305]),
    (48,'blue_balloon',[257,258,277,283]),
    (48,'square_frame',[303,177,432,306]),
    (48,'aruco_marker',[211,296,287,320]),
    (52,'red_balloon',[158,268,181,295]),
    (52,'blue_balloon',[190,262,211,289]),
    (52,'purple_balloon',[466,267,490,296]),
    (52,'square_frame',[303,172,443,312]),
    (54,'red_balloon',[144,271,170,303]),
    (54,'blue_balloon',[208,266,230,295]),
    (54,'purple_balloon',[496,276,529,316]),
    (54,'square_frame',[296,167,456,332]),
    (56,'red_balloon',[113,276,148,311]),
    (56,'blue_balloon',[232,270,254,301]),
    (56,'purple_balloon',[531,287,573,332]),
    (56,'square_frame',[289,159,467,338]),
    (56,'aruco_marker',[173,325,271,353]),
    (60,'red_balloon',[47,289,91,335]),
    (60,'blue_balloon',[304,271,329,305]),
    (60,'square_frame',[280,143,497,360]),
    (60,'aruco_marker',[148,323,271,362]),
    (84,'blue_balloon',[353,308,403,375]),
]
RUN2 = [
    (0,'purple_balloon',[581,194,618,236]),
    (0,'square_frame',[396,91,515,225]),
    (0,'pillar_obstacle',[201,0,399,307]),
    (10,'red_balloon',[106,172,156,231]),
    (10,'purple_balloon',[522,204,551,238]),
    (10,'square_frame',[396,91,515,225]),
    (20,'purple_balloon',[524,204,555,238]),
    (43,'blue_balloon',[394,207,414,238]),
    (60,'red_balloon',[154,187,194,233]),
    (70,'square_frame',[400,118,513,260]),
    (70,'pillar_obstacle',[217,0,403,393]),
]


def rows(run, split, group_id, labels):
    for frame, category, box in labels:
        verified = run == 'run_001' and frame == 36 and category == 'red_balloon'
        yield {'image': f'{run}/frame_{frame:04d}.png',
               'category': category, 'expression': EXP[category],
               'bbox': box, 'split': split, 'group_id': group_id,
               'review_status': 'human_verified' if verified else 'assistant_visual_review',
               'source': 'original_simulation_png_not_yolo_pseudolabel'}


def main():
    data = (list(rows('run_001','train','run_001_static_20261015',RUN1))+
            list(rows('run_003_observation_route','train','run_003_route_20261015',RUN3))+
            list(rows('run_002_takeoff_land','val','run_002_takeoff_land_20261015',RUN2)))
    OUT.parent.mkdir(parents=True,exist_ok=True)
    OUT.write_text(''.join(json.dumps(row,ensure_ascii=False)+'\n' for row in data),encoding='utf-8')
    print(json.dumps({'path':str(OUT),'rows':len(data),'train':len(RUN1)+len(RUN3),
                      'val':len(RUN2),'review_status':'1 human_verified + assistant_visual_review'},ensure_ascii=False))


if __name__=='__main__':main()
