#!/usr/bin/env python3
"""Deterministic FM9G4B-V full-image <box> baseline per official PDF."""
import argparse
import json
import re
from pathlib import Path

import torch
from PIL import Image,ImageDraw
from transformers import AutoModel,AutoProcessor

MODEL='/home/ubuntu/model/FM9G4B-V'
BOX_RE=re.compile(r'<box>\s*\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\]\s*</box>')


def parse_box(raw):
    m=BOX_RE.search(raw)
    if not m:return None
    box=list(map(float,m.groups()))
    if not(0<=box[0]<box[2]<=1000 and 0<=box[1]<box[3]<=1000):return None
    return box


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--image',required=True)
    ap.add_argument('--output',required=True)
    ap.add_argument('--draw-output',required=True)
    args=ap.parse_args()
    image=Image.open(args.image).convert('RGB')
    w,h=image.size
    processor=AutoProcessor.from_pretrained(MODEL,trust_remote_code=True,local_files_only=True)
    model=AutoModel.from_pretrained(MODEL,trust_remote_code=True,local_files_only=True,
                                    torch_dtype=torch.bfloat16,attn_implementation='sdpa').eval().to('cuda')
    targets={'red_balloon':'红色气球','square_frame':'可穿越的方形框架'}
    results={}
    draw=ImageDraw.Draw(image)
    with torch.inference_mode():
        for key,name in targets.items():
            raw=str(model.chat(image=image,msgs=[{'role':'user','content':f'请框出图中的{name}。只输出一个 <box>。'}],
                               tokenizer=processor.tokenizer,processor=processor,
                               sampling=False,num_beams=1,max_new_tokens=64,
                               max_slice_nums=4,use_image_id=True))
            box=parse_box(raw)
            pixel=[round(box[i]*(w if i%2==0 else h)/1000) for i in range(4)] if box else None
            if pixel:draw.rectangle(pixel,outline='red',width=3)
            results[key]={'raw':raw,'normalized_box':box,'pixel_box':pixel,'parse_valid':box is not None}
            print(key,raw,flush=True)
    Path(args.output).write_text(json.dumps({'image':args.image,'results':results},ensure_ascii=False,indent=2),encoding='utf-8')
    image.save(args.draw_output)


if __name__=='__main__':main()
