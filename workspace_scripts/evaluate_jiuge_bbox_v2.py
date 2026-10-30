#!/usr/bin/env python3
"""Strict official-format image+prompt bbox eval on disjoint v2 capture groups."""
import argparse
import json
import re
import time
from pathlib import Path

import torch
from PIL import Image
from transformers import AutoModel,AutoProcessor

ROOT=Path('/root/gpufree-data/cicc_workspace')
MODEL='/home/ubuntu/model/FM9G4B-V'
PAT=re.compile(r'<box>\s*\[\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*\]\s*</box>')


def box_from_raw(raw,w,h):
    match=PAT.fullmatch(raw.strip())
    if not match:return None
    v=list(map(float,match.groups()))
    if not(0<=v[0]<v[2]<=1000 and 0<=v[1]<v[3]<=1000):return None
    return [v[0]*w/1000,v[1]*h/1000,v[2]*w/1000,v[3]*h/1000]


def iou(a,b):
    x1=max(a[0],b[0]);y1=max(a[1],b[1]);x2=min(a[2],b[2]);y2=min(a[3],b[3])
    inter=max(0,x2-x1)*max(0,y2-y1)
    aa=(a[2]-a[0])*(a[3]-a[1]);bb=(b[2]-b[0])*(b[3]-b[1])
    return inter/(aa+bb-inter) if aa+bb-inter else 0


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--adapter',default='')
    ap.add_argument('--split',choices=('train','val'),default='val')
    ap.add_argument('--output',required=True)
    ap.add_argument('--data',default=str(ROOT/'annotations/reviewed_bbox_v2.jsonl'))
    args=ap.parse_args()
    rows=[json.loads(s) for s in Path(args.data).read_text(encoding='utf-8').splitlines() if s.strip()]
    rows=[r for r in rows if r['split']==args.split]
    processor=AutoProcessor.from_pretrained(MODEL,trust_remote_code=True,local_files_only=True)
    base=AutoModel.from_pretrained(MODEL,trust_remote_code=True,local_files_only=True,
        attn_implementation='sdpa',torch_dtype=torch.bfloat16).eval().to('cuda')
    if args.adapter:
        from peft import PeftModel
        wrapped=PeftModel.from_pretrained(base,args.adapter,is_trainable=False).eval()
        model=wrapped.base_model.model
        model.eval()
    else:model=base
    results=[]
    for i,row in enumerate(rows,1):
        image=Image.open(ROOT/'recordings'/row['image']).convert('RGB')
        w,h=image.size
        t=time.monotonic()
        try:
            with torch.inference_mode():
                raw=str(model.chat(image=image,
                    msgs=[{'role':'user','content':row['expression']}],
                    tokenizer=processor.tokenizer,processor=processor,
                    sampling=False,num_beams=1,max_new_tokens=64,
                    max_slice_nums=4,use_image_id=True))
            box=box_from_raw(raw,w,h)
            score=iou(box,row['bbox']) if box else None
            status='valid' if box else 'invalid_format_or_range'
        except Exception as exc:
            raw='';box=None;score=None;status='inference_error:'+str(exc)
        rec={'image':row['image'],'group_id':row['group_id'],'category':row['category'],'gt_bbox':row['bbox'],
             'raw':raw,'pred_bbox':box,'parse_valid':box is not None,
             'iou':score,'iou_ge_0_5':score is not None and score>=0.5,
             'status':status,'latency_sec':round(time.monotonic()-t,3)}
        results.append(rec)
        print(json.dumps({'step':i,**rec},ensure_ascii=False),flush=True)
    n=len(results)
    valid=[r for r in results if r['parse_valid']]
    metrics={'split':args.split,'rows':n,'adapter':args.adapter or None,
             'review_status':'assistant_visual_review_not_human_ground_truth',
             'strict_parse_rate':len(valid)/n if n else None,
             'mean_iou_valid':sum(r['iou'] for r in valid)/len(valid) if valid else None,
             'iou_ge_0_5_rate_all':sum(r['iou_ge_0_5'] for r in results)/n if n else None,
             'mean_latency_sec':sum(r['latency_sec'] for r in results)/n if n else None,
             'by_category':{},'results':results}
    for category in sorted({r['category'] for r in results}):
        sub=[r for r in results if r['category']==category]
        metrics['by_category'][category]={'n':len(sub),
            'parse_rate':sum(r['parse_valid'] for r in sub)/len(sub),
            'iou_ge_0_5_rate':sum(r['iou_ge_0_5'] for r in sub)/len(sub)}
    Path(args.output).write_text(json.dumps(metrics,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'event':'done','output':args.output,'strict_parse_rate':metrics['strict_parse_rate'],
        'iou_ge_0_5_rate_all':metrics['iou_ge_0_5_rate_all']},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
