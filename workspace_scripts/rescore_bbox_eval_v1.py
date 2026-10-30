#!/usr/bin/env python3
"""Re-score saved raw responses after fixing whitespace-tolerant protocol parser."""
import json
from pathlib import Path
from evaluate_jiuge_bbox_v1 import box_from_raw,iou

root=Path('/root/gpufree-data/cicc_workspace/outputs')
for stem in ('bbox_eval_base_val_v1','bbox_eval_lora_val_v1'):
    source=root/(stem+'.json')
    data=json.loads(source.read_text(encoding='utf-8'))
    for row in data['results']:
        box=box_from_raw(row['raw'],640,480)
        score=iou(box,row['gt_bbox']) if box else None
        row.update(pred_bbox=box,parse_valid=box is not None,iou=score,
                   iou_ge_0_5=score is not None and score>=0.5,
                   status='valid' if box else 'invalid_format_or_range')
    results=data['results'];n=len(results)
    valid=[r for r in results if r['parse_valid']]
    data['strict_parse_rate']=len(valid)/n
    data['mean_iou_valid']=sum(r['iou'] for r in valid)/len(valid) if valid else None
    data['iou_ge_0_5_rate_all']=sum(r['iou_ge_0_5'] for r in results)/n
    for category in data['by_category']:
        sub=[r for r in results if r['category']==category]
        data['by_category'][category]['parse_rate']=sum(r['parse_valid'] for r in sub)/len(sub)
        data['by_category'][category]['iou_ge_0_5_rate']=sum(r['iou_ge_0_5'] for r in sub)/len(sub)
    data['scoring_note']='Protocol permits whitespace after <box>; re-scored saved raw outputs without new inference.'
    dest=root/(stem+'_rescored.json')
    dest.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'file':str(dest),'parse_rate':data['strict_parse_rate'],
        'mean_iou_valid':data['mean_iou_valid'],
        'iou_ge_0_5_rate_all':data['iou_ge_0_5_rate_all'],
        'by_category':data['by_category']},ensure_ascii=False))
