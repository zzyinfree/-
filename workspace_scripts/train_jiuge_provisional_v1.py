#!/usr/bin/env python3
"""One-epoch provisional FM9G4B-V LoRA; not official final detector training.

The input contains assistant-reviewed boxes, clearly distinct from verified GT.
Never replaces the base model or changes its architecture.
"""
import argparse
import json
import random
import time
from pathlib import Path

import torch
from PIL import Image
from peft import LoraConfig,get_peft_model
from transformers import AutoModel,AutoProcessor

ROOT=Path('/root/gpufree-data/cicc_workspace')
MODEL='/home/ubuntu/model/FM9G4B-V'
DATA=ROOT/'annotations/provisional_bbox_v1.jsonl'
OUT=ROOT/'adapters/lora_provisional_bbox_v1'
TARGET_MODULES=['q_a_proj','q_b_proj','kv_a_proj_with_mqa','kv_b_proj',
                'o_proj','gate_proj','up_proj','down_proj']


def norm_answer(row,w,h):
    box=row['bbox']
    vals=[round(box[i]/(w if i%2==0 else h)*1000) for i in range(4)]
    return '<box>['+','.join(map(str,vals))+']</box>'


def encode(row,processor):
    image=Image.open(ROOT/'recordings'/row['image']).convert('RGB')
    answer=norm_answer(row,*image.size)
    tok=processor.tokenizer
    user={'role':'user','content':'(<image>./</image>)\n'+row['expression']}
    prefix=tok.apply_chat_template([user],tokenize=False,add_generation_prompt=True)
    full=tok.apply_chat_template([user,{'role':'assistant','content':answer}],
                                 tokenize=False,add_generation_prompt=False)
    p=processor([prefix],[[image]],max_slice_nums=4,use_image_id=True,
                return_tensors='pt',max_length=1024)
    x=processor([full],[[image]],max_slice_nums=4,use_image_id=True,
                return_tensors='pt',max_length=1024)
    pi=p['input_ids'][0].tolist();fi=x['input_ids'][0].tolist()
    common=0
    for a,b in zip(pi,fi):
        if a!=b:break
        common+=1
    if common<len(pi)-2:raise RuntimeError(f'prefix mismatch {row["image"]} {common}/{len(pi)}')
    labels=x['input_ids'].clone().long()
    labels[:,:common]=-100
    labels[~x['attention_mask']]=-100
    supervised=(labels[...,1:]!=-100).sum().item()
    if supervised<5:raise RuntimeError(f'no assistant labels {row["image"]}')
    x['position_ids']=x['attention_mask'].long().cumsum(-1)-1
    x['position_ids'].masked_fill_(~x['attention_mask'],0)
    x.pop('image_sizes',None)
    return x.to('cuda'),labels.to('cuda'),answer,int(supervised)


def sample_loss(model,x,labels):
    out=model(data=x,attention_mask=x['attention_mask'],labels=labels,
              use_cache=False,return_dict=True)
    active=(labels[...,1:]!=-100).reshape(-1)
    return out.loss[active].mean()


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--accept-assistant-reviewed',action='store_true',required=True,
                    help='Explicit consent to use provisional, non-human-verified labels')
    ap.add_argument('--seed',type=int,default=26015)
    ap.add_argument('--lr',type=float,default=5e-5)
    ap.add_argument('--accum',type=int,default=4)
    ap.add_argument('--epochs',type=int,default=1)
    ap.add_argument('--train-resampler',action='store_true',
                    help='Controlled visual-domain ablation; save resampler with LoRA')
    args=ap.parse_args()
    if not 1 <= args.epochs <= 5:raise ValueError('epochs must be 1..5')
    if OUT.exists():raise FileExistsError(OUT)
    rows=[json.loads(s) for s in DATA.read_text(encoding='utf-8').splitlines() if s.strip()]
    train=[r for r in rows if r['split']=='train']
    val=[r for r in rows if r['split']=='val']
    assert len(train)>=20 and len(val)>=5
    assert {r['group_id'] for r in train}.isdisjoint({r['group_id'] for r in val})
    assert all(r['review_status'] in ('human_verified','assistant_visual_review') for r in rows)
    random.seed(args.seed);torch.manual_seed(args.seed)
    random.shuffle(train)
    processor=AutoProcessor.from_pretrained(MODEL,trust_remote_code=True,local_files_only=True)
    model=AutoModel.from_pretrained(MODEL,trust_remote_code=True,local_files_only=True,
        attn_implementation='sdpa',torch_dtype=torch.bfloat16).to('cuda')
    model.requires_grad_(False)
    cfg=LoraConfig(r=16,lora_alpha=32,lora_dropout=0.05,bias='none',target_modules=TARGET_MODULES,
                   modules_to_save=['resampler'] if args.train_resampler else None)
    model=get_peft_model(model,cfg)
    model.train()
    model.base_model.model.vpm.eval()
    opt=torch.optim.AdamW((p for p in model.parameters() if p.requires_grad),lr=args.lr)
    opt.zero_grad(set_to_none=True)
    started=time.time()
    log=[]
    print(json.dumps({'event':'start','train_rows':len(train),'val_rows':len(val),
        'trainable':sum(p.numel() for p in model.parameters() if p.requires_grad),
        'seed':args.seed,'lr':args.lr,'accum':args.accum,'epochs':args.epochs,
        'train_resampler':args.train_resampler},ensure_ascii=False),flush=True)
    for i,row in enumerate(train * args.epochs,1):
        x,labels,answer,n=encode(row,processor)
        loss=sample_loss(model,x,labels)
        if not torch.isfinite(loss):raise RuntimeError(f'nonfinite loss at step {i}')
        (loss/args.accum).backward()
        if i%args.accum==0 or i==len(train)*args.epochs:
            torch.nn.utils.clip_grad_norm_((p for p in model.parameters() if p.requires_grad),1.0)
            opt.step();opt.zero_grad(set_to_none=True)
        rowlog={'event':'train_step','step':i,'epoch':(i-1)//len(train)+1,'category':row['category'],
                'image':row['image'],'loss':round(float(loss.detach()),5),
                'supervised_tokens':n,'answer':answer}
        log.append(rowlog)
        print(json.dumps(rowlog,ensure_ascii=False),flush=True)
        del x,labels,loss
    model.eval()
    losses=[]
    with torch.inference_mode():
        for i,row in enumerate(val,1):
            x,labels,answer,n=encode(row,processor)
            value=float(sample_loss(model,x,labels))
            losses.append({'image':row['image'],'category':row['category'],'loss':value})
            del x,labels
    OUT.mkdir(parents=True)
    model.save_pretrained(OUT,safe_serialization=True)
    processor.save_pretrained(OUT)
    summary={'status':'provisional_not_competition_ready',
             'base_model':MODEL,'dataset':str(DATA),'train_rows':len(train),'val_rows':len(val),
             'train_groups':sorted({r['group_id'] for r in train}),
             'val_groups':sorted({r['group_id'] for r in val}),
             'review_status_counts':{k:sum(r['review_status']==k for r in rows)
                                    for k in ('human_verified','assistant_visual_review')},
             'epochs':args.epochs,'lr':args.lr,'accum':args.accum,'seed':args.seed,
             'train_resampler':args.train_resampler,
             'target_modules':TARGET_MODULES,'max_slice_nums':4,'use_image_id':True,
             'mean_val_teacher_forced_loss':sum(v['loss'] for v in losses)/len(losses),
             'validation_losses':losses,'elapsed_sec':round(time.time()-started,1),
             'peak_cuda_memory_gib':round(torch.cuda.max_memory_allocated()/2**30,2),
             'training_log':log}
    (OUT/'provisional_training_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'event':'done','out':str(OUT),'mean_val_teacher_forced_loss':
        summary['mean_val_teacher_forced_loss'],'elapsed_sec':summary['elapsed_sec']},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
