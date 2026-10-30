#!/usr/bin/env python3
"""Continue official-architecture FM9G4B-V LoRA on visually reviewed v2 boxes.

Research-only: assistant-reviewed labels, not competition-ready ground truth.
Never publishes ROS detection or flight topics.
"""
import argparse
import json
import random
import time
from pathlib import Path

import torch
from peft import PeftModel
from transformers import AutoModel, AutoProcessor

from train_jiuge_provisional_v1 import encode, sample_loss

ROOT = Path('/root/gpufree-data/cicc_workspace')
MODEL = '/home/ubuntu/model/FM9G4B-V'
DATA = ROOT / 'annotations/reviewed_bbox_v2.jsonl'
SOURCE = ROOT / 'adapters/lora_provisional_bbox_v1'
OUT = ROOT / 'adapters/lora_reviewed_bbox_v2'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--accept-assistant-reviewed', action='store_true', required=True)
    ap.add_argument('--epochs', type=int, default=2)
    ap.add_argument('--lr', type=float, default=2e-5)
    ap.add_argument('--accum', type=int, default=4)
    ap.add_argument('--seed', type=int, default=26016)
    args = ap.parse_args()
    if OUT.exists():
        raise FileExistsError(OUT)
    assert 1 <= args.epochs <= 3 and args.accum >= 1
    rows = [json.loads(s) for s in DATA.read_text(encoding='utf-8').splitlines() if s.strip()]
    train = [r for r in rows if r['split'] == 'train']
    val = [r for r in rows if r['split'] == 'val']
    assert len(train) >= 40 and len(val) >= 10
    assert {r['group_id'] for r in train}.isdisjoint({r['group_id'] for r in val})
    assert all(r['review_status'] in ('assistant_visual_review', 'human_verified') for r in rows)
    random.seed(args.seed)
    torch.manual_seed(args.seed)
    processor = AutoProcessor.from_pretrained(MODEL, trust_remote_code=True, local_files_only=True)
    base = AutoModel.from_pretrained(MODEL, trust_remote_code=True, local_files_only=True,
        attn_implementation='sdpa', torch_dtype=torch.bfloat16).to('cuda')
    base.requires_grad_(False)
    model = PeftModel.from_pretrained(base, SOURCE, is_trainable=True)
    model.train()
    model.base_model.model.vpm.eval()
    opt = torch.optim.AdamW((p for p in model.parameters() if p.requires_grad), lr=args.lr)
    started = time.time()
    trace = []
    print(json.dumps({'event': 'start', 'model': MODEL, 'source_adapter': str(SOURCE),
        'train_rows': len(train), 'val_rows': len(val), 'epochs': args.epochs,
        'lr': args.lr, 'accum': args.accum,
        'trainable': sum(p.numel() for p in model.parameters() if p.requires_grad)}), flush=True)
    for epoch in range(1, args.epochs + 1):
        random.shuffle(train)
        opt.zero_grad(set_to_none=True)
        losses = []
        for i, row in enumerate(train, 1):
            x, labels, answer, n = encode(row, processor)
            loss = sample_loss(model, x, labels)
            if not torch.isfinite(loss):
                raise RuntimeError(f'nonfinite loss epoch={epoch} step={i}')
            (loss / args.accum).backward()
            if i % args.accum == 0 or i == len(train):
                torch.nn.utils.clip_grad_norm_((p for p in model.parameters() if p.requires_grad), 1.0)
                opt.step()
                opt.zero_grad(set_to_none=True)
            losses.append(float(loss.detach()))
            if i % 5 == 0 or i == len(train):
                print(json.dumps({'event': 'train_progress', 'epoch': epoch, 'step': i,
                    'rows': len(train), 'recent_mean_loss': round(sum(losses[-5:]) / len(losses[-5:]), 4)}), flush=True)
            del x, labels, loss
        model.eval()
        val_losses = []
        with torch.inference_mode():
            for row in val:
                x, labels, _, _ = encode(row, processor)
                val_losses.append(float(sample_loss(model, x, labels)))
                del x, labels
        record = {'epoch': epoch, 'train_teacher_forced_mean': sum(losses)/len(losses),
                  'val_teacher_forced_mean': sum(val_losses)/len(val_losses)}
        trace.append(record)
        print(json.dumps({'event': 'epoch_done', **record}), flush=True)
        model.train()
        model.base_model.model.vpm.eval()
    OUT.mkdir(parents=True)
    model.save_pretrained(OUT, safe_serialization=True)
    processor.save_pretrained(OUT)
    summary = {'status': 'research_only_not_competition_ready', 'source_adapter': str(SOURCE),
        'data': str(DATA), 'train_rows': len(train), 'val_rows': len(val),
        'review_status': 'assistant_visual_review', 'group_disjoint': True,
        'epochs': args.epochs, 'lr': args.lr, 'accum': args.accum,
        'trace': trace, 'elapsed_sec': round(time.time()-started, 1),
        'peak_cuda_gib': round(torch.cuda.max_memory_allocated()/2**30, 2)}
    (OUT/'training_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'event':'done', 'output': str(OUT), 'summary': summary}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
