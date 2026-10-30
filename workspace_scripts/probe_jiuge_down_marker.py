#!/usr/bin/env python3
"""Independent FM9G4B-V whole-image ground-marker bbox probe; no flight control."""
import argparse
import json
import re
import time
from pathlib import Path

import torch
from PIL import Image
from transformers import AutoModel, AutoProcessor

MODEL = '/home/ubuntu/model/FM9G4B-V'
ADAPTER = '/root/gpufree-data/cicc_workspace/adapters/lora_stage12_reviewed_seed'
PROMPT = '请框出图中的地面二维码标识。只输出一个 <box>。'
PAT = re.compile(r'<box>\s*\[\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*\]\s*</box>')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--images', nargs='+', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--adapter', action='store_true')
    ap.add_argument('--adapter-path', default=ADAPTER)
    args = ap.parse_args()
    processor = AutoProcessor.from_pretrained(MODEL, trust_remote_code=True,
                                              local_files_only=True)
    base = AutoModel.from_pretrained(MODEL, trust_remote_code=True,
                                    local_files_only=True,
                                    attn_implementation='sdpa',
                                    torch_dtype=torch.bfloat16).eval().to('cuda')
    if args.adapter:
        from peft import PeftModel
        model = PeftModel.from_pretrained(base, args.adapter_path, is_trainable=False).eval().base_model.model
    else:
        model = base
    results = []
    for path in args.images:
        image = Image.open(path).convert('RGB')
        start = time.monotonic()
        try:
            with torch.inference_mode():
                raw = str(model.chat(image=image,
                    msgs=[{'role': 'user', 'content': PROMPT}],
                    tokenizer=processor.tokenizer, processor=processor,
                    sampling=False, num_beams=1, max_new_tokens=64,
                    max_slice_nums=4, use_image_id=True))
            match = PAT.fullmatch(raw.strip())
            vals = list(map(float, match.groups())) if match else None
            valid = bool(vals and 0 <= vals[0] < vals[2] <= 1000 and
                         0 <= vals[1] < vals[3] <= 1000)
            bbox = [vals[0]*image.width/1000, vals[1]*image.height/1000,
                    vals[2]*image.width/1000, vals[3]*image.height/1000] if valid else None
            result = {'image': path, 'model': MODEL,
                      'adapter': args.adapter_path if args.adapter else None,
                      'prompt': PROMPT, 'raw': raw, 'valid': valid,
                      'bbox_1000': vals if valid else None,
                      'bbox_px': bbox, 'latency_sec': round(time.monotonic()-start, 3)}
        except Exception as exc:
            result = {'image': path, 'error': str(exc)}
        results.append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    Path(args.output).write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
