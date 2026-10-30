#!/usr/bin/env python3
"""Offline FM9G4B-V feasibility probe on recorded downward-camera frames."""
import argparse
import json
import time
import torch
from PIL import Image
from transformers import AutoModel, AutoProcessor

MODEL = '/home/ubuntu/model/FM9G4B-V'

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('images', nargs='+')
    args = ap.parse_args()
    processor = AutoProcessor.from_pretrained(MODEL, trust_remote_code=True, local_files_only=True)
    model = AutoModel.from_pretrained(MODEL, trust_remote_code=True,
        local_files_only=True, attn_implementation='sdpa',
        torch_dtype=torch.bfloat16).eval().to('cuda')
    for path in args.images:
        image = Image.open(path).convert('RGB')
        for label, prompt in [
            ('car', '请框出图中的移动小车。只输出一个 <box>；如果没有小车，只输出 NONE。'),
            ('blue_balloon', '请框出图中的蓝色气球。只输出一个 <box>；如果没有蓝色气球，只输出 NONE。'),
        ]:
            t = time.monotonic()
            try:
                with torch.inference_mode():
                    raw = str(model.chat(image=image,
                        msgs=[{'role': 'user', 'content': prompt}],
                        tokenizer=processor.tokenizer, processor=processor,
                        sampling=False, num_beams=1, max_new_tokens=64,
                        max_slice_nums=4, use_image_id=True))
                print(json.dumps({'image': path, 'label': label, 'raw': raw,
                    'seconds': round(time.monotonic()-t, 2)}, ensure_ascii=False), flush=True)
            except Exception as e:
                print(json.dumps({'image': path, 'label': label, 'error': str(e)}, ensure_ascii=False), flush=True)

if __name__ == '__main__':
    main()
