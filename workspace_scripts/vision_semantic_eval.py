#!/usr/bin/env python3
"""Offline semantic check of one real simulator camera frame."""

import argparse
import json
from pathlib import Path

import torch
from PIL import Image
from transformers import AutoModel, AutoProcessor, AutoTokenizer

MODEL = '/home/ubuntu/model/FM9G4B-V'
QUESTIONS = {
    'square_frame': '图中是否可见可穿越的方形框架？仅输出JSON：{"visible":true或false}。',
    'red_balloon': '图中是否可见红色气球？仅输出JSON：{"visible":true或false}。',
    'blue_balloon': '图中是否可见蓝色气球？仅输出JSON：{"visible":true或false}。',
    'purple_balloon': '图中是否可见紫色气球？仅输出JSON：{"visible":true或false}。',
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    image = Image.open(args.image).convert('RGB')
    processor = AutoProcessor.from_pretrained(MODEL, trust_remote_code=True, local_files_only=True)
    tokenizer = AutoTokenizer.from_pretrained(MODEL, trust_remote_code=True, local_files_only=True)
    model = AutoModel.from_pretrained(
        MODEL, trust_remote_code=True, local_files_only=True,
        attn_implementation='sdpa', torch_dtype=torch.bfloat16,
    ).eval().to('cuda')
    results = {}
    with torch.inference_mode():
        for key, prompt in QUESTIONS.items():
            response = str(model.chat(
                image=None, msgs=[{'role': 'user', 'content': [image, prompt]}],
                tokenizer=tokenizer, processor=processor,
                sampling=False, stream=False, max_new_tokens=64,
                max_slice_nums=4,
            ))
            try:
                parsed = json.loads(response)
                visible = parsed['visible'] if type(parsed.get('visible')) is bool else None
            except (json.JSONDecodeError, KeyError, TypeError):
                visible = None
            results[key] = {'prompt': prompt, 'raw': response, 'visible': visible}
            print(json.dumps({key: results[key]}, ensure_ascii=False), flush=True)
    Path(args.output).write_text(json.dumps({
        'image': args.image, 'results': results,
    }, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
