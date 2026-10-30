#!/usr/bin/env python3
"""Evaluate raw-image FM9G4B-V boxes without detector proposals."""
import argparse
import json
from pathlib import Path

import torch
from PIL import Image
from transformers import AutoModel, AutoProcessor, AutoTokenizer

MODEL = '/home/ubuntu/model/FM9G4B-V'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--image', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    image = Image.open(args.image).convert('RGB')
    processor = AutoProcessor.from_pretrained(MODEL, trust_remote_code=True, local_files_only=True)
    tokenizer = AutoTokenizer.from_pretrained(MODEL, trust_remote_code=True, local_files_only=True)
    model = AutoModel.from_pretrained(MODEL, trust_remote_code=True, local_files_only=True,
                                     attn_implementation='sdpa', torch_dtype=torch.bfloat16).eval().to('cuda')
    prompts = {
        'red_balloon': '请在原始完整图像中找到红色气球。只输出目标的像素边界框，格式为 JSON：{"bbox":[x1,y1,x2,y2]}。图像宽640像素、高480像素；若不存在输出 {"bbox":null}。不要解释。',
        'square_frame': '请在原始完整图像中找到可穿越的方形框架。只输出目标的像素边界框，格式为 JSON：{"bbox":[x1,y1,x2,y2]}。图像宽640像素、高480像素；若不存在输出 {"bbox":null}。不要解释。',
    }
    outputs = {}
    with torch.inference_mode():
        for name, prompt in prompts.items():
            raw = str(model.chat(image=None, msgs=[{'role':'user','content':[image,prompt]}],
                                 tokenizer=tokenizer, processor=processor, sampling=False,
                                 stream=False, max_new_tokens=128, max_slice_nums=4))
            try:
                parsed = json.loads(raw)
            except json.JSONDecodeError:
                parsed = None
            outputs[name] = {'raw':raw, 'parsed':parsed}
            print(name, raw, flush=True)
    Path(args.output).write_text(json.dumps({'image':args.image,'outputs':outputs},
                                            ensure_ascii=False,indent=2),encoding='utf-8')


if __name__ == '__main__':
    main()
