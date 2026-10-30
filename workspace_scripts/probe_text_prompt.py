import json
import os
import time

import torch
from transformers import AutoModel, AutoTokenizer

MODEL = "/home/ubuntu/model/FM9G4B-V"
PROMPT = "/home/ubuntu/model/prompts/mission_prompt.txt"
TASKS = [
    "飞机起飞，然后穿过框，再去识别二维码，然后再穿回框，接着去打击红色气球，完成后就回到起点并降落；",
    "起飞，穿框，识别二维码，再穿回框，打击蓝色气球，回到起点，降落；",
    "飞机起飞，穿框，识别二维码，再穿回框，刺破紫球，回到起点并降落；",
    "起飞后穿框识别二维码，再穿回来并降落，不打气球。",
]

knowledge = open(PROMPT, encoding="utf-8").read()
started = time.time()
model = AutoModel.from_pretrained(
    MODEL, trust_remote_code=True, attn_implementation="sdpa", torch_dtype=torch.bfloat16
).eval().cuda()
tokenizer = AutoTokenizer.from_pretrained(MODEL, trust_remote_code=True)
print("MODEL_LOAD_SECONDS", round(time.time() - started, 2), flush=True)
for task in TASKS:
    msgs = [
        {"role": "user", "content": [knowledge]},
        {"role": "assistant", "content": ["已了解。请给出任务。"]},
        {"role": "user", "content": [task]},
    ]
    t0 = time.time()
    with torch.inference_mode():
        output = model.chat(image=None, msgs=msgs, tokenizer=tokenizer, sampling=False, stream=False)
    print(json.dumps({"task": task, "reply": output, "seconds": round(time.time()-t0, 2)}, ensure_ascii=False), flush=True)
