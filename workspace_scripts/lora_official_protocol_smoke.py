#!/usr/bin/env python3
"""PDF-compliant one-step image→<box> LoRA smoke, not an accuracy claim."""
import json
from pathlib import Path

import torch
from PIL import Image
from peft import LoraConfig, get_peft_model
from transformers import AutoModel, AutoProcessor

ROOT=Path('/root/gpufree-data/cicc_workspace')
MODEL='/home/ubuntu/model/FM9G4B-V'
image=Image.open(ROOT/'recordings/run_001/frame_0036.png').convert('RGB')
W,H=image.size
pixel_box=[78,165,134,228]  # The sole visually checked sample for plumbing test.
norm=[round(pixel_box[i]/(W if i%2==0 else H)*1000) for i in range(4)]
answer='<box>['+','.join(map(str,norm))+']</box>'
query='请框出图中的红色气球。只输出一个 <box>。'
processor=AutoProcessor.from_pretrained(MODEL,trust_remote_code=True,local_files_only=True)
tok=processor.tokenizer
user={'role':'user','content':'(<image>./</image>)\n'+query}
prefix=tok.apply_chat_template([user],tokenize=False,add_generation_prompt=True)
full=tok.apply_chat_template([user,{'role':'assistant','content':answer}],
                             tokenize=False,add_generation_prompt=False)
prefix_inputs=processor([prefix],[[image]],max_slice_nums=4,use_image_id=True,
                        return_tensors='pt',max_length=1024)
inputs=processor([full],[[image]],max_slice_nums=4,use_image_id=True,
                 return_tensors='pt',max_length=1024)
prefix_ids=prefix_inputs['input_ids'][0].tolist(); full_ids=inputs['input_ids'][0].tolist()
common=0
for a,b in zip(prefix_ids,full_ids):
    if a!=b:break
    common+=1
if common<len(prefix_ids)-2:raise RuntimeError(f'prompt prefix mismatch {common}/{len(prefix_ids)}')
labels=inputs['input_ids'].clone().long()
labels[:,:common]=-100
labels[~inputs['attention_mask']]=-100
inputs['position_ids']=inputs['attention_mask'].long().cumsum(-1)-1
inputs['position_ids'].masked_fill_(~inputs['attention_mask'],0)
inputs.pop('image_sizes',None)
inputs=inputs.to('cuda');labels=labels.to('cuda')
model=AutoModel.from_pretrained(MODEL,trust_remote_code=True,local_files_only=True,
                                attn_implementation='sdpa',torch_dtype=torch.bfloat16).to('cuda')
model.requires_grad_(False)
cfg=LoraConfig(r=16,lora_alpha=32,lora_dropout=0.05,bias='none',target_modules=[
    'q_a_proj','q_b_proj','kv_a_proj_with_mqa','kv_b_proj','o_proj',
    'gate_proj','up_proj','down_proj'])
model=get_peft_model(model,cfg)
model.train();model.base_model.model.vpm.eval()
opt=torch.optim.AdamW((p for p in model.parameters() if p.requires_grad),lr=1e-4)
opt.zero_grad(set_to_none=True)
out=model(data=inputs,attention_mask=inputs['attention_mask'],labels=labels,
          use_cache=False,return_dict=True)
shift=labels[...,1:].reshape(-1)!=-100
loss=out.loss[shift].mean()
if not torch.isfinite(loss).item():raise RuntimeError('nonfinite loss')
loss.backward()
gn=torch.nn.utils.clip_grad_norm_((p for p in model.parameters() if p.requires_grad),1.0)
opt.step()
dest=ROOT/'adapters/lora_official_protocol_smoke_001'
if dest.exists():raise FileExistsError(dest)
dest.mkdir(parents=True)
model.save_pretrained(dest,safe_serialization=True)
processor.save_pretrained(dest)
result={'status':'one_step_protocol_smoke_not_trained_detector','loss':float(loss),
        'grad_norm':float(gn),'trainable_parameters':sum(p.numel() for p in model.parameters() if p.requires_grad),
        'answer':answer,'image':'recordings/run_001/frame_0036.png',
        'max_slice_nums':4,'use_image_id':True,'format':'normalized_xyxy_0_1000',
        'supervised_tokens':int(shift.sum()),
        'peak_cuda_memory_gib':round(torch.cuda.max_memory_allocated()/2**30,2),
        'base_architecture_unchanged':True,'image_label_status':'single visually checked box; not eval set'}
(dest/'training_config.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False),flush=True)
