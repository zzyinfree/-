#!/usr/bin/env python3
"""One-step FM9G4B-V image-to-box LoRA pipeline test; NOT a trained detector."""
import json
from pathlib import Path

import torch
from PIL import Image
from peft import LoraConfig, inject_adapter_in_model, get_peft_model_state_dict
from transformers import AutoModel, AutoProcessor

ROOT = Path('/root/gpufree-data/cicc_workspace')
MODEL = '/home/ubuntu/model/FM9G4B-V'
image = Image.open(ROOT / 'recordings/run_001/frame_0036.png').convert('RGB')
question = ('请在原始完整图像中找到红色气球。只输出目标的像素边界框，格式为 JSON：'
            '{"bbox":[x1,y1,x2,y2]}。图像宽640像素、高480像素；若不存在输出 '
            '{"bbox":null}。不要解释。')
# This box was visually checked against the red balloon on the original full frame.
answer = '{"bbox":[78,165,134,228]}'
processor = AutoProcessor.from_pretrained(MODEL, trust_remote_code=True, local_files_only=True)
tokenizer = processor.tokenizer
model = AutoModel.from_pretrained(MODEL, trust_remote_code=True, local_files_only=True,
                                 attn_implementation='sdpa', torch_dtype=torch.bfloat16).to('cuda')
model.requires_grad_(False)
cfg = LoraConfig(r=8, lora_alpha=16, lora_dropout=0.0,
                 target_modules=['q_a_proj', 'kv_a_proj_with_mqa', 'o_proj'],
                 bias='none')
model = inject_adapter_in_model(cfg, model)
model.train()
# Frozen vision tower in evaluation mode: avoid needless dropout/state updates.
model.vpm.eval()
prompt_msg = [{'role':'user','content':['(<image>./</image>)',question]}]
# The model's chat method joins content items with a newline; reproduce that format.
prompt_msg[0]['content'] = '(<image>./</image>)\n' + question
prefix = tokenizer.apply_chat_template(prompt_msg, tokenize=False, add_generation_prompt=True)
full = tokenizer.apply_chat_template(prompt_msg + [{'role':'assistant','content':answer}],
                                     tokenize=False, add_generation_prompt=False)
prefix_data = processor([prefix], [[image]], max_slice_nums=1, return_tensors='pt', max_length=2048)
data = processor([full], [[image]], max_slice_nums=1, return_tensors='pt', max_length=2048)
prefix_ids = prefix_data['input_ids'][0]
full_ids = data['input_ids'][0]
common = 0
for a,b in zip(prefix_ids.tolist(),full_ids.tolist()):
    if a != b: break
    common += 1
if common < len(prefix_ids)-2:
    raise RuntimeError(f'Prompt token mismatch: {common}/{len(prefix_ids)}')
data.pop('image_sizes')
data = data.to('cuda')
labels = data['input_ids'].clone().long()
labels[:, :common] = -100
labels[data['attention_mask'] == 0] = -100
data['position_ids'] = torch.arange(data['input_ids'].shape[1],device='cuda').unsqueeze(0)
opt = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad], lr=1e-4)
opt.zero_grad(set_to_none=True)
out = model(data, labels=labels, use_cache=False)
loss = out.loss.sum() / (labels[:, 1:] != -100).sum().clamp_min(1)
if not torch.isfinite(loss):
    raise RuntimeError('nonfinite loss')
loss.backward()
grad_norm = torch.nn.utils.clip_grad_norm_([p for p in model.parameters() if p.requires_grad],1.0)
opt.step()
save_dir = ROOT / 'adapters/lora_smoke_001'
save_dir.mkdir(parents=True, exist_ok=True)
torch.save(get_peft_model_state_dict(model), save_dir / 'adapter_model.pt')
(save_dir / 'adapter_config.json').write_text(json.dumps(cfg.to_dict(),indent=2,
                                                        default=lambda x:list(x) if isinstance(x,set) else str(x)),
                                              encoding='utf-8')
result = {'status':'one_step_smoke_only','loss':float(loss.item()),'grad_norm':float(grad_norm.item()),
          'trainable_parameters':sum(p.numel() for p in model.parameters() if p.requires_grad),
          'sequence_length':int(data['input_ids'].shape[1]),'supervised_tokens':int((labels!=-100).sum()),
          'common_prefix_tokens':common,'peak_cuda_memory_gib':round(torch.cuda.max_memory_allocated()/2**30,2),
          'adapter_path':str(save_dir),'label_source':'one manually visually checked YOLO candidate; not eval ground truth'}
(save_dir/'smoke_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False),flush=True)
