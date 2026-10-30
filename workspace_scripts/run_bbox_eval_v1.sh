#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
PY=/root/gpufree-data/cicc_workspace/fm9g_train_venv/bin/python
SCRIPT=/root/gpufree-data/cicc_workspace/evaluate_jiuge_bbox_v1.py
ROOT=/root/gpufree-data/cicc_workspace/outputs
"$PY" -u "$SCRIPT" --split val --output "$ROOT/bbox_eval_base_val_v1.json" \
  > "$ROOT/bbox_eval_base_val_v1.log" 2>&1
"$PY" -u "$SCRIPT" --split val \
  --adapter /root/gpufree-data/cicc_workspace/adapters/lora_provisional_bbox_v1 \
  --output "$ROOT/bbox_eval_lora_val_v1.json" \
  > "$ROOT/bbox_eval_lora_val_v1.log" 2>&1
