#!/usr/bin/env bash
set -eo pipefail
ROOT=/root/gpufree-data/cicc_workspace
source /home/ubuntu/catkin_ws/devel/setup.bash
cd "$ROOT"
exec python3 record_front_frames.py \
  --output-dir recordings/run_017_blue_alignment_gate \
  --count 500 --interval 0.35 --timeout 155 --allow-partial \
  --group-id run_017_blue_alignment_gate \
  --scene-note one_prompt_blue_balloon_alignment_gate

