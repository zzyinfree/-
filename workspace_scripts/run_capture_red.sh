#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
cd /root/gpufree-data/cicc_workspace
exec python3 record_front_frames.py \
  --output-dir recordings/run_004_prompt_red \
  --count 120 --interval 0.8 --timeout 130 \
  --group-id run_004_prompt_red \
  --scene-note prompt_driven_red_balloon
