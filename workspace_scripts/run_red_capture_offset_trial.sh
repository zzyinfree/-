#!/usr/bin/env bash
set -eo pipefail
ROOT=/root/gpufree-data/cicc_workspace
source /home/ubuntu/catkin_ws/devel/setup.bash
cd "$ROOT"
exec python3 record_front_frames.py \
  --output-dir recordings/run_015_red_visible_offset \
  --count 500 --interval 0.35 --timeout 155 --allow-partial \
  --group-id run_015_red_visible_offset \
  --scene-note one_prompt_red_balloon_approach_x4p5_y0p4

