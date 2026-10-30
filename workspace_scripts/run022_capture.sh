#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
cd /root/gpufree-data/cicc_workspace
exec python3 record_front_frames.py \
  --output-dir recordings/run_022_blue_p_only_tight_gate \
  --count 800 --interval 0.2 --timeout 145 --allow-partial \
  --group-id run_022_blue_p_only_tight_gate \
  --scene-note blue_yaw_p_only_center_tolerance_10pct
