#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
cd /root/gpufree-data/cicc_workspace
exec python3 record_front_frames.py \
  --output-dir recordings/run_021_blue_yaw_gain \
  --count 800 --interval 0.2 --timeout 145 --allow-partial \
  --group-id run_021_blue_yaw_gain \
  --scene-note blue_threshold_2p5pct_yaw_p0p006_limit0p9
