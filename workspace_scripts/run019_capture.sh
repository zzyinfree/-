#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
cd /root/gpufree-data/cicc_workspace
exec python3 record_front_frames.py \
  --output-dir recordings/run_019_blue_slow_telemetry \
  --count 900 --interval 0.2 --timeout 175 --allow-partial \
  --group-id run_019_blue_slow_telemetry \
  --scene-note blue_slow_approach_descent_floor
