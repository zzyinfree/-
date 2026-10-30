#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
cd /root/gpufree-data/cicc_workspace
exec /opt/conda/envs/demo/bin/python3 -u record_front_frames.py \
  --output-dir recordings/run_002_takeoff_land \
  --group-id run_002_takeoff_land_20261015 \
  --scene-note 'original competition scene, map process unchanged, controlled takeoff-land; no scene seed claimed' \
  --count 80 --interval 0.25 --timeout 45
