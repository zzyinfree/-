#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
cd /root/gpufree-data/cicc_workspace
exec /opt/conda/envs/demo/bin/python3 -u record_front_frames.py \
  --output-dir recordings/run_003_observation_route \
  --group-id run_003_observation_route_20261015 \
  --scene-note 'original competition scene; takeoff-cross_frame-land BT route, no strike; no scene seed claimed' \
  --count 180 --interval 0.4 --timeout 95
