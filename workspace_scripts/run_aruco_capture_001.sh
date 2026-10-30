#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
exec /opt/conda/envs/demo/bin/python3 -u \
  /root/gpufree-data/cicc_workspace/record_aruco_observation.py \
  --output-dir /root/gpufree-data/cicc_workspace/recordings/aruco_run_001 \
  --timeout 125
