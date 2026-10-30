#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
exec /root/gpufree-data/cicc_workspace/fm9g_train_venv/bin/python -u \
  /root/gpufree-data/cicc_workspace/jiuge_vision_bridge.py \
  --image-topic /rflysim/sensor1/img_rgb "$@"
