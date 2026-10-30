#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
exec /root/gpufree-data/cicc_workspace/fm9g_train_venv/bin/python -u \
  /root/gpufree-data/cicc_workspace/train_jiuge_provisional_v1.py \
  --accept-assistant-reviewed
