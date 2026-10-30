#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
PYTHON_BIN=/root/gpufree-data/cicc_workspace/fm9g_train_venv/bin/python
if [[ ! -x "$PYTHON_BIN" ]]; then
  PYTHON_BIN=/opt/conda/envs/demo/bin/python
fi
exec "$PYTHON_BIN" -u /root/gpufree-data/cicc_workspace/mission_online.py "$@"
