#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
exec /opt/conda/envs/demo/bin/python3 -u /root/gpufree-data/cicc_workspace/mission_safety_supervisor.py
