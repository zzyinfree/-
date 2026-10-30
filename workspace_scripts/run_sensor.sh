#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
set -u
SDK=/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK
export PYTHONPATH="$SDK:$SDK/comm:$SDK/ctrl:$SDK/ue:$SDK/vision:$SDK/phm:$SDK/swarm:$SDK/adb:$SDK/wsl:$SDK/test:$SDK/word:${PYTHONPATH:-}"
cd /home/ubuntu/catkin_ws/src/sensor_pkg
exec /opt/conda/envs/demo/bin/python3 -u main.py
