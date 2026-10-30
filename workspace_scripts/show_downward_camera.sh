#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
export DISPLAY=:20
export XAUTHORITY=/home/ubuntu/.Xauthority
exec /usr/bin/python3 /tmp/downward_camera_qt.py
