#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
export DISPLAY=:20
exec roslaunch object_det detection_color.launch
