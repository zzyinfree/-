#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
export DISPLAY=:20
export XAUTHORITY=/home/ubuntu/.Xauthority
cd /tmp
exec rosrun rqt_image_view rqt_image_view
