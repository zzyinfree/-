#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
exec roslaunch mission_pkg llm_mission.launch
