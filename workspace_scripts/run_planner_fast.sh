#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
set -u
# Local simulation trial only. Keep the scene, renderer and safety limits intact.
exec roslaunch ego_planner FS-J310_ego-planner.launch
