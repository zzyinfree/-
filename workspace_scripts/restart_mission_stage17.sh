#!/usr/bin/env bash
source /home/ubuntu/catkin_ws/devel/setup.bash
set -euo pipefail
timeout 6 rostopic echo -n1 /mavros/state | grep -q 'armed: False'
tmux kill-session -t cicc_mission_fast
tmux new-session -d -s cicc_mission_fast 'bash -lc "source /home/ubuntu/catkin_ws/devel/setup.bash; roslaunch mission_pkg llm_mission.launch > /root/gpufree-data/cicc_workspace/outputs/stage17_mission.log 2>&1"'
sleep 4
rostopic info /mission_command | tail -8
tail -5 /root/gpufree-data/cicc_workspace/outputs/stage17_mission.log
