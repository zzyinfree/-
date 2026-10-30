#!/usr/bin/env bash
# Reset this single-UAV development scene only while connected and disarmed.
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
set -u
ROOT=/root/gpufree-data/cicc_workspace
state=$(timeout 10 rostopic echo -n 1 /mavros/state || true)
if ! grep -q 'connected: True' <<<"$state" || ! grep -q 'armed: False' <<<"$state"; then
  echo 'REFUSED: MAVROS must be connected and the aircraft disarmed before reset.' >&2
  exit 2
fi
tag=$(date +%Y%m%d_%H%M%S)
tmux kill-session -t cicc_mission_fast 2>/dev/null || true
tmux kill-session -t cicc_map 2>/dev/null || true
tmux kill-session -t cicc_planner_fast 2>/dev/null || true
tmux kill-session -t cicc_sitl 2>/dev/null || true
pkill -x px4 || true
pkill -x CopterSim || true
sleep 3
tmux new-session -d -s cicc_sitl "bash $ROOT/start_sitl_clone.sh"
sleep 15
tmux new-session -d -s cicc_map "bash $ROOT/run_map.sh > $ROOT/outputs/map_manual_reset_$tag.log 2>&1"
sleep 8
tmux new-session -d -s cicc_planner_fast "bash $ROOT/run_planner_fast.sh > $ROOT/outputs/planner_manual_reset_$tag.log 2>&1"
sleep 6
tmux new-session -d -s cicc_mission_fast "bash $ROOT/run_mission_patched.sh > $ROOT/outputs/mission_manual_reset_$tag.log 2>&1"
state=$(timeout 10 rostopic echo -n 1 /mavros/state || true)
printf '%s\n' "$state"
if ! grep -q 'connected: True' <<<"$state" || ! grep -q 'armed: False' <<<"$state"; then
  echo 'RESET INCOMPLETE: do not send a flight prompt; inspect the logs.' >&2
  exit 3
fi
timeout 10 rostopic echo -n 1 /rflysim/uav1/local/odom
echo 'Reset commands completed. Confirm the aircraft is near the origin and the camera shows the competition scene.'
