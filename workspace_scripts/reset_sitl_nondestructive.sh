#!/usr/bin/env bash
set -eo pipefail
# Development-only reset. Preserve UE, roscore, Jupyter and datasets.
source /opt/ros/noetic/setup.bash
set -u
state=$(timeout 8 rostopic echo -n 1 /mavros/state || true)
if ! grep -q 'armed: False' <<<"$state"; then
  echo 'Refusing SITL reset: vehicle is armed or MAVROS state unavailable' >&2
  exit 2
fi
pkill -x px4 || true
pkill -x CopterSim || true
sleep 3
export DISPLAY=:20
export LD_LIBRARY_PATH="/home/ubuntu/PX4PSP/CopterSim/lib:${LD_LIBRARY_PATH:-}"
cd /home/ubuntu/PX4PSP/CopterSim
./CopterSim 2 1 310 0 2 ChallengeMap 0 -0.1 -0.8 0 1 2 \
  > /root/gpufree-data/cicc_workspace/outputs/cop_sim_reset.log 2>&1 &
sleep 5
cd /home/ubuntu/PX4PSP/Firmware
bash ./Tools/sitl_multiple_run_rfly.sh 1 1 iris \
  > /root/gpufree-data/cicc_workspace/outputs/px4_reset.log 2>&1
sleep 8
new_state=$(timeout 8 rostopic echo -n 1 /mavros/state || true)
printf '%s\n' "$new_state"
if ! grep -q 'connected: True' <<<"$new_state"; then
  echo 'MAVROS did not reconnect; check px4_reset.log' >&2
  exit 3
fi
