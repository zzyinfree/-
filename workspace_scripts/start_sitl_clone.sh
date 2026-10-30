#!/usr/bin/env bash
set -euo pipefail
ROOT=/root/gpufree-data/cicc_workspace
export DISPLAY=:20
export LD_LIBRARY_PATH="/home/ubuntu/PX4PSP/CopterSim/lib:${LD_LIBRARY_PATH:-}"
cd /home/ubuntu/PX4PSP/CopterSim
./CopterSim 2 1 310 0 2 ChallengeMap 0 -0.1 -0.8 0 1 2 > "$ROOT/outputs/cop_sim_clone.log" 2>&1 &
coptersim_pid=$!
sleep 5
cd /home/ubuntu/PX4PSP/Firmware
bash ./Tools/sitl_multiple_run_rfly.sh 1 1 iris > "$ROOT/outputs/px4_clone.log" 2>&1
# PX4 launcher forks and returns; keep the owning tmux session alive.
wait "$coptersim_pid"
