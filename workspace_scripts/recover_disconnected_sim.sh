#!/usr/bin/env bash
# Restart only this single-UAV development stack; never reset an armed vehicle.
set -eo pipefail
ROOT=/root/gpufree-data/cicc_workspace
source /home/ubuntu/catkin_ws/devel/setup.bash
set -u

if ! test -S /tmp/.X11-unix/X20; then
  echo 'REFUSED: graphical desktop DISPLAY=:20 is unavailable.' >&2
  exit 2
fi
state=$(timeout 10 rostopic echo -n1 /mavros/state || true)
if grep -q 'armed: True' <<<"$state" || ! grep -q 'armed: False' <<<"$state"; then
  echo 'REFUSED: aircraft must be confirmed disarmed.' >&2
  exit 3
fi

tag=$(date +%Y%m%d_%H%M%S)
mkdir -p "$ROOT/outputs"
printf '%s\n' "$state" > "$ROOT/outputs/full_restart_prestate_$tag.txt"

for name in cicc_mission_fast cicc_safety cicc_planner_fast cicc_cloud \
            cicc_aruco cicc_det cicc_map cicc_sensor cicc_sitl cicc_ue cicc_roscore; do
  tmux kill-session -t "$name" 2>/dev/null || true
done
pkill -x px4 || true
pkill -x CopterSim || true
pkill -x RflySim3D || true
sleep 4

# UE needs to finish loading ChallengeMap before CopterSim/PX4 connect.
# Starting the entire stack immediately caused SITL to disconnect on the
# first full-restart trial, although the camera briefly emitted frames.
tmux new-session -d -s cicc_roscore \
  "source /opt/ros/noetic/setup.bash; roscore > $ROOT/outputs/roscore_manual.log 2>&1"
sleep 3
tmux new-session -d -s cicc_ue \
  "bash $ROOT/run_ue_only.sh > $ROOT/outputs/ue_manual.log 2>&1"
sleep 25
if ! pgrep -x RflySim3D >/dev/null; then
  echo 'INCOMPLETE: UE process exited before SITL startup.' >&2
  exit 4
fi

bash "$ROOT/start_sim_manual.sh" | tee "$ROOT/outputs/full_restart_$tag.log"

echo '--- post-restart verification ---'
for _ in $(seq 1 12); do
  state=$(timeout 5 rostopic echo -n1 /mavros/state || true)
  camera=$(timeout 5 rostopic echo -n1 /rflysim/sensor1/img_rgb/header || true)
  if grep -q 'connected: True' <<<"$state" &&
     grep -q 'armed: False' <<<"$state" &&
     grep -q 'frame_id:' <<<"$camera"; then
    printf '%s\n' "$state" | grep -E 'connected:|armed:|mode:'
    printf '%s\n' "$camera" | head -8
    sleep 20
    stable_state=$(timeout 5 rostopic echo -n1 /mavros/state || true)
    if pgrep -x RflySim3D >/dev/null && pgrep -x CopterSim >/dev/null &&
       pgrep -x px4 >/dev/null &&
       grep -q 'connected: True' <<<"$stable_state" &&
       grep -q 'armed: False' <<<"$stable_state"; then
      echo 'READY: UE, CopterSim, PX4, camera and MAVROS stayed available for 20 seconds.'
      exit 0
    fi
    echo 'INCOMPLETE: SITL disconnected during stability check; do not fly.' >&2
    exit 4
  fi
  sleep 3
done
echo 'INCOMPLETE: do not fly; inspect full_restart log and component logs.' >&2
exit 4
