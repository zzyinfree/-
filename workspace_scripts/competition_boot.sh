#!/usr/bin/env bash
# Start the competition simulator, but never arm or send a mission prompt.
set -eo pipefail
ROOT=/root/gpufree-data/cicc_workspace
source /home/ubuntu/catkin_ws/devel/setup.bash
set -u

if [[ "${1:-}" != "--check" ]]; then
  bash "$ROOT/start_sim_manual.sh"
fi

ready=0
for _ in $(seq 1 18); do
  state=$(timeout 6 rostopic echo -n 1 /mavros/state 2>/dev/null || true)
  if grep -q 'connected: True' <<<"$state" &&
     grep -q 'armed: False' <<<"$state" &&
     pgrep -x RflySim3D >/dev/null &&
     pgrep -x CopterSim >/dev/null &&
     pgrep -x px4 >/dev/null &&
     timeout 5 rostopic echo -n 1 /rflysim/sensor1/img_rgb/header >/dev/null 2>&1 &&
     timeout 5 rostopic echo -n 1 /rflysim/sensor2/img_rgb/header >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 5
done

if (( ! ready )); then
  echo 'NOT READY: simulator, MAVROS or cameras unavailable; do not send a flight prompt.' >&2
  exit 1
fi
if [[ ! -d /home/ubuntu/model/FM9G4B-V || ! -f /home/ubuntu/model/prompts/mission_prompt.txt ]]; then
  echo 'NOT READY: Jiuge model or prompt knowledge file missing.' >&2
  exit 1
fi
echo 'READY: simulator + front/down cameras; aircraft connected and disarmed.'
echo 'Next, in this terminal run:'
echo "  bash $ROOT/run_mission_online.sh --once --timeout 300"
echo 'Wait for [待命], then paste ONE exact organizer-provided natural-language instruction and press Enter.'
