#!/usr/bin/env bash
# One natural-language trial in the restored clone; not a competition batch.
set -eo pipefail
ROOT=/root/gpufree-data/cicc_workspace
source /home/ubuntu/catkin_ws/devel/setup.bash
state=$(timeout 8 rostopic echo -n 1 /mavros/state)
if ! grep -q 'connected: True' <<<"$state" || ! grep -q 'armed: False' <<<"$state"; then
  echo 'Refusing trial: MAVROS disconnected or vehicle already armed' >&2
  exit 2
fi
cd "$ROOT"
exec bash run_mission_online.sh --once --timeout 170 \
  --prompt '起飞，穿框，识别二维码，再穿回框，打击红色气球，回到起点，降落。'
