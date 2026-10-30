#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
cd /root/gpufree-data/cicc_workspace
state=$(timeout 8 rostopic echo -n1 /mavros/state)
if ! grep -q 'connected: True' <<<"$state" || ! grep -q 'armed: False' <<<"$state"; then
  echo 'Refusing trial: MAVROS disconnected or vehicle already armed' >&2
  exit 2
fi
exec bash run_mission_online.sh --once --timeout 170 \
  --prompt '起飞，穿框，识别二维码，再穿回框，打击蓝色气球，回到起点，降落。'
