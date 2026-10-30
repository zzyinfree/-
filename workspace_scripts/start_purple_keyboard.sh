#!/usr/bin/env bash
# Start manual waypoint survey only from a disarmed simulator.
set -euo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
ROOT=/root/gpufree-data/cicc_workspace
state=$(timeout 8 rostopic echo -n 1 /mavros/state)
if ! grep -q 'connected: True' <<<"$state" || ! grep -q 'armed: False' <<<"$state"; then
  echo '拒绝：必须先确认仿真连接正常、飞机未解锁。' >&2
  exit 2
fi
if tmux has-session -t cicc_mission_fast 2>/dev/null; then
  echo '暂停自动任务节点，避免键盘与任务同时发送飞控指令。'
  tmux kill-session -t cicc_mission_fast
  sleep 2
fi
exec /opt/conda/envs/demo/bin/python -u "$ROOT/purple_waypoint_keyboard.py"
