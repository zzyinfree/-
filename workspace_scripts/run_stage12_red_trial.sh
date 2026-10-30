#!/usr/bin/env bash
set -eo pipefail
ROOT=/root/gpufree-data/cicc_workspace
source /home/ubuntu/catkin_ws/devel/setup.bash
state=$(timeout 6 rostopic echo -n1 /mavros/state || true)
if ! grep -q 'connected: True' <<< "$state" || ! grep -q 'armed: False' <<< "$state"; then
  echo 'REFUSED: simulator is not connected and disarmed' >&2
  exit 2
fi
tmux kill-session -t cicc_stage12_capture 2>/dev/null || true
tmux new-session -d -s cicc_stage12_capture \
  "bash -lc 'source /home/ubuntu/catkin_ws/devel/setup.bash; /opt/conda/envs/demo/bin/python -u $ROOT/record_front_frames.py --output-dir $ROOT/recordings/run_023_red_stage12 --count 170 --interval 0.7 --timeout 180 --allow-partial --group-id run_023_red_stage12 --scene-note stage12_red_return_planner > $ROOT/outputs/stage12_capture.log 2>&1'"
cd "$ROOT"
bash run_mission_online.sh --once --timeout 210 --prompt \
  '飞机起飞，穿过方框，识别二维码，再穿回方框，打击红色气球，最后返回起点并降落。' \
  > outputs/stage12_red_online.log 2>&1
tail -12 outputs/stage12_red_online.log
