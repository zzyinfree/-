#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
set -u
W=/root/gpufree-data/cicc_workspace
RUN=${RUN:-run_024_purple_stage13}
PROMPT=${PROMPT:-'飞机起飞，穿过方框，使用下视摄像头识别二维码，再安全穿回方框，识别并打击紫色气球，最后避开障碍返回起点降落。'}
mkdir -p "$W/recordings/$RUN/front" "$W/recordings/$RUN/down" "$W/outputs"
STATE=$(rostopic echo -n 1 /mavros/state)
printf '%s\n' "$STATE" | grep -q 'connected: True'
printf '%s\n' "$STATE" | grep -q 'armed: False'
python3 "$W/record_front_frames.py" --output-dir "$W/recordings/$RUN/front" --count 240 --interval 0.65 --timeout 185 --allow-partial --group-id "${RUN}_front" --scene-note '紫球整架前视，未人工标注' > "$W/outputs/${RUN}_front.log" 2>&1 &
FPID=$!
python3 "$W/record_front_frames.py" --output-dir "$W/recordings/$RUN/down" --count 240 --interval 0.65 --timeout 185 --allow-partial --group-id "${RUN}_down" --scene-note '二维码及返航下视，未人工标注' --topic /rflysim/sensor2/img_rgb > "$W/outputs/${RUN}_down.log" 2>&1 &
DPID=$!
cleanup() { kill "$FPID" "$DPID" 2>/dev/null || true; wait "$FPID" "$DPID" 2>/dev/null || true; }
trap cleanup EXIT
bash "$W/run_mission_online.sh" --once --timeout 210 --prompt "$PROMPT" 2>&1 | tee "$W/outputs/${RUN}_online.log"
