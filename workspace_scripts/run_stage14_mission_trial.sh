#!/usr/bin/env bash
# Development sortie: one natural-language prompt, two cameras and ROS telemetry.
# Not a substitute for entering the committee's prompts during the official run.
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
set -u
W=/root/gpufree-data/cicc_workspace
RUN=${RUN:?set RUN to a unique sortie identifier}
PROMPT=${PROMPT:?set PROMPT to the single natural-language test instruction}
DIR="$W/recordings/$RUN"
mkdir -p "$DIR/front" "$DIR/down" "$W/outputs"
STATE=$(timeout 8 rostopic echo -n 1 /mavros/state)
printf '%s\n' "$STATE" | grep -q 'connected: True'
printf '%s\n' "$STATE" | grep -q 'armed: False'
timeout 8 rostopic echo -n 1 /rflysim/sensor1/img_rgb/header >/dev/null
timeout 8 rostopic echo -n 1 /rflysim/sensor2/img_rgb/header >/dev/null
echo "$STATE" > "$DIR/preflight_state.txt"

python3 "$W/record_front_frames.py" --output-dir "$DIR/front" \
  --count 420 --interval 0.55 --timeout 310 --allow-partial \
  --group-id "${RUN}_front" --scene-note "$RUN complete front sortie; unverified" \
  > "$W/outputs/${RUN}_front.log" 2>&1 &
FPID=$!
python3 "$W/record_front_frames.py" --output-dir "$DIR/down" \
  --count 420 --interval 0.55 --timeout 310 --allow-partial \
  --group-id "${RUN}_down" --scene-note "$RUN complete down sortie; unverified" \
  --topic /rflysim/sensor2/img_rgb \
  > "$W/outputs/${RUN}_down.log" 2>&1 &
DPID=$!
rosbag record -O "$DIR/flight.bag" /mavros/local_position/odom \
  /rflysim/uav1/local/odom /mavros/state /mission_command /mission_result \
  /aruco/detected_id /objects /mavros/setpoint_raw/local /cloud_registered \
  /planning/pos_cmd /cicc/down_balloon_hint \
  > "$W/outputs/${RUN}_rosbag.log" 2>&1 &
BPID=$!
cleanup() {
  kill "$FPID" "$DPID" 2>/dev/null || true
  kill -INT "$BPID" 2>/dev/null || true
  wait "$FPID" "$DPID" "$BPID" 2>/dev/null || true
}
trap cleanup EXIT

set +e
bash "$W/run_mission_online.sh" --once --timeout 290 --prompt "$PROMPT" \
  2>&1 | tee "$W/outputs/${RUN}_online.log"
RC=${PIPESTATUS[0]}
set -e
STATE=$(timeout 8 rostopic echo -n 1 /mavros/state || true)
printf '%s\n' "$STATE" > "$DIR/postflight_state.txt"
echo "SORTIE_RC=$RC"
echo "$STATE" | grep -E 'connected:|armed:|mode:' || true
exit "$RC"
