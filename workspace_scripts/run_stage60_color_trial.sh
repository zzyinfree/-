#!/usr/bin/env bash
set -eo pipefail
ROOT=/root/gpufree-data/cicc_workspace
source /home/ubuntu/catkin_ws/devel/setup.bash
set -u
color="${1:?blue or red}"
run_id="${2:?run id}"
case "$color" in
  blue) cname=蓝色 ;;
  red) cname=红色 ;;
  *) echo "Unsupported color: $color" >&2; exit 2 ;;
esac
exec 9>"$ROOT/.stage60_flight.lock"
flock -n 9 || { echo 'Another trial is running' >&2; exit 3; }
state=$(timeout 10 rostopic echo -n 1 /mavros/state)
grep -q 'connected: True' <<< "$state"
grep -q 'armed: False' <<< "$state"
out="$ROOT/recordings/$run_id"
mkdir -p "$out/front" "$out/down" "$ROOT/outputs"
rosbag record -O "$out/flight.bag" \
  /mavros/local_position/odom /mavros/state /mavros/setpoint_raw/local \
  /rflysim/uav1/local/odom /objects /planning/pos_cmd \
  /move_base_simple/goal /mission_command /mission_result \
  > "$ROOT/outputs/${run_id}_rosbag.log" 2>&1 & bag_pid=$!
python3 "$ROOT/record_front_frames.py" --output-dir "$out/front" \
  --count 1000 --interval 0.6 --timeout 330 --allow-partial \
  --group-id "$run_id" --scene-note "full_${color}_trial_front" \
  > "$ROOT/outputs/${run_id}_front.log" 2>&1 & front_pid=$!
python3 "$ROOT/record_front_frames.py" --output-dir "$out/down" \
  --topic /rflysim/sensor2/img_rgb \
  --count 1000 --interval 0.6 --timeout 330 --allow-partial \
  --group-id "$run_id" --scene-note "full_${color}_trial_down" \
  > "$ROOT/outputs/${run_id}_down.log" 2>&1 & down_pid=$!
cleanup() {
  kill -INT "$bag_pid" 2>/dev/null || true
  kill -TERM "$front_pid" "$down_pid" 2>/dev/null || true
  wait "$bag_pid" "$front_pid" "$down_pid" 2>/dev/null || true
}
trap cleanup EXIT
sleep 2
set +e
bash "$ROOT/run_mission_online.sh" --once --timeout 300 \
  --prompt "起飞，穿过识别框，识别二维码，穿回识别框，搜索打击${cname}气球，返回起飞点降落" \
  2>&1 | tee "$ROOT/outputs/${run_id}_online.log"
rc=${PIPESTATUS[0]}
set -e
echo "TRIAL_EXIT_CODE=$rc"
timeout 10 rostopic echo -n 1 /mavros/state | grep -E 'connected:|armed:|mode:' || true
exit "$rc"
