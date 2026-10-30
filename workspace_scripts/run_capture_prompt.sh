#!/usr/bin/env bash
set -eo pipefail
source /home/ubuntu/catkin_ws/devel/setup.bash
cd /root/gpufree-data/cicc_workspace
group_id="${1:?usage: run_capture_prompt.sh GROUP_ID}"
count="${2:-180}"
capture_timeout="${3:-145}"
exec python3 record_front_frames.py \
  --output-dir "recordings/$group_id" \
  --count "$count" --interval 0.65 --timeout "$capture_timeout" \
  --group-id "$group_id" \
  --scene-note "prompt_driven_${group_id}"
