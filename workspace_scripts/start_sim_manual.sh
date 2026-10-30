#!/usr/bin/env bash
# Start only missing components of the local ChallengeMap test stack.
# Do not run a second copy of any simulator component already in tmux.
set -eo pipefail
ROOT=/root/gpufree-data/cicc_workspace
cd "$ROOT"
mkdir -p outputs

if ! test -S /tmp/.X11-unix/X20; then
  echo 'ERROR: Linux graphical desktop DISPLAY=:20 is not running. Start the platform desktop first.' >&2
  exit 2
fi

start_missing() {
  local name="$1" command="$2"
  if tmux has-session -t "$name" 2>/dev/null; then
    printf 'KEEP  %s\n' "$name"
  else
    tmux new-session -d -s "$name" "$command"
    printf 'START %s\n' "$name"
  fi
}

start_missing cicc_roscore "source /opt/ros/noetic/setup.bash; roscore > $ROOT/outputs/roscore_manual.log 2>&1"
sleep 3
start_missing cicc_ue "bash $ROOT/run_ue_only.sh > $ROOT/outputs/ue_manual.log 2>&1"
sleep 4
start_missing cicc_sitl "bash $ROOT/start_sitl_clone.sh"
sleep 12
start_missing cicc_sensor "bash $ROOT/run_sensor.sh > $ROOT/outputs/sensor_manual.log 2>&1"
sleep 5
start_missing cicc_down_view "source /home/ubuntu/catkin_ws/devel/setup.bash; export DISPLAY=:20; /opt/conda/envs/demo/bin/python -u $ROOT/downward_camera_view.py > $ROOT/outputs/downward_view_manual.log 2>&1"
start_missing cicc_map "bash $ROOT/run_map.sh > $ROOT/outputs/map_manual.log 2>&1"
start_missing cicc_det "bash $ROOT/run_det_only.sh > $ROOT/outputs/det_manual.log 2>&1"
start_missing cicc_aruco "source /home/ubuntu/catkin_ws/devel/setup.bash; roslaunch mission_pkg aruco_detect.launch > $ROOT/outputs/aruco_manual.log 2>&1"
start_missing cicc_cloud "source /home/ubuntu/catkin_ws/devel/setup.bash; export PATH=/opt/conda/envs/demo/bin:\$PATH; roslaunch sensor_pkg cloud_transform.launch > $ROOT/outputs/cloud_manual.log 2>&1"
start_missing cicc_planner_fast "bash $ROOT/run_planner_fast.sh > $ROOT/outputs/planner_manual.log 2>&1"
start_missing cicc_mission_fast "bash $ROOT/run_mission_patched.sh > $ROOT/outputs/mission_manual.log 2>&1"
start_missing cicc_safety "bash $ROOT/run_safety_supervisor.sh > $ROOT/outputs/safety_manual.log 2>&1"

source /home/ubuntu/catkin_ws/devel/setup.bash
echo '--- MAVROS state ---'
timeout 15 rostopic echo -n 1 /mavros/state | grep -E 'connected:|armed:|mode:' || true
echo '--- Required topics ---'
rostopic list | grep -E '^/(rflysim/sensor1/img_rgb|mission_command|mission_result)$' || true
echo '--- Sessions ---'
tmux list-sessions
echo 'Simulator startup issued. Verify connected=True, armed=False and the front-camera image before sending a prompt.'
