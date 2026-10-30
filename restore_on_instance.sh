#!/usr/bin/env bash
# Restore the competition sources onto a fresh clone of the same official simulator image.
set -euo pipefail

REPO_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROS_WS=/home/ubuntu/catkin_ws
RUN_DIR=/root/gpufree-data/cicc_workspace

usage() {
  echo "Usage: bash restore_on_instance.sh --check | --install"
  echo "Run this on the official Ubuntu competition instance, as root."
}

case "${1:-}" in
  --check|--install) ;;
  *) usage; exit 2 ;;
esac

missing=0
require_file() {
  if [[ ! -f "$1" ]]; then echo "MISSING file: $1" >&2; missing=1; fi
}
require_dir() {
  if [[ ! -d "$1" ]]; then echo "MISSING directory: $1" >&2; missing=1; fi
}
require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then echo "MISSING command: $1" >&2; missing=1; fi
}

require_dir "$REPO_DIR/ros_src/mission_pkg"
require_file "$REPO_DIR/workspace_scripts/competition_boot.sh"
require_dir "$ROS_WS/src"
require_file /opt/ros/noetic/setup.bash
export PATH="/opt/ros/noetic/bin:$PATH"
require_command catkin_make
require_command tmux
require_file /opt/conda/envs/demo/bin/python
require_file /home/ubuntu/PX4PSP/RflySimUE5/RflySim3D/Binaries/Linux/RflySim3D
require_file /home/ubuntu/PX4PSP/CopterSim/CopterSim
require_file /home/ubuntu/PX4PSP/Firmware/Tools/sitl_multiple_run_rfly.sh
require_dir /home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK
require_dir /home/ubuntu/model/FM9G4B-V
require_file /home/ubuntu/model/prompts/mission_prompt.txt
require_file "$ROS_WS/src/object_det/models/best.pt"
require_dir "$ROS_WS/src/object_det/third_party/ultralytics"
require_file "$RUN_DIR/run_ue_only.sh"
require_file "$RUN_DIR/start_sitl_clone.sh"

if (( missing )); then
  echo 'Environment is incomplete. Restore the official simulator image and model assets first.' >&2
  exit 1
fi

if [[ "$1" == --check ]]; then
  echo 'PASS: repository sources and required official-image assets are present.'
  echo 'This checks files only; it does not prove flight behavior or scoring.'
  exit 0
fi

if (( EUID != 0 )); then
  echo 'Run --install as root so the official workspace paths can be updated.' >&2
  exit 1
fi

for pkg in "$REPO_DIR"/ros_src/*; do
  [[ -d "$pkg" ]] || continue
  cp -a "$pkg" "$ROS_WS/src/"
done
mkdir -p "$RUN_DIR"
cp -a "$REPO_DIR"/workspace_scripts/. "$RUN_DIR/"

source /opt/ros/noetic/setup.bash
cd "$ROS_WS"
catkin_make

echo 'Installed and built. To verify the live simulator, run:'
echo "  bash $RUN_DIR/competition_boot.sh"
