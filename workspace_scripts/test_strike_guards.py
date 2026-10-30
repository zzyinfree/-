"""Fast static regression checks for the live strike-servo safety contract.

This does not establish a scored hit; it catches the false-success path observed
in run018 and unsafe experimental approach offsets before deployment.
"""

from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE / "source_patches"
if not ROOT.is_dir():
    ROOT = Path("/home/ubuntu/catkin_ws/src/mission_pkg")
    servo_path = ROOT / "src/nodes/action/visual_servo.cpp"
    launch_path = ROOT / "launch/llm_mission.launch"
else:
    servo_path = ROOT / "visual_servo.cpp"
    launch_path = ROOT / "llm_mission.launch"
bt_path = ROOT / "src/bt_ros.cpp" if ROOT.name == "mission_pkg" else ROOT / "bt_ros.cpp"
servo = servo_path.read_text(encoding="utf-8")
launch = launch_path.read_text(encoding="utf-8")
bt = bt_path.read_text(encoding="utf-8")

lost = servo.split("if (current_state_ != SPRINTING && isTargetLost()) {", 1)[1]
lost = lost.split("// 计算基本量", 1)[0]
close_lost = lost.split("if (max_area_ratio_ > approach_area_threshold_) {", 1)[1]
close_lost = close_lost.split("// 搜索模式", 1)[0]
assert "NodeStatus::FAILURE" in close_lost
assert "NodeStatus::SUCCESS" not in close_lost
assert "strike unverified" in close_lost

approach = servo.split("case APPROACHING: {", 1)[1].split("case SPRINTING: {", 1)[0]
assert approach.index("current_ratio >= approach_area_threshold_ && !isCentered()") < approach.index(
    "current_state_ = SPRINTING"
)
assert 'value="5.3;1.0;1.0"' in launch
assert 'value="4.0;1.0;0.7"' in launch
assert 'value="0.45"' in launch
assert "min_flight_z_ && vz < 0.0" in servo
assert 'name="visual_servo/approach_area_threshold_blue" value="0.025"' in launch
assert '"visual_servo/approach_area_threshold_" + color' in bt
assert '"visual_servo/k_p_yaw_rate_" + color' in bt
assert '"visual_servo/max_yaw_rate_" + color' in bt
assert '"visual_servo/center_tolerance_" + color' in bt
assert 'name="visual_servo/max_yaw_rate_blue"' in launch
assert 'name="visual_servo/k_d_yaw_rate_blue"       value="0.001"' in launch
print("PASS: false-success blocked, off-axis sprint gated, slow approach and descent floor configured")
