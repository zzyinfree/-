#!/usr/bin/env python3
"""Interactive waypoint survey for the competition simulator, not a mission autopilot."""

import json
import math
import os
import select
import sys
import termios
import time
import tty
from datetime import datetime

import rosgraph
import rospy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry


ROOT = "/root/gpufree-data/cicc_workspace"
SAVE_PATH = os.path.join(ROOT, "outputs", "purple_waypoint_manual.json")
RATE = 20
STEP = 0.10
YAW_STEP = math.radians(15)
LIMITS = ((-0.80, 5.80), (-2.80, 2.80), (0.32, 2.20))


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Survey:
    def __init__(self):
        self.state = None
        self.odom = None
        self.goal = None
        self.yaw = 0.0
        self.message = "等待定位"
        self.pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        rospy.Subscriber("/mavros/state", State, self.on_state, queue_size=1)
        rospy.Subscriber("/rflysim/uav1/local/odom", Odometry, self.on_odom, queue_size=1)
        self.arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    def on_state(self, msg):
        self.state = msg

    def on_odom(self, msg):
        self.odom = msg

    def measured(self):
        p = self.odom.pose.pose.position
        return [p.x, p.y, p.z]

    def publishers_clear(self):
        # The official mission node continuously publishes a stale setpoint
        # even when no mission is active. Never share its control topic.
        try:
            pubs, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            for topic, nodes in pubs:
                if topic == "/mavros/setpoint_raw/local":
                    others = [n for n in nodes if n != rospy.get_name()]
                    return not others, others
        except Exception as exc:
            return False, [str(exc)]
        return True, []

    def publish(self):
        if self.goal is None:
            return
        x, y, z = self.goal
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "purple_keyboard"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                         PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                         PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                         PositionTarget.IGNORE_YAW_RATE | PositionTarget.FORCE)
        # Match mission_pkg::publishPosition's coordinates exactly.
        msg.position.x = -y
        msg.position.y = x
        msg.position.z = z
        msg.yaw = self.yaw + math.pi / 2
        self.pub.publish(msg)

    def set_goal(self, point):
        if any(not lo <= v <= hi for v, (lo, hi) in zip(point, LIMITS)):
            self.message = "拒绝：位置超过手动探点边界"
            return
        actual = self.measured()
        if math.dist(point, actual) > 0.45:
            self.message = "拒绝：目标超过实测位置45cm，请等飞机跟上"
            return
        self.goal = point
        self.message = "目标已更新"

    def hold(self):
        self.goal = self.measured()
        self.yaw = yaw_from_quat(self.odom.pose.pose.orientation)
        self.message = "保持当前位置"

    def move_body(self, forward, right, up=0.0):
        # Positive body-right is negative ENU-left.
        dx = forward * math.cos(self.yaw) + right * math.sin(self.yaw)
        dy = forward * math.sin(self.yaw) - right * math.cos(self.yaw)
        self.set_goal([self.goal[0] + dx, self.goal[1] + dy, self.goal[2] + up])

    def save(self):
        measured = self.measured()
        record = {
            "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "measured_enu_m": dict(zip(("x", "y", "z"), measured)),
            "target_enu_m": dict(zip(("x", "y", "z"), self.goal)),
            "measured_yaw_rad": yaw_from_quat(self.odom.pose.pose.orientation),
            "target_yaw_rad": self.yaw,
            "note": "Purple corner survey. Do not copy this into mission params without a path test."
        }
        os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
        with open(SAVE_PATH, "w", encoding="utf-8") as file:
            json.dump(record, file, ensure_ascii=False, indent=2)
        self.message = "坐标已保存到 " + SAVE_PATH

    def takeoff(self):
        if self.state.armed:
            self.message = "已经解锁，忽略起飞键"
            return
        self.hold()
        self.goal[2] = 0.90
        for _ in range(30):
            self.publish()
            time.sleep(1.0 / RATE)
        result = self.mode(0, "OFFBOARD")
        if not result.mode_sent:
            self.message = "OFFBOARD 切换失败，未解锁"
            return
        time.sleep(0.3)
        armed = self.arm(True)
        self.message = "起飞至 0.90 m" if armed.success else "解锁失败"

    def land(self):
        try:
            result = self.mode(0, "AUTO.LAND")
            self.message = "已请求 AUTO.LAND" if result.mode_sent else "降落请求失败，请人工接管"
        except rospy.ServiceException as exc:
            self.message = "降落服务异常，请人工接管：" + str(exc)


def key_ready():
    readable, _, _ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.read(1) if readable else None


def main():
    if not sys.stdin.isatty():
        raise SystemExit("必须在虚拟机的交互终端运行，不能用后台运行。")
    rospy.init_node("purple_waypoint_keyboard", anonymous=False)
    survey = Survey()
    start = time.monotonic()
    while not rospy.is_shutdown() and (survey.state is None or survey.odom is None):
        if time.monotonic() - start > 8:
            raise SystemExit("8 秒内没有收到 MAVROS 状态和里程计；检查仿真。")
        time.sleep(0.1)
    if not survey.state.connected:
        raise SystemExit("MAVROS 未连接。")
    if survey.state.armed:
        raise SystemExit("飞机已在飞行；本脚本只允许从未解锁状态接管。")
    clear, others = survey.publishers_clear()
    if not clear:
        raise SystemExit("已有其他飞控指令发布者：" + ", ".join(others))
    survey.hold()
    old_term = termios.tcgetattr(sys.stdin.fileno())
    print("\n紫球手动探点：t起飞  w/s前后  a/d左右  r/f升降  q/e逆/顺时针15°  U逆时针90°")
    print("空格悬停  g保存实测坐标  l降落  x退出（空中退出先自动降落）。每次位移10cm。")
    tick = time.monotonic()
    control_check = tick
    try:
        tty.setcbreak(sys.stdin.fileno())
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            survey.publish()
            if time.monotonic() - control_check >= 2.0:
                control_check = time.monotonic()
                clear, others = survey.publishers_clear()
                if not clear or not survey.state.connected:
                    survey.message = "控制冲突或连接断开：" + ", ".join(others)
                    survey.land()
                    break
            key = key_ready()
            if key:
                if key == "t":
                    survey.takeoff()
                elif key == "l":
                    survey.land()
                elif key == "x":
                    break
                elif key == " ":
                    survey.hold()
                elif key == "g":
                    survey.save()
                elif survey.state.armed and survey.state.mode == "OFFBOARD":
                    if key == "w": survey.move_body(STEP, 0)
                    elif key == "s": survey.move_body(-STEP, 0)
                    elif key == "a": survey.move_body(0, -STEP)
                    elif key == "d": survey.move_body(0, STEP)
                    elif key == "r": survey.move_body(0, 0, STEP)
                    elif key == "f": survey.move_body(0, 0, -STEP)
                    elif key == "q": survey.yaw = wrap(survey.yaw + YAW_STEP)
                    elif key == "e": survey.yaw = wrap(survey.yaw - YAW_STEP)
                    elif key == "U": survey.yaw = wrap(survey.yaw + math.pi / 2)
                else:
                    survey.message = "先按 t 起飞并确认 OFFBOARD，才能移动"
            if time.monotonic() - tick >= 0.5:
                actual = survey.measured()
                print("\r实测 x={:+.2f} y={:+.2f} z={:.2f} | 目标 x={:+.2f} y={:+.2f} z={:.2f} yaw={:+.0f}° | {}       ".format(
                    *actual, *survey.goal, math.degrees(survey.yaw), survey.message), end="", flush=True)
                tick = time.monotonic()
            rate.sleep()
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
        if survey.state and survey.state.armed:
            survey.land()
            print("\n已请求自动降落，请目视确认飞机落稳、armed=False。")
        else:
            print("\n手动探点已退出。")


if __name__ == "__main__":
    main()
