/******************************************************************************
 * @file       takeoff.cpp
 * @brief      Takeoff 行为树节点 - 无人机起飞控制
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#include "nodes/action/takeoff.h"

Takeoff::Takeoff(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config) {}

PortsList Takeoff::providedPorts() {
    return { InputPort<Position3D>("goal", "起飞目标位置 (x, y, z)"),
             InputPort<double>("yaw", "起飞航向 (rad)，不指定则保持当前航向"),
             InputPort<int>("auto_arm", 1, "自动解锁起飞"),
             InputPort<double>("tolerance", 0.2, "到达容差"),
             OutputPort<Position3D>("home_hover", "实测起飞点上方的返航悬停点"),
             OutputPort<Position3D>("home_low", "实测起飞点上方的低空降落引导点") };
}

NodeStatus Takeoff::onStart() {
    const auto& launch = fcu_odom_ptr->pose.pose.position;
    Position3D home_hover{launch.x, launch.y, 1.8};
    Position3D home_low{launch.x, launch.y, 0.65};
    setOutput("home_hover", home_hover);
    setOutput("home_low", home_low);
    ROS_INFO("[%s]: recorded launch point (%.2f, %.2f, %.2f); return hover (%.2f, %.2f, %.2f)",
             name().c_str(), launch.x, launch.y, launch.z,
             home_hover.x, home_hover.y, home_hover.z);
    auto goal_res = getInput<Position3D>("goal");
    if (!goal_res) throw RuntimeError("Missing goal");
    goal_pos = goal_res.value();

    auto auto_arm_res = getInput<int>("auto_arm");
    is_rc_control = (auto_arm_res.value() == 0);  // auto_arm=0 表示需要遥控器，auto_arm=1 表示自动

    auto tol_res = getInput<double>("tolerance");
    tolerance = tol_res.value();

    // 读取 yaw 参数，未指定则使用当前航向
    auto yaw_res = getInput<double>("yaw");
    if (yaw_res) {
        takeoff_yaw_ = yaw_res.value();
    } else {
        takeoff_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    }

    ROS_INFO("[%s]: Takeoff start -> (%.2f, %.2f, %.2f), yaw: %.2f, tolerance: %.2f, %s", 
             name().c_str(), goal_pos.x, goal_pos.y, goal_pos.z, takeoff_yaw_, tolerance,
             is_rc_control ? "waiting for RC arm" : "auto arm enabled");
    return NodeStatus::RUNNING;
}

NodeStatus Takeoff::onRunning() {
    geometry_msgs::Point target;
    target.x = goal_pos.x;
    target.y = goal_pos.y;
    target.z = goal_pos.z;

    // 1. 发布位置指令 (维持心跳，保持起飞时的航向)
    publishPosition(target, takeoff_yaw_);

    // 2. 检查状态
    bool ready = (fcu_state_ptr->mode == "OFFBOARD" && fcu_state_ptr->armed);
    
    // 3. 自动切模式逻辑
    if (!is_rc_control && !ready) {
        if (fcu_state_ptr->mode != "OFFBOARD") {
            throttledSetMode("OFFBOARD");
        } else if (!fcu_state_ptr->armed) {
            throttledArm(true);
        }
    }

    // 4. 到达判断
    if (ready && checkArrival(target, tolerance)) {
        ROS_INFO("[%s]: Takeoff completed.", name().c_str());
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<Takeoff>("Takeoff");
}
