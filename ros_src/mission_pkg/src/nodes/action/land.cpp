/******************************************************************************
 * @file       land.cpp
 * @brief      Land 行为树节点 - 无人机降落控制
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#include "nodes/action/land.h"
#include "utils/pose_utils.h"
#include <algorithm>
#include <cmath>

Land::Land(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config) {}

PortsList Land::providedPorts() {
    return { InputPort<int>("use_speed", 0, "0: AUTO.LAND, 1: 速度下降"),
             InputPort<Position3D>("center_goal", "起飞点中心，先定点下降再自动降落") };
}

NodeStatus Land::onStart() {
    auto res = getInput<int>("use_speed");
    use_speed_mode = (res.value() != 0);
    auto center = getInput<Position3D>("center_goal");
    center_descent_mode_ = bool(center);
    if (center_descent_mode_) {
        center_goal_ = center.value();
        center_descent_start_ = ros::Time::now();
        center_descent_start_z_ = fcu_odom_ptr->pose.pose.position.z;
        center_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    }
    return NodeStatus::RUNNING;
}

NodeStatus Land::onRunning() {
    if (!fcu_state_ptr->armed) {
        ROS_INFO("[%s]: Landed and disarmed.", name().c_str());
        return NodeStatus::SUCCESS;
    }
    if (use_speed_mode) {
        publishVelocityWithYawRate(0, 0, -0.3, 0); // 0.3m/s 下降，yaw_rate=0保持航向
    } else if (center_descent_mode_) {
        const auto& p = fcu_odom_ptr->pose.pose.position;
        const double elapsed = (ros::Time::now() - center_descent_start_).toSec();
        const double horizontal_error = std::hypot(p.x - center_goal_.x, p.y - center_goal_.y);
        if (elapsed < 18.0 && (p.z > 0.17 || horizontal_error > 0.08)) {
            geometry_msgs::Point setpoint;
            setpoint.x = center_goal_.x;
            setpoint.y = center_goal_.y;
            setpoint.z = std::max(0.12, center_descent_start_z_ - 0.30 * elapsed);
            publishPosition(setpoint, center_yaw_);
            ROS_INFO_THROTTLE(1.0, "[%s]: center descent actual=(%.2f,%.2f,%.2f) error=%.2fm target_z=%.2f",
                              name().c_str(), p.x, p.y, p.z, horizontal_error, setpoint.z);
            return NodeStatus::RUNNING;
        }
        ROS_INFO_THROTTLE(2.0, "[%s]: center descent handoff at (%.2f,%.2f,%.2f), error=%.2fm",
                          name().c_str(), p.x, p.y, p.z, horizontal_error);
        throttledSetMode("AUTO.LAND");
    } else {
        throttledSetMode("AUTO.LAND");
    }

    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<Land>("Land");
}
