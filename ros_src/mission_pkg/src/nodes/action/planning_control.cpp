/******************************************************************************
 * @file       planning_control.cpp
 * @brief      Planning Control 行为树节点 - 规划器控制
 *
 * @details    与外部路径规划器交互，执行复杂导航任务
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#include "nodes/action/planning_control.h"
#include "utils/pose_utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.h>

PlanningControl::PlanningControl(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config), has_planner_cmd(false) {
    
    auto nh = mct->getROSHandle();
    std::string goal_topic, cmd_topic;
    nh->param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
    nh->param<std::string>("planner_cmd_topic", cmd_topic, "/planning/pos_cmd");

    planner_goal_pub = nh->advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    planner_cmd_sub = nh->subscribe(cmd_topic, 1, &PlanningControl::plannerCmdCallback, this);
    cloud_sub = nh->subscribe("/cloud_registered", 1, &PlanningControl::cloudCallback, this);
}

PortsList PlanningControl::providedPorts() {
    return { InputPort<Position3D>("goal", "目标点(x;y;z)"),
             InputPort<double>("offset_z", 0.0, "相对于目标的Z轴偏移(随飞机航向)"),
             InputPort<double>("tolerance", "到达容差"),
             InputPort<bool>("control_yaw", false, "是否控制偏航角"),
             InputPort<double>("max_tracking_error", 0.0, "超过该跟踪误差则悬停并中止; 0=关闭"),
             InputPort<double>("min_obstacle_distance", 0.0, "前方点云停车距离; 0=关闭"),
             InputPort<bool>("record_breadcrumbs", false, "记录实际安全飞行路径"),
             OutputPort<Position3D>("trail_1"),
             OutputPort<Position3D>("trail_2"),
             OutputPort<Position3D>("trail_3"),
             OutputPort<Position3D>("final_position", "到达的最终位置(x;y;z)") };
}

void PlanningControl::plannerCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    // The trajectory server keeps publishing the previous goal during replan.
    // Never forward an old segment into a newly started return leg.
    if (!target_initialized ||
        std::hypot(msg->goal_pos.x - target_pos_.x,
                   msg->goal_pos.y - target_pos_.y) > 0.15 ||
        std::abs(msg->goal_pos.z - target_pos_.z) > 0.15) return;
    last_planner_cmd = *msg;
    has_planner_cmd = true;
}

void PlanningControl::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!target_initialized || min_obstacle_distance_ <= 0.0 ||
        !fcu_odom_ptr || msg->width * msg->height == 0) return;
    // Registered cloud is in world/ENU; ignore invalid and floor points.
    const auto& p = fcu_odom_ptr->pose.pose.position;
    double dx = (has_planner_cmd ? last_planner_cmd.position.x : target_pos_.x) - p.x;
    double dy = (has_planner_cmd ? last_planner_cmd.position.y : target_pos_.y) - p.y;
    double norm = std::hypot(dx, dy);
    if (norm < 0.10) {
        dx = target_pos_.x - p.x;
        dy = target_pos_.y - p.y;
        norm = std::hypot(dx, dy);
    }
    if (norm < 0.10) return;
    dx /= norm; dy /= norm;
    double nearest = 100.0, omni_nearest = 100.0;
    try {
        sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
        for (; x != x.end(); ++x, ++y, ++z) {
            if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
            const double rx = *x - p.x, ry = *y - p.y;
            const double ahead = rx * dx + ry * dy;
            const double side = std::abs(-rx * dy + ry * dx);
            if (std::abs(*z - p.z) < 0.32)
                omni_nearest = std::min(omni_nearest, std::hypot(rx, ry));
            if (ahead > 0.08 && ahead < min_obstacle_distance_ &&
                side < 0.28 && std::abs(*z - p.z) < 0.32) nearest = std::min(nearest, ahead);
        }
    } catch (const std::runtime_error& e) {
        ROS_WARN_THROTTLE(2.0, "[PlanningControl]: invalid registered cloud: %s", e.what());
        return;
    }
    forward_clearance_ = nearest;
    nearest_body_clearance_ = omni_nearest;
    last_valid_cloud_time_ = ros::Time::now();
}

void PlanningControl::initializeTarget(const Position3D& pos) {
    target_pos_ = pos;

    // 计算带有偏移的目标点 (沿着飞机当前位姿的机头方向投射)
    tf2::Quaternion q_curr;
    tf2::fromMsg(fcu_odom_ptr->pose.pose.orientation, q_curr);
    double r, p, yaw;
    tf2::Matrix3x3(q_curr).getRPY(r, p, yaw);

    double target_x = target_pos_.x + offset_z * cos(yaw);
    double target_y = target_pos_.y + offset_z * sin(yaw);
    double target_z = target_pos_.z;

    // 向规划器发送目标（都使用 ENU，直接发送）
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "world";
    ps.pose.position.x = target_x;
    ps.pose.position.y = target_y;
    ps.pose.position.z = target_z;
    
    tf2::Quaternion q_tgt;
    q_tgt.setRPY(0, 0, 0); 
    ps.pose.orientation = tf2::toMsg(q_tgt);

    planner_goal_pub.publish(ps);
    has_planner_cmd = false;
    last_valid_cloud_time_ = ros::Time(0);
    forward_clearance_ = 100.0;
    nearest_body_clearance_ = 100.0;
    goal_start_time_ = ros::Time::now();
    tracking_error_since_ = ros::Time(0);
    obstacle_hold_since_ = ros::Time(0);
    obstacle_replan_pending_ = false;

    ROS_ERROR("[%s]: >>> NED goal to ego=(%.2f, %.2f, %.2f), yaw=%.2f, offset=%.2f <<<",
             name().c_str(), ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, yaw, offset_z);

    // 为了 checkArrival 判断，更新目标点
    target_pos_.x = target_x;
    target_pos_.y = target_y;

    target_initialized = true;
    ROS_INFO("[%s]: Target acquired! Planning navigation start -> (%.2f, %.2f, %.2f), offset: %.2f", 
             name().c_str(), target_x, target_y, target_z, offset_z);
}

NodeStatus PlanningControl::onStart() {
    target_initialized = false;
    has_planner_cmd = false;
    auto off_res = getInput<double>("offset_z");
    offset_z = off_res.value();

    auto tol_res = getInput<double>("tolerance");
    if (tol_res) {
        tolerance = tol_res.value();
    } else {
        ros::NodeHandle nh;
        nh.param("tolerance", tolerance, 0.2);
    }

    auto yaw_res = getInput<bool>("control_yaw");
    control_yaw = yaw_res.value();
    record_breadcrumbs_ = getInput<bool>("record_breadcrumbs").value();
    const auto& start = fcu_odom_ptr->pose.pose.position;
    last_breadcrumb_pos_ = {start.x, start.y, start.z};
    for (auto& mark : breadcrumbs_) mark = last_breadcrumb_pos_;
    max_tracking_error_ = getInput<double>("max_tracking_error").value();
    min_obstacle_distance_ = getInput<double>("min_obstacle_distance").value();
    tracking_replans_ = 0;

    ROS_INFO("[%s]: Initialized with offset_z=%.2f, tolerance=%.2f, control_yaw=%d",
             name().c_str(), offset_z, tolerance, control_yaw);

    // 捕获初始偏航角 (供 !control_yaw 使用)
    initial_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    
    // 尝试获取目标点
    auto goal_res = getInput<Position3D>("goal");
    if (!goal_res) {
        ROS_INFO("[%s]: Waiting for goal...", name().c_str());
        target_initialized = false;
        sendHoverCommand();
        return NodeStatus::RUNNING;
    }

    // 初始化目标
    initializeTarget(goal_res.value());
    return NodeStatus::RUNNING;
}

NodeStatus PlanningControl::onRunning() {
    // 如果目标还未初始化，尝试获取
    if (!target_initialized) {
        auto goal_res = getInput<Position3D>("goal");
        if (!goal_res) {
            sendHoverCommand();
            return NodeStatus::RUNNING;
        }
        
        initializeTarget(goal_res.value());
        ROS_INFO("[%s]: Target acquired!", name().c_str());
    }

    if (record_breadcrumbs_) {
        const auto& p = fcu_odom_ptr->pose.pose.position;
        if (std::hypot(p.x - last_breadcrumb_pos_.x,
                       p.y - last_breadcrumb_pos_.y) >= 0.55) {
            breadcrumbs_[0] = breadcrumbs_[1];
            breadcrumbs_[1] = breadcrumbs_[2];
            breadcrumbs_[2] = {p.x, p.y, p.z};
            last_breadcrumb_pos_ = breadcrumbs_[2];
        }
    }

    // 检查是否到达 (3D 距离)
    geometry_msgs::Point target;
    target.x = target_pos_.x;
    target.y = target_pos_.y;
    target.z = target_pos_.z;
    if (checkArrival(target, tolerance)) {
        ROS_INFO("[%s]: Planning navigation completed.", name().c_str());
        
        // 输出最终到达的位置
        Position3D final_pos;
        final_pos.x = fcu_odom_ptr->pose.pose.position.x;
        final_pos.y = fcu_odom_ptr->pose.pose.position.y;
        final_pos.z = fcu_odom_ptr->pose.pose.position.z;
        setOutput("final_position", final_pos);
        if (record_breadcrumbs_) {
            setOutput("trail_1", breadcrumbs_[0]);
            setOutput("trail_2", breadcrumbs_[1]);
            setOutput("trail_3", breadcrumbs_[2]);
        }
        
        ROS_INFO("[%s]: Final position: (%.2f, %.2f, %.2f)", 
                 name().c_str(), final_pos.x, final_pos.y, final_pos.z);
        
        sendHoverCommand();
        return NodeStatus::SUCCESS;
    }

    const ros::Time now = ros::Time::now();
    // A moving balloon carrier may temporarily occupy the return corridor.
    // Wait at a captured hover point rather than either flying through it or
    // immediately aborting the whole mission. Replan once the cloud clears.
    if (min_obstacle_distance_ > 0.0 &&
        (nearest_body_clearance_ < 0.45 ||
         forward_clearance_ < min_obstacle_distance_)) {
        sendHoverCommand();
        if (obstacle_hold_since_.isZero()) obstacle_hold_since_ = now;
        obstacle_replan_pending_ = true;
        ROS_WARN_THROTTLE(1.0, "[%s]: return corridor occupied (omni %.2fm, ahead %.2fm); hovering for clearance",
                          name().c_str(), nearest_body_clearance_, forward_clearance_);
        if ((now - obstacle_hold_since_).toSec() > 10.0) {
            ROS_ERROR("[%s]: return corridor blocked for 10s; aborting instead of flying blind", name().c_str());
            return NodeStatus::FAILURE;
        }
        return NodeStatus::RUNNING;
    }
    if (obstacle_replan_pending_) {
        obstacle_hold_since_ = ros::Time(0);
        obstacle_replan_pending_ = false;
        auto goal_res = getInput<Position3D>("goal");
        if (!goal_res) return NodeStatus::FAILURE;
        initializeTarget(goal_res.value());
        sendHoverCommand();
        ROS_INFO("[%s]: return corridor cleared; replanning from current hover", name().c_str());
        return NodeStatus::RUNNING;
    }

    if (!has_planner_cmd || (now - last_planner_cmd.header.stamp).toSec() > 0.5) {
        sendHoverCommand();
        if ((now - goal_start_time_).toSec() > 3.0) {
            ROS_ERROR("[%s]: no fresh trajectory for current goal; aborting navigation", name().c_str());
            return NodeStatus::FAILURE;
        }
        return NodeStatus::RUNNING;
    }

    if (min_obstacle_distance_ > 0.0) {
        if ((now - goal_start_time_).toSec() > 2.0 &&
            (last_valid_cloud_time_.isZero() ||
             (now - last_valid_cloud_time_).toSec() > 1.0)) {
            ROS_ERROR("[%s]: depth cloud stale; refusing blind return", name().c_str());
            sendHoverCommand();
            return NodeStatus::FAILURE;
        }
    }

    if (max_tracking_error_ > 0.0 && (now - goal_start_time_).toSec() > 1.0) {
        const auto& p = fcu_odom_ptr->pose.pose.position;
        const auto& c = last_planner_cmd.position;
        const double error = std::sqrt(std::pow(p.x - c.x, 2) +
                                       std::pow(p.y - c.y, 2) +
                                       std::pow(p.z - c.z, 2));
        if (error > max_tracking_error_) {
            if (tracking_error_since_.isZero()) tracking_error_since_ = now;
            sendHoverCommand();
            if ((now - tracking_error_since_).toSec() > 0.25) {
                if (++tracking_replans_ <= 2) {
                    auto goal_res = getInput<Position3D>("goal");
                    if (!goal_res) return NodeStatus::FAILURE;
                    initializeTarget(goal_res.value());
                    tracking_error_since_ = ros::Time(0);
                    ROS_WARN("[%s]: tracking error %.2fm; hover and replan attempt %d/2",
                             name().c_str(), error, tracking_replans_);
                    return NodeStatus::RUNNING;
                }
                ROS_ERROR("[%s]: tracking error %.2fm persisted after replans; stopping before obstacle",
                          name().c_str(), error);
                return NodeStatus::FAILURE;
            }
            return NodeStatus::RUNNING;
        }
        tracking_error_since_ = ros::Time(0);
    }

    // Ego-planner 输出经过 publishPosition 做 ENU→NED 转换
    geometry_msgs::Point ego_pos = last_planner_cmd.position;
    double ego_yaw = control_yaw ? last_planner_cmd.yaw : initial_yaw_;
    
    publishPosition(ego_pos, ego_yaw);
    
    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<PlanningControl>("PlanningControl");
}
