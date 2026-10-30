/******************************************************************************
 * @file       goto.h
 * @brief      GoTo 行为树节点声明
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#ifndef GOTO_H
#define GOTO_H

#include "base/drone_action_node.h"
#include <sensor_msgs/PointCloud2.h>
#include <mutex>
#include <array>

class GoTo : public DroneActionNode {
public:
    GoTo(const std::string& name, const NodeConfig& config);
    
    static PortsList providedPorts();

    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;

private:
    geometry_msgs::Point target_pos;
    double target_yaw;
    double tolerance;
    double yaw_tolerance;
    double max_setpoint_distance_ = 0.45;
    double return_ceiling_guard_ = 2.15;

    // 偏航控制相关
    double yaw_rate_limit;
    double current_yaw_sp;
    ros::Time last_loop_time;
    bool check_yaw;
    bool face_travel_ = false;
    bool front_avoidance_ = false;
    bool shortest_detour_ = false;
    bool prefer_right_detour_ = false;
    bool climb_first_pending_ = false;
    geometry_msgs::Point climb_anchor_;
    bool avoiding_ = false;
    int avoidance_attempts_ = 0;
    geometry_msgs::Point avoid_target_;
    geometry_msgs::Point turn_anchor_;
    bool turning_ = false;
    bool braking_ = false;
    ros::Time turn_settled_since_;
    ros::Time brake_settled_since_;
    geometry_msgs::Point last_measured_pos_;
    bool record_breadcrumbs_ = false;
    Position3D last_breadcrumb_pos_;
    std::array<Position3D, 3> breadcrumbs_;
    ros::Subscriber front_cloud_sub_;
    ros::Subscriber inflated_map_sub_;
    sensor_msgs::PointCloud2ConstPtr latest_cloud_;
    sensor_msgs::PointCloud2ConstPtr latest_inflated_map_;
    ros::Time cloud_received_;
    ros::Time inflated_map_received_;
    std::mutex cloud_mutex_;
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void inflatedMapCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
};

#endif // GOTO_H
