/******************************************************************************
 * @file       planning_control.h
 * @brief      Planning Control 行为树节点声明
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#ifndef PLANNING_CONTROL_H
#define PLANNING_CONTROL_H

#include "base/drone_action_node.h"
#include <quadrotor_msgs/PositionCommand.h>
#include <sensor_msgs/PointCloud2.h>

class PlanningControl : public DroneActionNode {
public:
    PlanningControl(const std::string& name, const NodeConfig& config);
    
    static PortsList providedPorts();

    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;

private:
   void plannerCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
   void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
   void initializeTarget(const Position3D& pos);

    Position3D target_pos_;
    double tolerance;
    double offset_z;
    bool control_yaw;
    bool record_breadcrumbs_{false};
    Position3D breadcrumbs_[3];
    Position3D last_breadcrumb_pos_;
    double initial_yaw_{0.0};  // Starting yaw when control_yaw is false
    bool target_initialized;
    double max_tracking_error_{0.0};
    double min_obstacle_distance_{0.0};
    double forward_clearance_{100.0};
    double nearest_body_clearance_{100.0};
    ros::Time last_valid_cloud_time_;
    ros::Time goal_start_time_;
    ros::Time tracking_error_since_;
    ros::Time obstacle_hold_since_;
    bool obstacle_replan_pending_{false};
    int tracking_replans_{0};

    ros::Publisher planner_goal_pub;
    ros::Subscriber planner_cmd_sub;
    ros::Subscriber cloud_sub;
    quadrotor_msgs::PositionCommand last_planner_cmd;
    bool has_planner_cmd;
};

#endif // PLANNING_CONTROL_H
