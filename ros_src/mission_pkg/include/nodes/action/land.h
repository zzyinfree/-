/******************************************************************************
 * @file       land.h
 * @brief      Land 行为树节点声明
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#ifndef LAND_H
#define LAND_H

#include "base/drone_action_node.h"

class Land : public DroneActionNode {
public:
    Land(const std::string& name, const NodeConfig& config);
    
    static PortsList providedPorts();

    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;

private:
    bool use_speed_mode;
    bool center_descent_mode_{false};
    BT::Position3D center_goal_;
    ros::Time center_descent_start_;
    double center_descent_start_z_{0.0};
    double center_yaw_{0.0};
};

#endif // LAND_H
