/******************************************************************************
 * @file       target_tracker.h
 * @brief      TargetTracker 节点 - 简单像素跟踪
 *
 * @details    持续输出 pixel_target 用于视觉伺服
 *             设计：直接提取像素坐标，无需物理尺寸/3D转换/遮挡检测
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-13
 *****************************************************************************/
#ifndef TARGET_TRACKER_H
#define TARGET_TRACKER_H

#include "base/target_detector_base.h"
#include <deque>

class TargetTracker : public TargetDetectorBase {
public:
    TargetTracker(const std::string& name, const NodeConfig& config);
    
    static PortsList providedPorts();
    
    NodeStatus onStart() override;

protected:
    bool findTargetInObjects(const common_msgs::Objects& msg,
                             common_msgs::Obj& out_obj) override;
    NodeStatus handleDetectedObject(const common_msgs::Obj& obj) override;
    NodeStatus handleNoDetection() override;
    NodeStatus handleNoNewData() override;

private:
    // 像素坐标平滑（简单移动平均）
    std::deque<PixelTarget> pixel_history;
    int filter_window_size;
    double continuity_confidence_{0.55};
    double max_pixel_jump_{150.0};
    double continuity_window_{2.0};
    bool has_accepted_target_{false};
    bool last_frame_valid_{false};
    PixelTarget last_accepted_target_;
    ros::Time last_accepted_time_;
    
    PixelTarget getSmoothedPixel();
};

#endif // TARGET_TRACKER_H
