/******************************************************************************
 * @file       target_tracker.cpp
 * @brief      TargetTracker 节点实现
 *
 * @details    简单像素跟踪：直接提取像素坐标+简单平滑
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-13
 *****************************************************************************/
#include "nodes/perception/target_tracker.h"
#include <algorithm>
#include <cmath>

TargetTracker::TargetTracker(const std::string& name, const NodeConfig& config)
    : TargetDetectorBase(name, config) {}

PortsList TargetTracker::providedPorts() {
    auto ports = baseProvidedPorts();
    ports.insert(InputPort<int>("filter_window_size", 10, "像素坐标平滑窗口大小"));
    ports.insert(InputPort<double>("continuity_confidence", 0.55, "已锁定同类目标的最低临时置信度"));
    ports.insert(InputPort<double>("max_pixel_jump", 150.0, "低置信度连续检测的最大像素跳变"));
    ports.insert(OutputPort<PixelTarget>("pixel_target", "像素目标（中心+尺寸）"));
    return ports;
}

NodeStatus TargetTracker::onStart() {
    // 调用基类 onStart
    NodeStatus base_status = TargetDetectorBase::onStart();
    
    // 获取自己的参数
    auto window_res = getInput<int>("filter_window_size");
    filter_window_size = window_res.value();
    continuity_confidence_ = getInput<double>("continuity_confidence").value();
    max_pixel_jump_ = getInput<double>("max_pixel_jump").value();
    has_accepted_target_ = false;
    last_frame_valid_ = false;

    pixel_history.clear();
    
    ROS_INFO("[%s]: Pixel tracking filter window: %d", name().c_str(), filter_window_size);
    
    return base_status;
}

bool TargetTracker::findTargetInObjects(const common_msgs::Objects& msg,
                                        common_msgs::Obj& out_obj) {
    // A high-confidence same-class hit may establish/re-establish the track.
    if (TargetDetectorBase::findTargetInObjects(msg, out_obj)) return true;
    // Lower confidence is allowed only for the *same recently tracked target*.
    // This prevents lowering the global first-acquisition threshold.
    if (!has_accepted_target_ ||
        (ros::Time::now() - last_accepted_time_).toSec() > continuity_window_) return false;
    const common_msgs::Obj* best = nullptr;
    for (const auto& obj : msg.objects) {
        if (obj.class_name != target_class || obj.score < continuity_confidence_) continue;
        const double dist = std::hypot(obj.center_x - last_accepted_target_.center_x,
                                       obj.center_y - last_accepted_target_.center_y);
        const double area = (obj.right_bottom_x - obj.left_top_x) *
                            (obj.right_bottom_y - obj.left_top_y);
        const double old_area = last_accepted_target_.area();
        if (dist > max_pixel_jump_ || area <= 0 || old_area <= 0 ||
            area / old_area < 0.35 || area / old_area > 2.8) continue;
        if (!best || obj.score > best->score) best = &obj;
    }
    if (!best) return false;
    out_obj = *best;
    ROS_WARN_THROTTLE(1.0, "[%s]: continuing locked %s track at confidence %.2f (< %.2f)",
                      name().c_str(), target_class.c_str(), best->score, confidence_threshold);
    return true;
}

NodeStatus TargetTracker::handleDetectedObject(const common_msgs::Obj& obj) {
    // 直接提取像素坐标
    double pixel_width = obj.right_bottom_x - obj.left_top_x;
    double pixel_height = obj.right_bottom_y - obj.left_top_y;
    
    if (pixel_width <= 0 || pixel_height <= 0) {
        return handleNoDetection();  // 无效尺寸视为未检测到
    }
    
    // 添加到历史
    PixelTarget current_pixel(
        obj.center_x,
        obj.center_y,
        pixel_width,
        pixel_height,
        ros::Time::now().toSec(),
        true // ✅ is_detected = true
    );
    last_accepted_target_ = current_pixel;
    last_accepted_time_ = ros::Time::now();
    has_accepted_target_ = true;
    last_frame_valid_ = true;
    
    pixel_history.push_back(current_pixel);
    while (static_cast<int>(pixel_history.size()) > filter_window_size) {
        pixel_history.pop_front();
    }
    
    // 输出平滑后的像素坐标
    PixelTarget smoothed = getSmoothedPixel();
    smoothed.timestamp = ros::Time::now().toSec(); // 更新时间戳
    smoothed.is_detected = true;                   // 再次确认标志
    setOutput("pixel_target", smoothed);
    
    ROS_INFO_THROTTLE(0.5, "[%s]: Tracking at pixel (%.1f, %.1f)", 
                     name().c_str(), smoothed.center_x, smoothed.center_y);
    
    return NodeStatus::RUNNING;
}

NodeStatus TargetTracker::handleNoDetection() {
    last_frame_valid_ = false;
    // 即使未检测到，也发布数据，但标记 is_detected = false
    // 保留旧的平滑坐标（或者清零，视需求而定，这里选择发送空/旧数据+false标志）
    PixelTarget empty_target;
    empty_target.timestamp = ros::Time::now().toSec();
    empty_target.is_detected = false; // ❌ is_detected = false
    
    setOutput("pixel_target", empty_target);
    
    // 清空历史，避免"鬼影"（突然出现时用到旧历史平均）
    // 或者不清空以保持连续性？建议清空，因为断开了
    pixel_history.clear();
    
    ROS_WARN_THROTTLE(1.0, "[%s]: No target detected, publishing INVALID target", name().c_str());
    return NodeStatus::RUNNING;
}

NodeStatus TargetTracker::handleNoNewData() {
    // The BT ticks faster than /objects. Preserve the last valid detection
    // only until the next expected camera frame; do not invent a new vote.
    if (last_frame_valid_ &&
        (ros::Time::now() - last_accepted_time_).toSec() < 0.25) {
        PixelTarget held = getSmoothedPixel();
        held.timestamp = last_accepted_target_.timestamp;
        held.is_detected = true;
        setOutput("pixel_target", held);
        return NodeStatus::RUNNING;
    }
    return handleNoDetection();
}

PixelTarget TargetTracker::getSmoothedPixel() {
    if (pixel_history.empty()) {
        return PixelTarget(0, 0, 0, 0, ros::Time::now().toSec());
    }
    
    // 简单移动平均
    double sum_x = 0, sum_y = 0, sum_w = 0, sum_h = 0;
    for (const auto& px : pixel_history) {
        sum_x += px.center_x;
        sum_y += px.center_y;
        sum_w += px.width;
        sum_h += px.height;
    }
    
    int n = pixel_history.size();
    return PixelTarget(
        sum_x / n,
        sum_y / n,
        sum_w / n,
        sum_h / n,
        ros::Time::now().toSec()
    );
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<TargetTracker>("TargetTracker");
}
