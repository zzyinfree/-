/******************************************************************************
 * @file       visual_servo.h
 * @brief      Visual Servo - 三阶段控制（body系 VX+VZ+yaw_rate 控制版）
 * 
 * @details    ALIGN → APPROACH → SPRINT
 *             全程使用 publishVelocityWithYawRate (body 坐标系)
 ******************************************************************************/
#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#include "base/drone_action_node.h"
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <mutex>
#include <atomic>
#include <opencv2/core.hpp>

class VisualServo : public DroneActionNode {
public:
    VisualServo(const std::string& name, const NodeConfig& config);
    static PortsList providedPorts();
    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;

private:
    // 三阶段状态机
    enum ServoState {
        ALIGN,      // 对齐：原地旋转 yaw_rate，使目标水平居中
        APPROACHING, // 靠近：VX 前进 + yaw_rate + VZ 修正
        SPRINTING,  // 冲刺：全速 VX 冲刺一段固定时间
        BRAKE_PULSE, // 冲刺后短时反向制动，同时向2m爬升
        BRAKING,    // 停稳后才允许规划器接管返航
        RECOVERING, // 近距瞬时漏检：悬停复核，不盲追
        BOUNDARY_HOLD, // 紫球靠墙时停止追击，向场内稳定后再搜索
        POST_STRIKE_VERIFY,
        RETRY_TRANSIT,
        RETRY_DESCENT
    };
    
    // 状态
    ServoState current_state_;
    PixelTarget last_target_;
    ros::Time last_valid_detection_time_;
    bool has_ever_received_data_;
    double max_area_ratio_;
    ros::Time sprint_start_time_;
    ros::Time gated_restrike_not_before_;
    ros::Time unsafe_sprint_first_time_;
    ros::Time sprint_recheck_after_;
    int unsafe_sprint_rejections_{0};
    geometry_msgs::Point boundary_hold_anchor_;
    ros::Time boundary_hold_start_;
    ros::Time boundary_hold_settled_since_;
    geometry_msgs::Point boundary_hold_prev_pos_;
    ros::Time boundary_hold_prev_time_;
    int boundary_hold_count_{0};
    int boundary_hold_reanchors_{0};
    ros::Time brake_start_time_;
    ros::Time brake_pulse_start_time_;
    ros::Time brake_window_start_;
    geometry_msgs::Point brake_prev_pos_;
    double brake_window_distance_{0.0};
    geometry_msgs::Point sprint_origin_;
    geometry_msgs::Point verification_anchor_;
    double verification_yaw_{0.0};
    ros::Time verification_start_time_;
    ros::Time phase_start_time_;
    int strike_attempts_{0};
    int sprint_confirmations_{0};
    double sprint_last_detection_ts_{0.0};
    int down_verify_frames_{0};
    int down_verify_balloon_frames_{0};
    int down_verify_other_color_frames_{0};
    int down_verify_motion_frames_{0};
    cv::Mat down_previous_gray_;
    cv::Mat down_previous_shadow_mask_;
    int align_stable_frames_;
    
    // 相机参数
    double img_cx_, img_cy_;
    int img_w_, img_h_;
    
    // yaw_rate PD 控制参数
    double k_p_yaw_rate_;   // 偏航角速度 P 增益 (rad/s per pixel)
    double k_d_yaw_rate_;   // 偏航角速度 D 增益
    double max_yaw_rate_;   // 最大偏航角速度 (rad/s)
    double deadzone_yaw_;   // 偏航死区 (pixel)
    
    // VZ PD 控制参数
    double k_p_vz_;         // 垂直速度 P 增益
    double k_d_vz_;         // 垂直速度 D 增益
    double max_vz_;         // 最大垂直速度 (m/s)
    double deadzone_vz_;    // 垂直死区 (pixel)
    
    // 前进速度参数
    double forward_velocity_;       // 逼近阶段前进速度 (m/s)
    double sprint_velocity_;        // 冲刺速度 (m/s)
    double sprint_time_;            // 冲刺持续时间 (s)
    double active_sprint_time_{0.0}; // 按真实边界余量缩短，而非全盘拒绝
    bool sprint_guard_positive_yaw_{false}; // 靠近 +X 墙时只允许向场内侧修正航向
    bool sprint_front_disappeared_{false}; // 冲刺期间前视目标确实从检测结果中消失
    bool sprint_assumed_after_0p5_{false}; // 策略上允许返航，不代表裁判确认命中
    int post_sprint_reanchors_{0};
    
    // 其他参数
    double data_timeout_;           // 数据超时 (s)
    double approach_area_threshold_;// 触发冲刺的面积占比阈值
    double center_tolerance_;       // 对齐判定区域比例
    double search_yaw_rate_;        // 搜索模式旋转速度 (rad/s)
    double safety_x_min_, safety_x_max_, safety_y_min_, safety_y_max_;
    double safety_z_min_, safety_z_max_, safety_margin_;
    double min_flight_z_;
    double chase_altitude_{0.40};
    double search_altitude_{0.40};
    bool initial_chase_level_complete_{true};
    bool purple_initial_search_active_{false};
    bool purple_inward_staging_{false};
    bool blue_chase_staging_{false};
    geometry_msgs::Point blue_chase_anchor_;
    double blue_chase_yaw_{0.0};
    geometry_msgs::Point blue_tracking_origin_;
    ros::Time blue_tracking_start_;
    geometry_msgs::Point initial_chase_anchor_;
    double filtered_yaw_rate_{0.0};
    double filtered_forward_speed_{0.0};
    double filtered_lateral_speed_{0.0};
    double fresh_pixel_x_{0.0};
    double fresh_pixel_stamp_{0.0};
    double filtered_pixel_rate_{0.0}; // horizontal target motion, px/s
    double previous_diameter_px_{0.0};
    double previous_diameter_stamp_{0.0};
    double filtered_diameter_rate_{0.0}; // apparent expansion, px/s; not metric depth
    geometry_msgs::Point sprint_previous_pos_;
    double sprint_measured_distance_{0.0};
    double sprint_last_segment_check_{0.0};
    double search_timeout_;
    double blue_max_search_time_{35.0};
    double recovery_timeout_{3.5};
    ros::Time recovery_start_time_;
    double recovery_last_detection_ts_{0.0};
    int recovery_confirmations_{0};
    geometry_msgs::Point recovery_anchor_;
    double recovery_yaw_{0.0};
    ros::Time recovery_yaw_update_;
    double recovery_spin_rate_{0.45};
    double recovery_spin_sign_{-1.0}; // 右侧出画面顺时针，左侧出画面逆时针
    double last_seen_center_x_{0.0};
    double last_pixel_dx_{0.0};
    std::string target_color_;
    ros::Subscriber down_image_sub_;
    ros::Subscriber front_image_sub_;
    std::atomic<double> front_frame_time_{0.0};
    ros::Publisher down_hint_pub_;
    std::mutex down_mutex_;
    ros::Time down_seen_time_;
    double down_area_{0.0};
    void downImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void frontImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void holdHorizontalWithClimb(const geometry_msgs::Point& anchor, double target_z);
    ros::Time servo_start_time_;
    
    // 微分计算用
    double prev_err_x_, prev_err_y_;
    ros::Time prev_time_;
    
    // 常量
    static constexpr int ALIGN_STABLE_FRAMES_REQUIRED = 5;

    // 辅助函数
    void updateTargetData();
    bool isTargetLost();
    bool isCentered() const;
    
    // PD 控制器
    double computeYawRate(double err_x, double dt);
    double computeVz(double err_y, double dt);
};

#endif
