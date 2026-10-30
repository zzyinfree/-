/******************************************************************************
 * @file       visual_servo.cpp
 * @brief      Visual Servo 行为树节点 - 三阶段控制（body系 VX+VZ+yaw_rate）
 *
 * @details    三阶段：对齐(ALIGN) -> 靠近(APPROACH) -> 冲刺(SPRINT)
 *             全程使用 publishVelocityWithYawRate（body 坐标系）
 *             控制变量：VX(前进) + VZ(升降) + yaw_rate(偏航角速度)
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-03-16
 ******************************************************************************/
#include "nodes/action/visual_servo.h"
#include "utils/pose_utils.h"
#include <algorithm>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <vector>


VisualServo::VisualServo(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config), 
      current_state_(ALIGN),
      has_ever_received_data_(false),
      max_area_ratio_(0.0) {
    auto nh = mct->getROSHandle();
    nh->param<double>("cx", img_cx_, 320.0);
    nh->param<double>("cy", img_cy_, 240.0);
    nh->param<int>("img_w", img_w_, 640);
    nh->param<int>("img_h", img_h_, 480);
    down_image_sub_ = nh->subscribe("/rflysim/sensor2/img_rgb", 1,
                                    &VisualServo::downImageCallback, this);
    front_image_sub_ = nh->subscribe("/rflysim/sensor1/img_rgb", 1,
                                     &VisualServo::frontImageCallback, this);
    down_hint_pub_ = nh->advertise<std_msgs::String>("/cicc/down_balloon_hint", 3);
}

PortsList VisualServo::providedPorts() {
    return { InputPort<PixelTarget>("pixel_target", "像素目标"),
             
             // yaw_rate PD 控制
             InputPort<double>("k_p_yaw_rate", 0.003, "偏航角速度 P 增益 (rad/s per pixel)"),
             InputPort<double>("k_d_yaw_rate", 0.008, "偏航角速度 D 增益"),
             InputPort<double>("max_yaw_rate", 0.5, "最大偏航角速度 (rad/s)"),
             InputPort<double>("deadzone_yaw", 8.0, "偏航死区 (pixel)"),
             
             // VZ PD 控制
             InputPort<double>("k_p_vz", 0.002, "垂直速度 P 增益"),
             InputPort<double>("k_d_vz", 0.006, "垂直速度 D 增益"),
             InputPort<double>("max_vz", 0.3, "最大垂直速度 (m/s)"),
             InputPort<double>("deadzone_vz", 10.0, "垂直死区 (pixel)"),
             
             // 速度参数
             InputPort<double>("forward_velocity", 0.35, "逼近阶段前进速度 (m/s)"),
             InputPort<double>("sprint_velocity", 1.5, "冲刺速度 (m/s)"),
             InputPort<double>("sprint_time", 1.2, "冲刺持续时间 (s)"),
             
             // 其他
             InputPort<double>("data_timeout", 1.0, "数据超时 (s)"),
             InputPort<double>("approach_area_threshold", 0.08, "触发冲刺的面积占比"),
             InputPort<double>("center_tolerance", 0.15, "对齐判定区域比例"),
             InputPort<double>("min_flight_z", 0.55, "接近气球时的最低允许高度"),
             InputPort<double>("recovery_timeout", 3.5, "近距漏检后悬停复核秒数"),
             InputPort<std::string>("target_color", "", "辅助下视识别的目标颜色"),
             InputPort<double>("search_yaw_rate", 0.3, "搜索模式旋转速度 (rad/s)") };
}

NodeStatus VisualServo::onStart() {
    // 加载参数
    k_p_yaw_rate_ = getInput<double>("k_p_yaw_rate").value();
    k_d_yaw_rate_ = getInput<double>("k_d_yaw_rate").value();
    max_yaw_rate_ = getInput<double>("max_yaw_rate").value();
    deadzone_yaw_ = getInput<double>("deadzone_yaw").value();
    
    k_p_vz_ = getInput<double>("k_p_vz").value();
    k_d_vz_ = getInput<double>("k_d_vz").value();
    max_vz_ = getInput<double>("max_vz").value();
    deadzone_vz_ = getInput<double>("deadzone_vz").value();
    
    forward_velocity_ = getInput<double>("forward_velocity").value();
    sprint_velocity_ = getInput<double>("sprint_velocity").value();
    sprint_time_ = getInput<double>("sprint_time").value();
    
    data_timeout_ = getInput<double>("data_timeout").value();
    approach_area_threshold_ = getInput<double>("approach_area_threshold").value();
    center_tolerance_ = getInput<double>("center_tolerance").value();
    search_yaw_rate_ = getInput<double>("search_yaw_rate").value();
    auto nh = mct->getROSHandle();
    nh->param<double>("safety/x_min", safety_x_min_, -1.0);
    nh->param<double>("safety/x_max", safety_x_max_, 6.0);
    nh->param<double>("safety/y_min", safety_y_min_, -3.0);
    nh->param<double>("safety/y_max", safety_y_max_, 3.0);
    nh->param<double>("safety/z_min", safety_z_min_, -0.1);
    nh->param<double>("safety/z_max", safety_z_max_, 2.5);
    // This margin is only for the close-range visual servo, not navigation.
    // A 10 cm reserve rejected an otherwise viable in-arena blue intercept.
    nh->param<double>("visual_servo/safety_margin", safety_margin_, 0.03);
    min_flight_z_ = getInput<double>("min_flight_z").value();
    nh->param<double>("visual_servo/chase_altitude", chase_altitude_, 0.40);
    recovery_timeout_ = getInput<double>("recovery_timeout").value();
    target_color_ = getInput<std::string>("target_color").value();
    nh->param<double>("visual_servo/chase_altitude_" + target_color_,
                      chase_altitude_, chase_altitude_);
    search_altitude_ = chase_altitude_;
    nh->param<double>("visual_servo/search_altitude_" + target_color_,
                      search_altitude_, search_altitude_);
    initial_chase_level_complete_ = target_color_ != "red" && target_color_ != "purple";
    purple_initial_search_active_ = target_color_ == "purple";
    if (fcu_odom_ptr) initial_chase_anchor_ = fcu_odom_ptr->pose.pose.position;
    if (target_color_ == "purple") {
        // Use the operator-approved point, not the odometry sample at BT
        // hand-off. The latter may still carry several cm of transit error.
        std::string configured_point;
        nh->param<std::string>("balloon_zone_B/approach_pos", configured_point, "");
        std::stringstream point_stream(configured_point);
        geometry_msgs::Point approved;
        char first = 0, second = 0;
        if (point_stream >> approved.x >> first >> approved.y >> second >> approved.z &&
            first == ';' && second == ';' && std::isfinite(approved.x) &&
            std::isfinite(approved.y) && std::isfinite(approved.z)) {
            initial_chase_anchor_ = approved;
        } else {
            ROS_ERROR("[%s]: missing valid operator-approved purple search point", name().c_str());
            return NodeStatus::FAILURE;
        }
    }
    {
        std::lock_guard<std::mutex> lock(down_mutex_);
        down_seen_time_ = ros::Time(0);
        down_area_ = 0.0;
    }
    // Purple search uses the configured yaw sweep rate; previously its
    // search_yaw_rate port was ignored and the sweep stayed at 0.45 rad/s.
    recovery_spin_rate_ = target_color_ == "purple"
        ? std::clamp(search_yaw_rate_, 0.30, 0.90) :
        (target_color_ == "blue" ? 0.60 : 0.45);
    recovery_spin_sign_ = -1.0;
    last_seen_center_x_ = img_cx_;
    last_pixel_dx_ = 0.0;
    nh->param<double>("visual_servo/search_timeout", search_timeout_, 12.0);
    nh->param<double>("visual_servo/search_timeout_" + target_color_,
                      search_timeout_, search_timeout_);
    nh->param<double>("visual_servo/blue_max_search_time", blue_max_search_time_, 35.0);
    servo_start_time_ = ros::Time::now();
    
    // 初始化状态机
    current_state_ = APPROACHING;  // 直接进入边飞边对齐模式（气球是移动目标）
    has_ever_received_data_ = false;
    max_area_ratio_ = 0.0;
    align_stable_frames_ = 0;
    recovery_confirmations_ = 0;
    strike_attempts_ = 0;
      sprint_guard_positive_yaw_ = false;
      sprint_front_disappeared_ = false;
      sprint_assumed_after_0p5_ = false;
      post_sprint_reanchors_ = 0;
    sprint_confirmations_ = 0;
    sprint_last_detection_ts_ = 0.0;
    gated_restrike_not_before_ = ros::Time(0);
    unsafe_sprint_first_time_ = ros::Time(0);
    sprint_recheck_after_ = ros::Time(0);
    unsafe_sprint_rejections_ = 0;
    boundary_hold_count_ = 0;
    boundary_hold_reanchors_ = 0;
    boundary_hold_settled_since_ = ros::Time(0);
    down_verify_frames_ = down_verify_balloon_frames_ = down_verify_motion_frames_ = 0;
    down_verify_other_color_frames_ = 0;
    {
        std::lock_guard<std::mutex> lock(down_mutex_);
        down_previous_gray_.release();
        down_previous_shadow_mask_.release();
    }
    
    // 初始化微分状态
    prev_err_x_ = 0.0;
    prev_err_y_ = 0.0;
    filtered_yaw_rate_ = 0.0;
    filtered_forward_speed_ = 0.0;
    filtered_lateral_speed_ = 0.0;
    fresh_pixel_x_ = 0.0;
    fresh_pixel_stamp_ = 0.0;
    filtered_pixel_rate_ = 0.0;
    previous_diameter_px_ = 0.0;
    previous_diameter_stamp_ = 0.0;
    filtered_diameter_rate_ = 0.0;
    sprint_measured_distance_ = 0.0;
    sprint_last_segment_check_ = 0.0;
    purple_inward_staging_ = false;
    blue_chase_staging_ = false;
    blue_tracking_start_ = ros::Time(0);
    prev_time_ = ros::Time::now();
    
    updateTargetData();
    
    ROS_INFO("[%s]: Visual Servo Started (VX+VZ+yaw_rate)", name().c_str());
    ROS_INFO("[%s]: k_p_yaw_rate=%.4f k_d=%.4f max_rate=%.2f deadzone=%.1f", 
             name().c_str(), k_p_yaw_rate_, k_d_yaw_rate_, max_yaw_rate_, deadzone_yaw_);
    ROS_INFO("[%s]: forward_vel=%.2f sprint_vel=%.2f sprint_time=%.1f area_thresh=%.3f",
             name().c_str(), forward_velocity_, sprint_velocity_, sprint_time_, approach_area_threshold_);
    
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::onRunning() {
    updateTargetData();
    // Independent of the outer watchdog: stop velocity commands before a
    // balloon pursuit drives the vehicle beyond the competition boundary.
    if (!fcu_odom_ptr || !std::isfinite(fcu_odom_ptr->pose.pose.position.x) ||
        !std::isfinite(fcu_odom_ptr->pose.pose.position.y) ||
        !std::isfinite(fcu_odom_ptr->pose.pose.position.z)) {
        sendHoverCommand();
        ROS_ERROR("[%s]: FCU odometry unavailable during visual servo", name().c_str());
        return NodeStatus::FAILURE;
    }
    const auto& p = fcu_odom_ptr->pose.pose.position;
    // During the post-impact climb the position controller is already braking
    // toward an in-bounds anchor. Do not abort at the same 3 cm margin used to
    // decide whether a NEW velocity sprint may start (run061 stopped at 5.97).
    const bool post_impact_position_control = current_state_ == BRAKE_PULSE || current_state_ == BRAKING ||
        current_state_ == POST_STRIKE_VERIFY || current_state_ == RETRY_TRANSIT ||
        current_state_ == RETRY_DESCENT || current_state_ == BOUNDARY_HOLD;
    const double position_margin = post_impact_position_control ? 0.0 : safety_margin_;
    if (p.x < safety_x_min_ + position_margin || p.x > safety_x_max_ - position_margin ||
        p.y < safety_y_min_ + position_margin || p.y > safety_y_max_ - position_margin ||
        p.z < safety_z_min_ + safety_margin_ || p.z > safety_z_max_ - safety_margin_) {
        sendHoverCommand();
        ROS_ERROR("[%s]: servo safety envelope reached at (%.2f,%.2f,%.2f); refusing to sprint",
                  name().c_str(), p.x, p.y, p.z);
        return NodeStatus::FAILURE;
    }
    
    // 计算时间步长
    ros::Time now = ros::Time::now();
    double dt = (now - prev_time_).toSec();
    if (dt < 0.001) dt = 0.001;

    // A missed detector frame while rotating is not evidence of imminent
    // collision. The old 55 cm guard ordered a metre-scale retreat despite
    // the purple balloon still being in the front view. Let target-loss
    // handling hold and scan; retreat only at the actual near-wall limit.
    if (target_color_ == "purple" &&
        current_state_ != BOUNDARY_HOLD &&
        current_state_ != BRAKE_PULSE && current_state_ != BRAKING &&
        current_state_ != POST_STRIKE_VERIFY &&
        (p.x < safety_x_min_ + 0.18 || p.x > safety_x_max_ - 0.18 ||
         p.y < safety_y_min_ + 0.30 || p.y > safety_y_max_ - 0.18)) {
        current_state_ = BOUNDARY_HOLD;
        ++boundary_hold_count_;
        boundary_hold_anchor_ = p;
        // Recover only far enough to regain a safe turning corridor. The old
        // 1.30 m setpoint pulled the aircraft away from a visible balloon.
        boundary_hold_anchor_.x = std::clamp(p.x, safety_x_min_ + 0.30, safety_x_max_ - 0.30);
        // run143 drifted through y=-3.0 after the old -2.70 setpoint.
        // Begin recovery before the wall and aim a full 55 cm inward.
        boundary_hold_anchor_.y = std::clamp(p.y, safety_y_min_ + 0.55, safety_y_max_ - 0.55);
        // A visible balloon is still a chase, not a search: do not jump from
        // 0.40 m to 0.70 m and fly over it while making the lateral correction.
        const bool front_locked = has_ever_received_data_ &&
            (now - last_valid_detection_time_).toSec() < 0.25;
        boundary_hold_anchor_.z = front_locked ? p.z : std::max(p.z, search_altitude_);
        boundary_hold_start_ = boundary_hold_prev_time_ = now;
        boundary_hold_prev_pos_ = p;
        boundary_hold_settled_since_ = ros::Time(0);
        filtered_forward_speed_ = filtered_lateral_speed_ = 0.0;
        ROS_WARN("[%s]: purple near arena edge (%.2f,%.2f); hold and move inward to (%.2f,%.2f,%.2f), attempt %d",
                 name().c_str(), p.x, p.y, boundary_hold_anchor_.x,
                 boundary_hold_anchor_.y, boundary_hold_anchor_.z, boundary_hold_count_);
    }
    if (current_state_ == BOUNDARY_HOLD) {
        if (p.y < boundary_hold_anchor_.y - 0.08) {
            // The position setpoint alone did not pull the aircraft away
            // from the moving car at y=-2.76 in run169. Command a bounded
            // inward world-Y velocity until the retreat point is reachable.
            const double yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
            publishVelocityWithYawRate(0.40 * std::sin(yaw),
                                       0.40 * std::cos(yaw), 0.0, 0.0);
        } else {
            publishPosition(boundary_hold_anchor_,
                PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation));
        }
        const double elapsed = (now - boundary_hold_prev_time_).toSec();
        const double speed = elapsed > 0.03 ?
            std::hypot(p.x-boundary_hold_prev_pos_.x,
                       p.y-boundary_hold_prev_pos_.y)/elapsed : 0.0;
        boundary_hold_prev_pos_ = p;
        boundary_hold_prev_time_ = now;
        ROS_INFO_THROTTLE(1.0,"[%s]: boundary hold actual=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) speed=%.2f",
                          name().c_str(),p.x,p.y,p.z,boundary_hold_anchor_.x,
                          boundary_hold_anchor_.y,boundary_hold_anchor_.z,speed);
        if (std::hypot(p.x-boundary_hold_anchor_.x,p.y-boundary_hold_anchor_.y) < 0.10 &&
            std::abs(p.z-boundary_hold_anchor_.z) < 0.15 && speed < 0.18) {
            if (boundary_hold_settled_since_.isZero()) boundary_hold_settled_since_ = now;
        } else boundary_hold_settled_since_ = ros::Time(0);
        if (!boundary_hold_settled_since_.isZero() &&
            (now-boundary_hold_settled_since_).toSec() > 0.70) {
            if (boundary_hold_count_ >= 2) {
                ROS_WARN("[%s]: purple repeatedly reached arena edge; strike skipped, returning home from stable hover",
                         name().c_str());
                return NodeStatus::SUCCESS;
            }
            current_state_ = RECOVERING;
            recovery_anchor_ = boundary_hold_anchor_;
            recovery_anchor_.z = search_altitude_;
            purple_initial_search_active_ = true;
            recovery_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
            recovery_start_time_ = recovery_yaw_update_ = now;
            recovery_last_detection_ts_ = last_target_.timestamp;
            recovery_confirmations_ = 0;
            ROS_INFO("[%s]: inward boundary hold stable; resume fixed-position purple search",
                     name().c_str());
        }
        if ((now-boundary_hold_start_).toSec() > 8.0) {
            if (boundary_hold_reanchors_ == 0 &&
                p.x > safety_x_min_+0.55 && p.x < safety_x_max_-0.55 &&
                p.y > safety_y_min_+0.55 && p.y < safety_y_max_-0.55) {
                // The moving car can shove the airframe away from the first
                // retreat setpoint. Re-anchor only at a measured in-bounds
                // position, climb, then hand a settled hover to return.
                ++boundary_hold_reanchors_;
                boundary_hold_count_ = 2;
                boundary_hold_anchor_ = p;
                boundary_hold_anchor_.z = std::max(0.80,p.z);
                boundary_hold_start_ = now;
                boundary_hold_settled_since_ = ros::Time(0);
                ROS_WARN("[%s]: purple retreat shifted; re-anchor at (%.2f,%.2f,%.2f) before return",
                         name().c_str(),boundary_hold_anchor_.x,boundary_hold_anchor_.y,
                         boundary_hold_anchor_.z);
            } else {
                ROS_ERROR("[%s]: purple boundary retreat still cannot stabilize; safety landing required",
                          name().c_str());
                return NodeStatus::FAILURE;
            }
        }
        return NodeStatus::RUNNING;
    }

    // Red/purple-zone sequence: settle search altitude before visual
    // search. Do not mix a descending position command with moving-car chase.
    if (!initial_chase_level_complete_) {
        if (std::abs(p.z - search_altitude_) > 0.05) {
            geometry_msgs::Point level = initial_chase_anchor_;
            level.z = search_altitude_;
            publishPosition(level, PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation));
            ROS_INFO_THROTTLE(1.0, "[%s]: %s pre-search vertical settling %.2f -> %.2f m, horizontal hold",
                              name().c_str(), target_color_.c_str(), p.z, search_altitude_);
            return NodeStatus::RUNNING;
        }
        initial_chase_level_complete_ = true;
        current_state_ = RECOVERING;
        recovery_start_time_ = recovery_yaw_update_ = now;
        recovery_anchor_ = target_color_ == "purple" ? initial_chase_anchor_ : p;
        recovery_anchor_.z = search_altitude_;
        recovery_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
        recovery_last_detection_ts_ = last_target_.timestamp;
        recovery_confirmations_ = 0;
        ROS_INFO("[%s]: %s pre-search settled at %.2fm; starting clockwise target search",
                 name().c_str(), target_color_.c_str(), p.z);
        return NodeStatus::RUNNING;
    }
    
    // 目标丢失处理（冲刺阶段不检查）
    if (current_state_ == BRAKE_PULSE || current_state_ == BRAKING) {
        if (!sprint_assumed_after_0p5_ &&
            (now - brake_start_time_).toSec() >= 0.5) {
            sprint_assumed_after_0p5_ = true;
            ROS_INFO("[%s]: 0.5s after sprint: assume strike attempt complete; return allowed after stable 2m climb; judge hit unverified",
                     name().c_str());
        }
        // The front camera can keep seeing the balloon until the impact pulse
        // ends. Count disappearance during the initial climb as well, but only
        // while fresh front images continue to arrive. Absence alone is not a
        // hit verdict; the downward view must corroborate it.
        if (!sprint_front_disappeared_ &&
            (now - sprint_start_time_).toSec() < 3.0 &&
            (now - last_valid_detection_time_).toSec() > 0.35 &&
            now.toSec() - front_frame_time_.load() < 0.25) {
            sprint_front_disappeared_ = true;
            ROS_INFO("[%s]: front balloon disappeared during immediate post-impact climb",
                     name().c_str());
        }
            // Keep altitude fixed for 0.2s after impact while braking forward
            // momentum. Only then start the established 2m climb.
        if (current_state_ == BRAKE_PULSE) {
            // run066 coasted >1m after a 1.5m/s pulse with position-only hold.
            // Counter-command horizontal speed while climbing immediately.
            // Blue run076 made an excellent contact approach, but switching
            // to a fixed XY position after 0.55s generated a large forward
            // rebound. Keep its proven sprint untouched; brake longer and
            // then use zero horizontal velocity through the 2m climb.
            const double elapsed = (now - brake_pulse_start_time_).toSec();
            const double reverse_duration = target_color_ == "blue" ? 0.80 : 0.55;
            const double level_pause = target_color_ == "purple" ? 0.75 : 0.20;
            if (elapsed < reverse_duration + level_pause) {
                const double climb_vz = target_color_ == "red" ? 0.45 : 0.8;
                // Purple has no authoritative contact topic: a disappearing
                // detection is not enough to justify a blind reverse pulse.
                if (target_color_ == "purple") {
                    // Preserve the requested level pause, then counter only
                    // the measured drift toward the -Y wall. This is a slow
                    // inward correction, not a blind backwards pull.
                    const auto& q = fcu_odom_ptr->pose.pose.position;
                    const double yaw = PoseUtils::getYawFromQuat(
                        fcu_odom_ptr->pose.pose.orientation);
                    const double inward_y = elapsed >= level_pause &&
                        q.y < safety_y_min_ + 0.65 ? 0.25 : 0.0;
                    publishVelocityWithYawRate(inward_y*std::sin(yaw),
                        inward_y*std::cos(yaw),
                        elapsed < level_pause ? 0.0 : climb_vz, 0.0);
                } else {
                    publishVelocityWithYawRate(-0.8, 0.0,
                        elapsed < level_pause ? 0.0 : climb_vz, 0.0);
                }
                return NodeStatus::RUNNING;
            }
            verification_anchor_ = fcu_odom_ptr->pose.pose.position;
            const double post_purple_margin = target_color_ == "purple" ? 0.65 : 0.15;
            verification_anchor_.x = std::clamp(verification_anchor_.x,
                safety_x_min_ + post_purple_margin,
                safety_x_max_ - post_purple_margin);
            verification_anchor_.y = std::clamp(verification_anchor_.y,
                safety_y_min_ + post_purple_margin,
                safety_y_max_ - post_purple_margin);
            verification_anchor_.z = 2.0;
            current_state_ = BRAKING;
            brake_window_start_ = now;
            brake_window_distance_ = 0.0;
            brake_prev_pos_ = fcu_odom_ptr->pose.pose.position;
            ROS_INFO("[%s]: active brake complete; 2m hold at (%.2f,%.2f)",
                     name().c_str(), verification_anchor_.x, verification_anchor_.y);
        }
        if (target_color_ == "purple") {
            const auto& q = fcu_odom_ptr->pose.pose.position;
            const double yaw = PoseUtils::getYawFromQuat(
                fcu_odom_ptr->pose.pose.orientation);
            double vx_world = std::clamp(1.0*(verification_anchor_.x-q.x), -0.35, 0.35);
            double vy_world = std::clamp(1.0*(verification_anchor_.y-q.y), -0.35, 0.35);
            const double speed = std::hypot(vx_world, vy_world);
            if (speed > 0.35) { vx_world *= 0.35/speed; vy_world *= 0.35/speed; }
            const double vx_body = vx_world*std::cos(yaw) + vy_world*std::sin(yaw);
            const double vy_body = -vx_world*std::sin(yaw) + vy_world*std::cos(yaw);
            const double vz = std::clamp(1.0*(2.0-q.z), -0.35, 0.80);
            publishVelocityWithYawRate(vx_body, vy_body, vz, 0.0);
        } else if (target_color_ == "blue") {
            holdHorizontalWithClimb(verification_anchor_, 2.0);
        } else {
            publishPosition(verification_anchor_, verification_yaw_);
        }
        // The cloned simulator publishes zero odom twist even while position
        // changes rapidly. Use measured travelled distance, not twist.
        const auto& q = fcu_odom_ptr->pose.pose.position;
        const double lateral_drift = std::hypot(q.x-verification_anchor_.x,
                                                q.y-verification_anchor_.y);
        if (lateral_drift > 1.30) {
            if (++post_sprint_reanchors_ > 2) {
                ROS_ERROR("[%s]: repeated post-sprint drift %.2fm; flight safety cannot stabilize",
                          name().c_str(), lateral_drift);
                return NodeStatus::FAILURE;
            }
            verification_anchor_ = q;
            verification_anchor_.z = 2.0;
            brake_window_start_ = now;
            brake_window_distance_ = 0.0;
            brake_prev_pos_ = q;
            if (q.z > 2.20)
                publishVelocityWithYawRate(0.0, 0.0, -0.5, 0.0);
            else
                publishPosition(verification_anchor_, verification_yaw_);
            ROS_WARN("[%s]: post-sprint drift %.2fm; reanchoring in-bounds 2m hold before return (%d/2)",
                     name().c_str(), lateral_drift, post_sprint_reanchors_);
            return NodeStatus::RUNNING;
        }
        brake_window_distance_ += std::sqrt(std::pow(q.x - brake_prev_pos_.x, 2) +
                                             std::pow(q.y - brake_prev_pos_.y, 2) +
                                             std::pow(q.z - brake_prev_pos_.z, 2));
        brake_prev_pos_ = q;
        if ((now - brake_window_start_).toSec() >= 1.0) {
            if (std::abs(q.z - 2.0) < 0.12 && lateral_drift < 0.40 &&
                brake_window_distance_ < 0.16 &&
                (now - brake_start_time_).toSec() >= 1.5) {
                // Never hand a still-moving vehicle to the return planner.
                if (!sprint_assumed_after_0p5_) return NodeStatus::RUNNING;
                if (target_color_ == "blue" || target_color_ == "purple") {
                    // The previous blue branch returned here without ever
                    // entering POST_STRIKE_VERIFY. Reset its evidence window
                    // only after the 2 m hover is stable.
                    {
                        std::lock_guard<std::mutex> lock(down_mutex_);
                        down_verify_frames_ = down_verify_balloon_frames_ = 0;
                        down_verify_other_color_frames_ = down_verify_motion_frames_ = 0;
                        down_previous_gray_.release();
                        down_previous_shadow_mask_.release();
                    }
                    verification_start_time_ = now;
                    current_state_ = POST_STRIKE_VERIFY;
                    ROS_INFO("[%s]: stable 2m hover; starting %s downward post-strike verification",
                             name().c_str(), target_color_.c_str());
                    return NodeStatus::RUNNING;
                }
                ROS_INFO("[%s]: sprint complete, 2m climb stabilized (%.2fm motion/1s); returning without collision verification; hit unverified",
                         name().c_str(), brake_window_distance_);
                return NodeStatus::SUCCESS;
            }
            brake_window_distance_ = 0.0;
            brake_window_start_ = now;
        }
        if ((now - brake_start_time_).toSec() > 12.0) {
            ROS_ERROR("[%s]: post-sprint 2m climb did not settle; refusing return", name().c_str());
            return NodeStatus::FAILURE;
        }
        return NodeStatus::RUNNING;
    }
    if (current_state_ == POST_STRIKE_VERIFY) {
        if (target_color_ == "purple") {
            // No backwards translation after impact: hold the measured coast
            // endpoint and examine the downward image there at 2m.
            publishPosition(verification_anchor_, verification_yaw_);
            int frames, balloon, other_color, motion;
            {
                std::lock_guard<std::mutex> lock(down_mutex_);
                frames = down_verify_frames_;
                balloon = down_verify_balloon_frames_;
                other_color = down_verify_other_color_frames_;
                motion = down_verify_motion_frames_;
            }
            const double elapsed = (now-verification_start_time_).toSec();
            if (elapsed < 1.5 || (frames < 15 && elapsed < 3.0)) return NodeStatus::RUNNING;
            ROS_INFO_THROTTLE(1.0, "[%s]: purple down verify frames=%d target=%d other=%d motion=%d attempt=%d front_disappeared=%s",
                              name().c_str(), frames, balloon, other_color, motion,
                              strike_attempts_, sprint_front_disappeared_ ? "yes" : "no");
            if (balloon >= 3) {
                if (strike_attempts_ >= 3) {
                    ROS_WARN("[%s]: purple still present after three attacks; returning without hit claim", name().c_str());
                    return NodeStatus::SUCCESS;
                }
                current_state_ = RETRY_TRANSIT;
                phase_start_time_ = now;
                ROS_WARN("[%s]: purple still visible downward; reacquiring with front camera for attempt %d", name().c_str(), strike_attempts_+1);
                return NodeStatus::RUNNING;
            }
            if (frames >= 20 || elapsed >= 3.0) {
                if (motion >= 5 && other_color == 0 && sprint_front_disappeared_) {
                    ROS_INFO("[%s]: down view has moving object but no purple after front disappearance; returning; judge hit unverified", name().c_str());
                    return NodeStatus::SUCCESS;
                }
                // An empty camera view is not proof that the moving target
                // car is empty. In run142 only 4 motion frames were observed,
                // yet this branch returned home after the first sprint.
                if (strike_attempts_ >= 3) {
                    ROS_WARN("[%s]: purple down view inconclusive after %d attacks; returning without hit claim",
                             name().c_str(), strike_attempts_);
                    return NodeStatus::SUCCESS;
                }
                current_state_ = RETRY_TRANSIT;
                phase_start_time_ = now;
                ROS_WARN("[%s]: purple down view inconclusive (frames=%d, motion=%d); descend at fixed XY and reacquire front target for attempt %d",
                         name().c_str(), frames, motion, strike_attempts_ + 1);
                return NodeStatus::RUNNING;
            }
            return NodeStatus::RUNNING;
        }
        if (target_color_ == "blue") {
            holdHorizontalWithClimb(verification_anchor_, 2.0);
            int frames, balloon, other_color, motion;
            {
                std::lock_guard<std::mutex> lock(down_mutex_);
                frames = down_verify_frames_;
                balloon = down_verify_balloon_frames_;
                other_color = down_verify_other_color_frames_;
                motion = down_verify_motion_frames_;
            }
            const double elapsed = (now - verification_start_time_).toSec();
            if (elapsed < 2.0 || (frames < 12 && elapsed < 3.5))
                return NodeStatus::RUNNING;
            ROS_INFO_THROTTLE(1.0, "[%s]: blue post-strike: front_disappeared=%s down_frames=%d blue_balloon=%d other_balloon=%d motion=%d attempt=%d",
                              name().c_str(), sprint_front_disappeared_ ? "yes" : "no",
                              frames, balloon, other_color, motion, strike_attempts_);
            if (frames >= 20 && balloon == 0 && other_color == 0 && sprint_front_disappeared_) {
                ROS_INFO("[%s]: blue disappeared from front and is absent in %d downward frames; returning, judge hit unverified",
                         name().c_str(), frames);
                return NodeStatus::SUCCESS;
            }
            if (balloon >= 3 || other_color >= 3) {
                if (strike_attempts_ >= 3) {
                    ROS_WARN("[%s]: blue remains or verification inconclusive after three attempts; return home without claiming a hit",
                             name().c_str());
                    return NodeStatus::SUCCESS;
                }
                current_state_ = RETRY_TRANSIT;
                phase_start_time_ = now;
                ROS_WARN("[%s]: blue target still visible/inconclusive after strike; repositioning for front-camera reacquisition",
                         name().c_str());
                return NodeStatus::RUNNING;
            }
            if (elapsed >= 4.5) {
                ROS_WARN("[%s]: blue not confirmed in downward verification window; returning instead of entering a blind search, judge hit unverified",
                         name().c_str());
                return NodeStatus::SUCCESS;
            }
            return NodeStatus::RUNNING;
        }
        publishPosition(verification_anchor_, verification_yaw_);
        if (std::hypot(p.x - verification_anchor_.x, p.y - verification_anchor_.y) > 0.40 ||
            std::abs(p.z - 2.0) > 0.18) {
            ROS_ERROR("[%s]: cannot verify from unstable 2m hover", name().c_str());
            return NodeStatus::FAILURE;
        }
        int frames, balloon, other_color, motion;
        {
            std::lock_guard<std::mutex> lock(down_mutex_);
            frames = down_verify_frames_;
            balloon = down_verify_balloon_frames_;
            other_color = down_verify_other_color_frames_;
            motion = down_verify_motion_frames_;
        }
        if ((now - verification_start_time_).toSec() < 3.0 || frames < 8)
            return NodeStatus::RUNNING;
        ROS_INFO_THROTTLE(1.0, "[%s]: down verify %d frames, target-balloon=%d, other-balloon=%d, moving-object=%d, attempt=%d, front-disappeared=%s",
                          name().c_str(), frames, balloon, other_color, motion, strike_attempts_,
                          sprint_front_disappeared_ ? "yes" : "no");
        if (balloon >= 3) {
            if (strike_attempts_ >= 3) {
                ROS_ERROR("[%s]: balloon remains after three attempts; do not report a hit", name().c_str());
                return NodeStatus::FAILURE;
            }
            current_state_ = RETRY_TRANSIT;
            phase_start_time_ = now;
            ROS_WARN("[%s]: balloon still visible downward; repositioning at 2m for another visual chase", name().c_str());
            return NodeStatus::RUNNING;
        }
        // run053 showed a RED balloon/car in the downward view after a BLUE
        // sprint. Motion alone would falsely certify that the blue target car
        // is now empty. Any other-colour balloon invalidates that inference.
        if (other_color >= 3) {
            // run058: the downward view was dominated by another car for 8 s.
            // Do not wait there or call a hit. Reposition while airborne and
            // reacquire the blue car with the forward camera.
            if (strike_attempts_ >= 3) {
                ROS_ERROR("[%s]: three strikes, but downward view shows a different car; hit unverified",
                          name().c_str());
                return NodeStatus::FAILURE;
            }
            current_state_ = RETRY_TRANSIT;
            phase_start_time_ = now;
            ROS_WARN("[%s]: other-colour car below after strike; leaving hover and searching target car again",
                     name().c_str());
            return NodeStatus::RUNNING;
        }
        if (motion >= 8 && frames >= 25 && sprint_front_disappeared_) {
            ROS_INFO("[%s]: front target left view during impact; downward camera sees moving car without target balloon in %d frames; return permitted, judge hit unverified",
                     name().c_str(), frames);
            return NodeStatus::SUCCESS;
        }
        if ((now - verification_start_time_).toSec() > 4.5) {
            if (strike_attempts_ >= 3) {
                ROS_ERROR("[%s]: downward view remains inconclusive after three attempts", name().c_str());
                return NodeStatus::FAILURE;
            }
            current_state_ = RETRY_TRANSIT;
            phase_start_time_ = now;
            ROS_WARN("[%s]: downward verification inconclusive; leaving hover to reacquire target",
                     name().c_str());
            return NodeStatus::RUNNING;
        }
        return NodeStatus::RUNNING;
    }
    if (current_state_ == RETRY_TRANSIT || current_state_ == RETRY_DESCENT) {
        // Purple restrike starts beneath the post-impact hover, not by
        // pulling the aircraft back over its old sprint origin.
        geometry_msgs::Point goal = target_color_ == "purple" ?
            verification_anchor_ : sprint_origin_;
        goal.z = current_state_ == RETRY_TRANSIT ? 2.0 :
                 (target_color_ == "purple" ? search_altitude_ :
                  std::max(min_flight_z_ + 0.20, sprint_origin_.z));
        if (goal.x < safety_x_min_ + safety_margin_ || goal.x > safety_x_max_ - safety_margin_ ||
            goal.y < safety_y_min_ + safety_margin_ || goal.y > safety_y_max_ - safety_margin_ ||
            (now - phase_start_time_).toSec() > 12.0) {
            sendHoverCommand();
            ROS_ERROR("[%s]: retry staging unsafe/timed out; no blind pursuit", name().c_str());
            return NodeStatus::FAILURE;
        }
        publishPosition(goal, PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation));
        if (std::hypot(p.x-goal.x, p.y-goal.y) < 0.18 && std::abs(p.z-goal.z) < 0.15) {
            if (current_state_ == RETRY_TRANSIT) {
                current_state_ = RETRY_DESCENT;
                phase_start_time_ = now;
            } else {
                current_state_ = RECOVERING;
                recovery_anchor_ = p;
                recovery_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
                recovery_yaw_update_ = recovery_start_time_ = now;
                recovery_confirmations_ = 0;
                recovery_last_detection_ts_ = last_target_.timestamp;
                max_area_ratio_ = 0.0;
              ROS_INFO("[%s]: retry staging reached; acquire two fresh front detections", name().c_str());
            }
        }
        return NodeStatus::RUNNING;
    }
    if (current_state_ == RECOVERING) {
        const double drift = std::hypot(p.x - recovery_anchor_.x, p.y - recovery_anchor_.y);
        if (purple_initial_search_active_ &&
            (drift > 0.15 || std::abs(p.z - recovery_anchor_.z) > 0.08)) {
            // Correct XY at the approved point before yawing or committing to
            // pursuit. Rotating while the transit inertia remains is what
            // pushed the old search outside the requested 15 cm radius.
            if ((now - recovery_start_time_).toSec() > 12.0) {
                if (boundary_hold_count_ > 0) {
                    ROS_WARN("[%s]: purple recovery search could not settle at inward anchor; returning home", name().c_str());
                    return NodeStatus::SUCCESS;
                }
                ROS_ERROR("[%s]: cannot hold approved purple point within 15 cm; aborting search", name().c_str());
                return NodeStatus::FAILURE;
            }
            recovery_confirmations_ = 0;
            publishPosition(recovery_anchor_, recovery_yaw_);
            ROS_WARN_THROTTLE(1.0, "[%s]: purple fixed-point correction: horizontal error %.3f m", name().c_str(), drift);
            return NodeStatus::RUNNING;
        }
        // run068: after a short forward camera dropout, measured inertia
        // carried the airframe 0.36m while the hold setpoint was already
        // active. Keep braking/searching instead of aborting at that instant.
        if (drift > 0.80) {
            ROS_ERROR("[%s]: recovery hover drift %.2fm; refusing blind pursuit", name().c_str(), drift);
            return NodeStatus::FAILURE;
        }
        if (has_ever_received_data_ && last_target_.timestamp > recovery_last_detection_ts_ + 0.05 &&
            (now - last_valid_detection_time_).toSec() < 0.25) {
            recovery_last_detection_ts_ = last_target_.timestamp;
            ++recovery_confirmations_;
        }
        if (recovery_confirmations_ >= 2 &&
            (now - last_valid_detection_time_).toSec() < 0.25) {
            purple_initial_search_active_ = false;
            if (target_color_ == "blue") {
                blue_tracking_origin_ = p;
                blue_tracking_start_ = now;
            }
            if (target_color_ == "blue" && p.z > chase_altitude_ + 0.06) {
                blue_chase_anchor_ = recovery_anchor_;
                blue_chase_anchor_.z = chase_altitude_;
                blue_chase_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
                blue_chase_staging_ = true;
                ROS_INFO("[%s]: blue target locked; hold search XY (%.2f,%.2f) while descending, yaw=%.2f",
                         name().c_str(), blue_chase_anchor_.x, blue_chase_anchor_.y, blue_chase_yaw_);
            }
            // Two fresh front-camera frames suffice. Do not perform another
            // stationary ALIGN turn after the purple target is visible.
            current_state_ = target_color_ == "purple" ? APPROACHING : ALIGN;
            align_stable_frames_ = 0;
            prev_err_x_ = prev_err_y_ = 0.0;
            ROS_INFO("[%s]: same-color balloon reacquired on %d fresh frames; resuming visual approach",
                     name().c_str(), recovery_confirmations_);
            return NodeStatus::RUNNING;
        }
        const double search_window = max_area_ratio_ > approach_area_threshold_
            ? recovery_timeout_ : search_timeout_;
        // Limit each unconfirmed scan and, for blue, the cumulative search
        // across intermittent single-frame sightings. Leave enough time for
        // front-camera return before the mission supervisor's deadline.
        const double unconfirmed_seconds = (now - recovery_start_time_).toSec();
        const double total_search_seconds = (now - servo_start_time_).toSec();
        if (unconfirmed_seconds > search_window ||
            (target_color_ == "blue" && total_search_seconds > blue_max_search_time_)) {
            sendHoverCommand();
            ROS_WARN("[%s]: no confirmed %s balloon (scan %.1fs, total %.1fs); strike skipped, navigating home; judge hit unverified",
                     name().c_str(), target_color_.c_str(), unconfirmed_seconds, total_search_seconds);
            return NodeStatus::SUCCESS;
        }
        const double scan_dt = std::clamp((now - recovery_yaw_update_).toSec(), 0.0, 0.1);
        // Freeze the search turn on the first fresh purple sighting while
        // waiting for the confirming second frame. Resume turning only if
        // that sighting expires, avoiding a full extra revolution past target.
        const bool pending_lock = (target_color_ == "purple" || target_color_ == "blue") &&
            recovery_confirmations_ > 0 &&
            (now - last_valid_detection_time_).toSec() < 0.25;
        if (!pending_lock)
            recovery_yaw_ += recovery_spin_sign_ * recovery_spin_rate_ * scan_dt;
        recovery_yaw_update_ = now;
        publishPosition(recovery_anchor_, recovery_yaw_); // x/y/z fixed; yaw only
        bool down_recent = false;
        double down_area = 0.0;
        {
            std::lock_guard<std::mutex> lock(down_mutex_);
            down_recent = !down_seen_time_.isZero() && (now - down_seen_time_).toSec() < 0.7;
            down_area = down_area_;
        }
        ROS_WARN_THROTTLE(1.0, "[%s]: %s scan at fixed z=%.2f; front confirmations=%d down_%s=%s area=%.0f",
                          name().c_str(), recovery_spin_sign_ < 0.0 ? "clockwise/right" : "counterclockwise/left",
                          recovery_anchor_.z, recovery_confirmations_,
                          target_color_.c_str(), down_recent ? "VISIBLE" : "none", down_area);
        return NodeStatus::RUNNING;
    }
    if (current_state_ != SPRINTING && isTargetLost()) {
        sendHoverCommand();
        sprint_confirmations_ = 0;
        current_state_ = RECOVERING;
        recovery_start_time_ = now;
        recovery_yaw_update_ = now;
        recovery_anchor_ = p;
        recovery_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
        if (last_seen_center_x_ < img_w_ * 0.25)
            recovery_spin_sign_ = 1.0;
        else if (last_seen_center_x_ > img_w_ * 0.75)
            recovery_spin_sign_ = -1.0;
        else if (std::abs(last_pixel_dx_) > 3.0)
            recovery_spin_sign_ = last_pixel_dx_ < 0.0 ? 1.0 : -1.0;
        else
            recovery_spin_sign_ = last_seen_center_x_ < img_cx_ ? 1.0 : -1.0;
        recovery_last_detection_ts_ = last_target_.timestamp;
        recovery_confirmations_ = 0;
        ROS_WARN("[%s]: target lost at ratio %.3f; scanning toward %s image exit",
                 name().c_str(), max_area_ratio_,
                 recovery_spin_sign_ < 0.0 ? "right" : "left");
        return NodeStatus::RUNNING;
    }
    if (current_state_ != SPRINTING && has_ever_received_data_ &&
        (now - last_valid_detection_time_).toSec() > 0.25) {
        // One or two missed detector frames are not grounds for landing, but
        // they are also not permission to keep closing on stale pixels.
        sendHoverCommand();
        sprint_confirmations_ = 0;
        return NodeStatus::RUNNING;
    }
    if (current_state_ == APPROACHING || current_state_ == ALIGN) {
        if (target_color_ == "blue" && blue_chase_staging_) {
            publishPosition(blue_chase_anchor_, blue_chase_yaw_);
            filtered_forward_speed_ = filtered_lateral_speed_ = 0.0;
            const double xy_error = std::hypot(p.x - blue_chase_anchor_.x,
                                                p.y - blue_chase_anchor_.y);
            if (xy_error < 0.12 && std::abs(p.z - chase_altitude_) < 0.06) {
                blue_chase_staging_ = false;
                blue_tracking_origin_ = p;
                blue_tracking_start_ = now;
                ROS_INFO("[%s]: blue fixed-XY descent complete; start camera-guided approach", name().c_str());
            } else {
                ROS_INFO_THROTTLE(1.0, "[%s]: blue fixed-XY descent: XY drift %.2fm, z %.2f->%.2f",
                                  name().c_str(), xy_error, p.z, chase_altitude_);
            }
            return NodeStatus::RUNNING;
        }
        if (p.z < min_flight_z_ - 0.01) {
            geometry_msgs::Point lift = p;
            lift.z = chase_altitude_;
            publishPosition(lift, PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation));
            ROS_WARN_THROTTLE(1.0, "[%s]: actual z %.2f below %.2f floor; hold x/y and climb before closing",
                              name().c_str(), p.z, min_flight_z_);
            return NodeStatus::RUNNING;
        }
        // Purple's approved search point is only 0.38 m from the -Y wall.
        // Once the target is confirmed, shift a short distance INWARD at the
        // search height before descending. The previous fixed-XY descent
        // drifted to y=-2.70, tripped BOUNDARY_HOLD and never pursued.
        if (target_color_ == "purple" && p.y < safety_y_min_ + 0.54 &&
            p.z > chase_altitude_ + 0.06)
            purple_inward_staging_ = true;
        if (target_color_ == "purple" && purple_inward_staging_) {
            geometry_msgs::Point inward = p;
            inward.y = safety_y_min_ + 0.58;
            inward.z = search_altitude_;
            publishPosition(inward, PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation));
            filtered_forward_speed_ = filtered_lateral_speed_ = 0.0;
            if (p.y >= safety_y_min_ + 0.52 &&
                std::abs(p.z - search_altitude_) < 0.10)
                purple_inward_staging_ = false;
            ROS_INFO_THROTTLE(1.0, "[%s]: purple front target confirmed; shifting inward at search height before chase descent (y %.2f -> %.2f)",
                              name().c_str(), p.y, inward.y);
            return NodeStatus::RUNNING;
        }
        // Complete the height change before horizontal pursuit.
        // All colour-specific visual chases now use the same 40cm datum.
        if (std::abs(p.z - chase_altitude_) > 0.06) {
            geometry_msgs::Point level = p;
            level.z = chase_altitude_;
            publishPosition(level, PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation));
            filtered_forward_speed_ = 0.0;
            ROS_INFO_THROTTLE(1.0, "[%s]: settling chase altitude %.2f -> %.2f m before pursuit",
                              name().c_str(), p.z, chase_altitude_);
            return NodeStatus::RUNNING;
        }
    }
    
    // 计算基本量
    double img_area = static_cast<double>(img_w_) * img_h_;
    double current_ratio = last_target_.area() / img_area;
    double err_x = last_target_.center_x - img_cx_;  // 正=目标在右侧
    double err_y = last_target_.center_y - img_cy_;  // 正=目标在下方
    if (last_target_.timestamp > fresh_pixel_stamp_ + 0.02) {
        const double frame_dt = last_target_.timestamp - fresh_pixel_stamp_;
        if (fresh_pixel_stamp_ > 0.0 && frame_dt > 0.025 && frame_dt < 0.35) {
            const double measured_rate = std::clamp(
                (err_x - fresh_pixel_x_) / frame_dt, -450.0, 450.0);
            filtered_pixel_rate_ = 0.55 * filtered_pixel_rate_ +
                                   0.45 * measured_rate;
        } else {
            filtered_pixel_rate_ = 0.0;
        }
        fresh_pixel_x_ = err_x;
        fresh_pixel_stamp_ = last_target_.timestamp;
    }
    // Apparent expansion provides a scale-free time-to-contact cue. It is
    // NOT a calibrated metric range and must never certify a collision.
    if (target_color_ == "purple" &&
        last_target_.timestamp > previous_diameter_stamp_ + 0.02) {
        const double diameter = std::sqrt(std::max(0.0, last_target_.area()));
        const double sample_dt = last_target_.timestamp - previous_diameter_stamp_;
        if (previous_diameter_stamp_ > 0.0 && sample_dt > 0.025 && sample_dt < 0.35) {
            const double rate = std::clamp((diameter - previous_diameter_px_) / sample_dt,
                                           -200.0, 200.0);
            filtered_diameter_rate_ = 0.6 * filtered_diameter_rate_ + 0.4 * rate;
        } else {
            filtered_diameter_rate_ = 0.0;
        }
        previous_diameter_px_ = diameter;
        previous_diameter_stamp_ = last_target_.timestamp;
    }
    
    // 更新最大面积记录
    if (current_ratio > max_area_ratio_) {
        max_area_ratio_ = current_ratio;
    }
    // Count distinct detector frames, not 100 Hz controller ticks. A single
              // transient blue blob must never authorize an impact sprint.
      if (last_target_.timestamp > sprint_last_detection_ts_ + 0.02) {
          if (sprint_last_detection_ts_ > 0.0 &&
              last_target_.timestamp - sprint_last_detection_ts_ > 0.30)
              sprint_confirmations_ = 0;
        sprint_last_detection_ts_ = last_target_.timestamp;
        const double sprint_vertical_band = target_color_ == "purple" ?
            img_h_ * 0.25 : img_h_ * center_tolerance_ * 1.2;
        const bool stable_sighting = current_ratio >= 0.005 &&
            std::abs(err_x) < img_w_ * center_tolerance_ * 1.2 &&
            std::abs(err_y) < sprint_vertical_band;
        sprint_confirmations_ = stable_sighting ?
            std::min(sprint_confirmations_ + 1, 8) : 0;
    }
    
    // PD 控制计算
    double yaw_rate = computeYawRate(err_x, dt);
    // Pixel vertical error changes sharply as the car passes beneath us.
    // Do not chase that error with altitude; hold the requested 40cm instead.
    double vz = std::clamp(1.2 * (chase_altitude_ - p.z), -0.22, 0.22);
    const double yaw_alpha = 1.0 - std::exp(-dt / 0.25);
    filtered_yaw_rate_ += yaw_alpha * (std::clamp(yaw_rate, -0.40, 0.40) - filtered_yaw_rate_);
    yaw_rate = filtered_yaw_rate_;
    // Do not keep descending to center a low image target: the run018
    // controller drove the aircraft almost to the ground without a hit.
    if (vz < 0.0 && p.z + vz * 0.30 <= min_flight_z_) {
        vz = 0.0;
        ROS_WARN_THROTTLE(1.0, "[%s]: minimum servo altitude %.2f m reached; descent inhibited",
                          name().c_str(), min_flight_z_);
    }
    
    // 更新微分状态
    prev_time_ = now;
    
    // =========== 三阶段状态机 ===========
    switch (current_state_) {
        case ALIGN: {
            // 阶段1：对齐 — 原地旋转 + 升降，使目标居中
            publishVelocityWithYawRate(0, 0, vz, yaw_rate);
            
            if (isCentered()) {
                align_stable_frames_++;
                if (align_stable_frames_ >= ALIGN_STABLE_FRAMES_REQUIRED) {
                    current_state_ = APPROACHING;
                    align_stable_frames_ = 0;
                    ROS_INFO("[%s]: Aligned! err:(%.0f,%.0f) -> APPROACHING", 
                             name().c_str(), err_x, err_y);
                }
            } else {
                align_stable_frames_ = 0;
            }
            
            ROS_INFO_THROTTLE(1.0, "[%s]: ALIGN - err:(%.0f,%.0f) yaw_rate:%.3f vz:%.3f stable:%d", 
                             name().c_str(), err_x, err_y, yaw_rate, vz, align_stable_frames_);
            break;
        }
        
        case APPROACHING: {
            // 阶段2：靠近 — VX 前进 + yaw_rate + VZ，全程持续修正
            // 根据对齐程度调节前进速度（越偏越慢）
            const double center_band = target_color_ == "red" ? 0.24 : 0.30;
            double align_ratio = std::max(0.0, 1.0 - std::abs(err_x) / (img_w_ * center_band));
            double vx = target_color_ == "red" ? forward_velocity_ : forward_velocity_ * align_ratio;
            // Coordinate pitch (forward) and aileron (body lateral) rather
            // than asking yaw alone to remove the pixel error. The sign is
            // consistent with the existing body-to-world transform below.
            const bool coordinated = target_color_ == "red" || target_color_ == "purple";
            const double apparent_diameter = std::max(45.0,
                std::sqrt(std::max(0.0,last_target_.area())));
            const bool fresh_red_motion = target_color_ == "red" &&
                now.toSec()-fresh_pixel_stamp_ < 0.25;
            // Match the car's lateral optical flow before it reaches the
            // camera. Error-only control reacted too late at 1-3% image area.
            const double red_flow_vy = fresh_red_motion ?
                -0.22 * filtered_pixel_rate_ / apparent_diameter : 0.0;
            const double desired_vy = target_color_ == "red" ?
                std::clamp(red_flow_vy - err_x/img_w_, -0.45, 0.45) :
                (coordinated ? std::clamp(-0.7*err_x/img_w_, -0.12, 0.12) : 0.0);
            const double lateral_alpha = 1.0 - std::exp(-dt /
                (target_color_ == "red" ? 0.12 : 0.25));
            filtered_lateral_speed_ += lateral_alpha * (desired_vy - filtered_lateral_speed_);
            double vy = filtered_lateral_speed_;
            if (target_color_ == "purple") {
                const double heading_now = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
                const double lateral_world_x = -vy * std::sin(heading_now);
                const double lateral_world_y =  vy * std::cos(heading_now);
                if ((p.x < safety_x_min_ + 0.55 && lateral_world_x < 0.0) ||
                    (p.x > safety_x_max_ - 0.55 && lateral_world_x > 0.0) ||
                    (p.y < safety_y_min_ + 0.55 && lateral_world_y < 0.0) ||
                    (p.y > safety_y_max_ - 0.55 && lateral_world_y > 0.0)) {
                    vy = filtered_lateral_speed_ = 0.0;
                    ROS_WARN_THROTTLE(1.0, "[%s]: purple lateral correction toward wall inhibited", name().c_str());
                }
            }

            // run032-034: the sphere slid below the front camera while the
            // vehicle continued closing. At close range, first finish a
            // bounded descent/yaw alignment instead of driving into the car
            // or wall. Keep the altitude floor and global fence active.
            if (target_color_ == "blue" && current_ratio >= 0.012 &&
                std::abs(err_y) >= img_h_ * center_tolerance_) {
                vx = 0.0;
                ROS_WARN_THROTTLE(1.0,
                    "[%s]: close balloon vertical error %.0fpx; hold forward for alignment (z=%.2f)",
                    name().c_str(), err_y, p.z);
            }

            // A large apparent target can still be far off the flight axis.
            // Stop closing range and re-align before allowing a sprint.
            if (target_color_ == "blue" && current_ratio >= approach_area_threshold_ && !isCentered()) {
                publishVelocityWithYawRate(0, 0, vz, yaw_rate);
                current_state_ = ALIGN;
                align_stable_frames_ = 0;
                ROS_WARN_THROTTLE(1.0, "[%s]: Close target off-axis (err_x=%.0f, err_y=%.0f); re-aligning before sprint",
                                  name().c_str(), err_x, err_y);
                break;
            }
            
            const double purple_diameter = std::sqrt(std::max(0.0, last_target_.area()));
            const double optical_ttc = filtered_diameter_rate_ > 8.0 ?
                purple_diameter / filtered_diameter_rate_ : 1e6;
            const bool purple_closing_window = target_color_ == "purple" &&
                current_ratio >= 0.006 && optical_ttc > 0.20 && optical_ttc < 1.10 &&
                std::abs(err_x + 0.12 * filtered_pixel_rate_) < img_w_ * 0.12;
            const bool strike_window = current_ratio >= approach_area_threshold_ || purple_closing_window;
            // The first blue balloon sighting can occur beside the red car.
            // Do not sprint immediately after descent: first follow the BLUE
            // target forward for a measurable distance and tighten centering.
            const double blue_yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
            const double blue_forward_progress = target_color_ == "blue" ?
                (p.x - blue_tracking_origin_.x) * std::cos(blue_yaw) +
                (p.y - blue_tracking_origin_.y) * std::sin(blue_yaw) : 0.0;
            const bool blue_tracking_ready = target_color_ != "blue" ||
                (!blue_tracking_start_.isZero() &&
                 (now - blue_tracking_start_).toSec() >= 0.60 &&
                 blue_forward_progress >= 0.15 &&
                 std::abs(err_x + 0.10 * filtered_pixel_rate_) < img_w_ * 0.09);
            if (strike_window && sprint_confirmations_ < 2) {
                vx = 0.0;
                ROS_WARN_THROTTLE(0.5, "[%s]: close balloon seen on only %d fresh frames; hold range until 2-frame confirmation",
                                  name().c_str(), sprint_confirmations_);
            }
            const double speed_alpha = 1.0 - std::exp(-dt / 0.30);
            filtered_forward_speed_ += speed_alpha * (vx - filtered_forward_speed_);
            vx = filtered_forward_speed_;
            const double heading = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
            const double lookahead = 0.75;
            // The approved purple search point itself lies 28 cm from -Y.
            // Reject motion *toward* the wall, not a parallel/inward approach
            // that starts at this point; the arena hard fence remains active.
            // In run138 a 0.18 m forward reserve allowed approach momentum
            // to carry purple pursuit from y=-2.57 through the -3.0 wall.
            // Stop only the outward translation sooner; yaw stays active.
            bool unsafe_approach = false;
            if (target_color_ == "purple") {
                // Project the *commanded* body velocity over a reaction/braking
                // horizon. The old fixed 0.75 m projection blocked even
                // parallel flight at the approved search point near the -Y wall.
                const double horizon = 1.3;
                const double reserve = 0.18;
                const double world_vx = vx * std::cos(heading) - vy * std::sin(heading);
                const double world_vy = vx * std::sin(heading) + vy * std::cos(heading);
                const double future_x = p.x + horizon * world_vx;
                const double future_y = p.y + horizon * world_vy;
                unsafe_approach =
                    (world_vx < -0.005 && future_x < safety_x_min_ + reserve) ||
                    (world_vx >  0.005 && future_x > safety_x_max_ - reserve) ||
                    (world_vy < -0.005 && future_y < safety_y_min_ + reserve) ||
                    (world_vy >  0.005 && future_y > safety_y_max_ - reserve);
            } else {
                const double reserve = 0.55;
                const double future_x = p.x + lookahead * std::cos(heading) - vy * std::sin(heading);
                const double future_y = p.y + lookahead * std::sin(heading) + vy * std::cos(heading);
                unsafe_approach = future_x < safety_x_min_ + reserve || future_x > safety_x_max_ - reserve ||
                                  future_y < safety_y_min_ + reserve || future_y > safety_y_max_ - reserve;
            }
            if (unsafe_approach) {
                vx = filtered_forward_speed_ = 0.0;
                vy = filtered_lateral_speed_ = 0.0;
                ROS_WARN_THROTTLE(1.0, "[%s]: pursuit boundary lookahead; holding forward and lateral speed until target turns inward",
                                  name().c_str());
            }
            publishVelocityWithYawRate(vx, vy, vz, yaw_rate);
            
            // 对齐偏差过大 → 回到 ALIGN
            // A moving car can cross the image centre repeatedly. Stay in
            // APPROACHING and smoothly reduce vx instead of toggling modes.
            
            // 面积达阈值 → 触发冲刺
            const bool contact_gated_color = target_color_ == "red" || target_color_ == "purple";
            if (strike_window && sprint_confirmations_ >= 2 && blue_tracking_ready &&
                now >= sprint_recheck_after_ &&
                (!contact_gated_color || now >= gated_restrike_not_before_)) {
                // Check the actual forward corridor, not a full-radius circle:
                // run043 faced yaw=-1.16 toward the blue balloon and was
                // incorrectly rejected by the old x+1.2m all-directions test.
                const double yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
                // Near the +X or -Y wall, a positive yaw correction can
                // steer a purple pulse outward. Permit only straight or
                // clockwise steering and forecast that one-sided corridor.
                sprint_guard_positive_yaw_ =
                    (p.x > safety_x_max_ - 0.70 && std::cos(yaw) > 0.0 && yaw < 0.0) ||
                    (target_color_ == "purple" && p.y < safety_y_min_ + 1.25 &&
                     std::sin(yaw) < -0.15);
                bool crosses_fence = false;
                active_sprint_time_ = sprint_time_;
                double travel = 0.0;
                while (active_sprint_time_ >= 0.40) {
                    // run051 measured ~0.30m post-command coast. Keep that
                    // physical margin while allowing shorter bounded pulses.
                    // run097 crossed y=-3.0 after a nominally in-bounds
                    // purple pulse. Its observed coast exceeded the older
                    // 0.75m allowance; reserve extra stopping distance.
                    travel = sprint_velocity_ * active_sprint_time_ +
                        (target_color_ == "purple" ? 0.95 : 0.75);
                    // Limit yaw authority during the impact pulse itself;
                    // the old max-rate fan rejected a nearly straight,
                    // pixel-centered intercept in run052.
                    const double fan = (sprint_guard_positive_yaw_ ? 0.20 :
                        (target_color_ == "red" ? 0.50 : 0.30)) * active_sprint_time_ + 0.04;
                    crosses_fence = false;
                    const std::vector<double> offsets = sprint_guard_positive_yaw_
                        ? std::vector<double>{-fan, 0.0}
                        : std::vector<double>{-fan, 0.0, fan};
                    for (const double offset : offsets) {
                        const double x_end = p.x + travel * std::cos(yaw + offset);
                        const double y_end = p.y + travel * std::sin(yaw + offset);
                        const double pulse_margin = target_color_ == "purple" ?
                            0.18 : safety_margin_;
                        if (x_end < safety_x_min_ + pulse_margin ||
                            x_end > safety_x_max_ - pulse_margin ||
                            y_end < safety_y_min_ + pulse_margin ||
                            y_end > safety_y_max_ - pulse_margin) crosses_fence = true;
                    }
                    if (!crosses_fence) break;
                    active_sprint_time_ -= 0.025;
                }
                if (crosses_fence || active_sprint_time_ < 0.40) {
                    sendHoverCommand();
                    if (unsafe_sprint_first_time_.isZero()) unsafe_sprint_first_time_ = now;
                    ++unsafe_sprint_rejections_;
                    if ((now - unsafe_sprint_first_time_).toSec() > 20.0) {
                        ROS_WARN("[%s]: no safe sprint corridor after %d checks; strike skipped, returning home",
                                 name().c_str(), unsafe_sprint_rejections_);
                        return NodeStatus::SUCCESS;
                    }
                    current_state_ = RECOVERING;
                    recovery_anchor_ = p;
                    recovery_anchor_.z = chase_altitude_;
                    recovery_yaw_ = yaw;
                    recovery_yaw_update_ = recovery_start_time_ = now;
                    recovery_confirmations_ = 0;
                    recovery_last_detection_ts_ = last_target_.timestamp;
                    sprint_recheck_after_ = now + ros::Duration(1.0);
                    filtered_forward_speed_ = 0.0;
                    ROS_WARN("[%s]: predicted sprint could cross arena fence; "
                             "pos=(%.2f,%.2f), yaw=%.2f, travel=%.2f. No sprint; hold and reacquire (%d).",
                             name().c_str(), p.x, p.y, yaw, travel, unsafe_sprint_rejections_);
                    return NodeStatus::RUNNING;
                }
                unsafe_sprint_first_time_ = ros::Time(0);
                unsafe_sprint_rejections_ = 0;
                // Check the moving purple target after a short first pulse.
                // The full 1.5 s burst let it slide 211 px left and carried
                // the UAV to the arena edge before the next camera decision.
                if (target_color_ == "purple")
                    active_sprint_time_ = std::min(active_sprint_time_, 0.65);
                current_state_ = SPRINTING;
                sprint_origin_ = p;
                sprint_previous_pos_ = p;
                sprint_measured_distance_ = 0.0;
                sprint_last_segment_check_ = 0.0;
                if (target_color_ == "purple") filtered_lateral_speed_ = 0.0;
                ++strike_attempts_;
                sprint_front_disappeared_ = false;
                sprint_assumed_after_0p5_ = false;
                sprint_start_time_ = ros::Time::now();
                ROS_INFO("[%s]: Area %.1f%% >= %.1f%% -> SPRINT! err=(%.0f,%.0f) pos=(%.2f,%.2f,%.2f) (sprint_vel=%.2f, time=%.1fs)",
                         name().c_str(), current_ratio*100, approach_area_threshold_*100,
                          err_x, err_y, p.x, p.y, p.z, sprint_velocity_, active_sprint_time_);
                if (target_color_ == "purple")
                    ROS_INFO("[%s]: purple intercept trigger=%s apparent_diameter=%.1fpx expansion=%.1fpx/s optical_ttc=%.2fs; metric_range=unavailable",
                             name().c_str(), purple_closing_window ? "expansion" : "area",
                             purple_diameter, filtered_diameter_rate_, optical_ttc);
            }
            if (target_color_ == "blue" && strike_window && !blue_tracking_ready)
                ROS_INFO_THROTTLE(0.7, "[%s]: blue sprint deferred; track forward first (progress=%.2fm, err=%.0fpx)",
                                  name().c_str(), blue_forward_progress, err_x);
            
            ROS_INFO_THROTTLE(0.5, "[%s]: APPROACH - vx:%.2f(%.0f%%) vy:%.2f px_rate:%.0f yaw_rate:%.3f vz:%.3f err:(%.0f,%.0f) area:%.1f%%", 
                             name().c_str(), vx, 100.0*vx/std::max(forward_velocity_, 0.01),
                             vy, filtered_pixel_rate_, yaw_rate, vz, err_x, err_y, current_ratio*100);
            break;
        }
        
        case SPRINTING: {
            // 阶段3：冲刺 — 全速前进，保持 yaw_rate 和 VZ 修正
            double sprint_dt = (ros::Time::now() - sprint_start_time_).toSec();
            if (target_color_ == "purple") {
                const double step = std::hypot(p.x - sprint_previous_pos_.x,
                                               p.y - sprint_previous_pos_.y);
                if (step < 0.25) sprint_measured_distance_ += step;
                sprint_previous_pos_ = p;
                // A short first pulse is extended only while the current
                // front image still contains a centered target and the
                // measured velocity/coast fit the wall corridor. Recheck
                // every 0.15 s instead of assuming command_speed * time.
                if (sprint_dt + 0.08 >= active_sprint_time_ &&
                    sprint_dt - sprint_last_segment_check_ >= 0.12 &&
                    active_sprint_time_ < sprint_time_ - 0.01) {
                    sprint_last_segment_check_ = sprint_dt;
                    const bool fresh_target = !isTargetLost() &&
                        (now - last_valid_detection_time_).toSec() < 0.25;
                    const double predicted_error = err_x + 0.12 * filtered_pixel_rate_;
                    const double yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
                    const double measured_speed = std::hypot(
                        fcu_odom_ptr->twist.twist.linear.x,
                        fcu_odom_ptr->twist.twist.linear.y);
                    const double proposed = std::min(sprint_time_, active_sprint_time_ + 0.15);
                    const double remaining = std::max(0.0, proposed - sprint_dt);
                    const double reach = std::max(sprint_velocity_, measured_speed) * remaining +
                                         std::max(0.45, 0.42 * measured_speed);
                    const double fan = 0.10;
                    bool clear = true;
                    for (const double angle : {-fan, 0.0, fan}) {
                        const double x = p.x + reach * std::cos(yaw + angle);
                        const double y = p.y + reach * std::sin(yaw + angle);
                        clear &= x >= safety_x_min_ + 0.18 && x <= safety_x_max_ - 0.18 &&
                                 y >= safety_y_min_ + 0.18 && y <= safety_y_max_ - 0.18;
                    }
                    if (fresh_target && std::abs(predicted_error) < img_w_ * 0.15 && clear) {
                        active_sprint_time_ = proposed;
                        ROS_INFO("[%s]: purple segmented sprint extended to %.2fs, measured_distance=%.2fm speed=%.2fm/s target_err=%.0fpx boundary_clear=yes",
                                 name().c_str(), proposed, sprint_measured_distance_, measured_speed, predicted_error);
                    }
                }
            }
            // Keep the balloon near image center during the pulse; allow a
            // slightly stronger live front-camera correction without changing
            // the tested forward pulse speed or arena-fence guard.
            const double sprint_yaw = sprint_guard_positive_yaw_
                ? std::clamp(yaw_rate, -0.20, 0.0)
                : std::clamp(yaw_rate,
                    target_color_ == "red" ? -0.50 : -0.30,
                    target_color_ == "red" ? 0.50 : 0.30);
            // A moving car can leave the image centre during a fixed-yaw
            // pulse (red trial: +17 -> -198 px in 0.8 s). Coordinate aileron
            // with yaw/pitch, using only fresh front-camera motion. Blue's
            // previously successful impact controller remains unchanged.
            double sprint_vy = 0.0;
            if (target_color_ == "blue" && !isTargetLost() &&
                (now - last_valid_detection_time_).toSec() < 0.25) {
                // Blue run163 repeatedly saw the target slide left while
                // sprint_vy stayed zero. Add a modest body-left correction;
                // use only fresh pixels and retain the arena fence guard.
                const double predicted_x = err_x + 0.10 * filtered_pixel_rate_;
                sprint_vy = std::clamp(0.04 - 0.50 * predicted_x / img_w_,
                                       -0.04, 0.14);
                const double yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
                const double horizon = std::max(0.0, active_sprint_time_ - sprint_dt) + 0.35;
                const double side_x = p.x - sprint_vy * std::sin(yaw) * horizon;
                const double side_y = p.y + sprint_vy * std::cos(yaw) * horizon;
                if (side_x < safety_x_min_ + 0.25 || side_x > safety_x_max_ - 0.25 ||
                    side_y < safety_y_min_ + 0.25 || side_y > safety_y_max_ - 0.25)
                    sprint_vy = 0.0;
            }
            if (target_color_ == "purple" && !isTargetLost() &&
                (now - last_valid_detection_time_).toSec() < 0.25) {
                // Run144 crossed the target from +25 to -25 px while the
                // purple pulse commanded zero aileron. Add a small LEFT bias
                // (body +Y), with fresh-frame centering and optical flow.
                const double predicted_x = err_x + 0.10 * filtered_pixel_rate_;
                sprint_vy = std::clamp(0.08 - 0.50 * predicted_x / img_w_,
                                       -0.08, 0.16);
                const double yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
                const double horizon = std::max(0.0, active_sprint_time_ - sprint_dt) + 0.35;
                const double side_x = p.x - sprint_vy * std::sin(yaw) * horizon;
                const double side_y = p.y + sprint_vy * std::cos(yaw) * horizon;
                if (side_x < safety_x_min_ + 0.18 || side_x > safety_x_max_ - 0.18 ||
                    side_y < safety_y_min_ + 0.18 || side_y > safety_y_max_ - 0.18)
                    sprint_vy = 0.0;
            }
            if (target_color_ == "red") {
                const double frame_age = now.toSec() - fresh_pixel_stamp_;
                if (frame_age < 0.25) {
                    const double predicted_x = err_x + 0.15 * filtered_pixel_rate_;
                    // Optical flow alone grows rapidly as range closes. Use
                    // apparent balloon diameter to convert px/s into an
                    // approximate lateral m/s of the moving car, then add a
                    // small centering correction. No extra forward impulse.
                    const double apparent_diameter = std::max(45.0,
                        std::sqrt(std::max(0.0,last_target_.area())));
                    const double lateral_feedforward =
                        -0.28 * filtered_pixel_rate_ / apparent_diameter;
                    const double limit = target_color_ == "red" ? 0.45 : 0.28;
                    const double desired = std::clamp(
                        lateral_feedforward - 0.6 * predicted_x / img_w_,
                        -limit, limit);
                    const double alpha = 1.0 - std::exp(-dt / 0.10);
                    filtered_lateral_speed_ += alpha * (desired-filtered_lateral_speed_);
                    sprint_vy = filtered_lateral_speed_;
                } else {
                    filtered_lateral_speed_ *= std::exp(-dt / 0.10);
                }
                const double yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
                const double lateral_horizon =
                    std::max(0.0, active_sprint_time_-sprint_dt)+0.35;
                const double lateral_x = p.x - sprint_vy * std::sin(yaw) * lateral_horizon;
                const double lateral_y = p.y + sprint_vy * std::cos(yaw) * lateral_horizon;
                if (lateral_x < safety_x_min_ + 0.25 || lateral_x > safety_x_max_ - 0.25 ||
                    lateral_y < safety_y_min_ + 0.25 || lateral_y > safety_y_max_ - 0.25)
                    sprint_vy = 0.0;
            }
            const bool fresh_front = now.toSec() - front_frame_time_.load() < 0.25;
            const double target_age = (now - last_valid_detection_time_).toSec();
            // A timed red/purple pulse is not evidence of contact. Only sustained
            // disappearance in fresh front frames can start post-strike braking.
            const bool contact_gated_color = target_color_ == "red";
            const bool contact_proxy = contact_gated_color && fresh_front &&
                sprint_dt >= 0.45 && target_age > 0.35;
            const bool purple_still_visible = target_color_ == "purple" &&
                fresh_front && !isTargetLost() && target_age < 0.25;
            // The balloon leaving the side of the image is not a hit. In the
            // October 30 trace it exited at -231 px while still visible below.
            const bool purple_off_axis_exit = target_color_ == "purple" &&
                fresh_front && target_age < 0.70 &&
                std::abs(err_x) > img_w_ * 0.25;
            if (sprint_dt >= active_sprint_time_ &&
                ((contact_gated_color && !contact_proxy) || purple_still_visible ||
                 purple_off_axis_exit)) {
                publishVelocityWithYawRate(0.0, 0.0, vz, 0.0);
                current_state_ = APPROACHING;
                filtered_forward_speed_ = 0.0;
                sprint_confirmations_ = 0;
                gated_restrike_not_before_ = now + ros::Duration(0.45);
                if (purple_still_visible || purple_off_axis_exit)
                    ROS_WARN("[%s]: purple sprint ended with target %s; "
                             "hold level and resume camera-guided pursuit (target age %.2fs, err_x=%.0fpx)",
                             name().c_str(), purple_still_visible ? "visible" : "exiting image side",
                             target_age, err_x);
                else
                    ROS_WARN("[%s]: %s sprint duration ended but no sustained front disappearance; "
                             "no reverse command, resume camera-guided pursuit (target age %.2fs)",
                             name().c_str(), target_color_.c_str(), target_age);
                return NodeStatus::RUNNING;
            }
            
            // 冲刺结束后位置控制需要时间消除惯性；不可立即把高速度
            // 状态交给返航规划器，否则旧速度会把飞机推过规划轨迹。
            if (sprint_dt >= active_sprint_time_ || contact_proxy) {
                verification_anchor_ = p;
                const double coast_yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
                verification_anchor_.z = 2.0;
                verification_yaw_ = coast_yaw;
                publishVelocityWithYawRate(target_color_ == "purple" ? 0.0 : -0.8,
                                           0.0, 0.0, 0.0);
                brake_start_time_ = ros::Time::now();
                brake_pulse_start_time_ = brake_start_time_;
                brake_window_start_ = brake_start_time_;
                brake_window_distance_ = 0.0;
                brake_prev_pos_ = p;
                current_state_ = BRAKE_PULSE;
                ROS_INFO("[%s]: Sprint stopped (%.1fs, close-contact proxy=%s, front disappearance flag=%s); brake level for 0.2s, then climb to 2m; judge hit unverified",
                         name().c_str(), sprint_dt, contact_proxy ? "yes" : "no",
                         sprint_front_disappeared_ ? "yes" : "no");
                return NodeStatus::RUNNING;
            }
            
            // 冲刺中仍然持续修正 yaw_rate 和 vz（如果还看得到目标）
            if (!isTargetLost()) {
                publishVelocityWithYawRate(sprint_velocity_, sprint_vy, vz, sprint_yaw);
            } else {
                // 目标丢失，保持直线冲刺
                publishVelocityWithYawRate(sprint_velocity_, 0, 0, 0);
            }
            
            ROS_INFO_THROTTLE(0.2, "[%s]: SPRINT front-center err_x=%.0fpx vx:%.2f vy:%.2f px_rate:%.0f yaw_rate:%.3f vz:%.3f (%.1f/%.1fs)",
                             name().c_str(), err_x, sprint_velocity_, sprint_vy, filtered_pixel_rate_, sprint_yaw, vz,
                             sprint_dt, active_sprint_time_);
            break;
        }
    }
    
    return NodeStatus::RUNNING;
}

void VisualServo::holdHorizontalWithClimb(const geometry_msgs::Point& anchor,
                                          double target_z) {
    const auto& q = fcu_odom_ptr->pose.pose.position;
    const double yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    double vx_world = std::clamp(1.5 * (anchor.x - q.x), -0.8, 0.8);
    double vy_world = std::clamp(1.5 * (anchor.y - q.y), -0.8, 0.8);
    const double norm = std::hypot(vx_world, vy_world);
    if (norm > 0.8) {
        vx_world *= 0.8 / norm;
        vy_world *= 0.8 / norm;
    }
    const double vx_body = vx_world * std::cos(yaw) + vy_world * std::sin(yaw);
    const double vy_body = -vx_world * std::sin(yaw) + vy_world * std::cos(yaw);
    const double vz = std::clamp(1.0 * (target_z - q.z), -0.35, 0.8);
    publishVelocityWithYawRate(vx_body, vy_body, vz, 0.0);
}

void VisualServo::frontImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    (void)msg;
    front_frame_time_.store(ros::Time::now().toSec());
}

void VisualServo::downImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    if (target_color_ != "blue" && target_color_ != "red" && target_color_ != "purple") return;
    try {
        const cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat hsv, mask;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        if (target_color_ == "blue") {
            cv::inRange(hsv, cv::Scalar(95, 100, 45), cv::Scalar(132, 255, 255), mask);
        } else if (target_color_ == "purple") {
            cv::inRange(hsv, cv::Scalar(133, 80, 45), cv::Scalar(172, 255, 255), mask);
        } else {
            cv::Mat low, high;
            cv::inRange(hsv, cv::Scalar(0, 95, 45), cv::Scalar(12, 255, 255), low);
            cv::inRange(hsv, cv::Scalar(170, 95, 45), cv::Scalar(179, 255, 255), high);
            cv::bitwise_or(low, high, mask);
        }
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        // A differently coloured balloon means the downward view may be of
        // another car. It must not certify that our target car is empty.
        cv::Mat other_mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
        if (target_color_ != "red") {
            cv::Mat low, high, red;
            cv::inRange(hsv, cv::Scalar(0, 95, 45), cv::Scalar(12, 255, 255), low);
            cv::inRange(hsv, cv::Scalar(170, 95, 45), cv::Scalar(179, 255, 255), high);
            cv::bitwise_or(low, high, red);
            cv::bitwise_or(other_mask, red, other_mask);
        }
        if (target_color_ != "purple") {
            cv::Mat purple;
            cv::inRange(hsv, cv::Scalar(133, 80, 45), cv::Scalar(172, 255, 255), purple);
            cv::bitwise_or(other_mask, purple, other_mask);
        }
        if (target_color_ != "blue") {
            cv::Mat blue;
            cv::inRange(hsv, cv::Scalar(95, 100, 45), cv::Scalar(132, 255, 255), blue);
            cv::bitwise_or(other_mask, blue, other_mask);
        }
        cv::morphologyEx(other_mask, other_mask, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        std::vector<std::vector<cv::Point>> other_contours;
        cv::findContours(other_mask, other_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        bool other_balloon = false;
        for (const auto& contour : other_contours) {
            const double area = cv::contourArea(contour);
            const cv::Rect box = cv::boundingRect(contour);
            const double perimeter = cv::arcLength(contour, true);
            const double circularity = perimeter > 1.0 ? 4.0*M_PI*area/(perimeter*perimeter) : 0.0;
            cv::Mat value;
            cv::extractChannel(hsv(box), value, 2);
            const double mean_value = cv::mean(value, other_mask(box))[0];
            if (area >= 350.0 && area <= 8000.0 && circularity >= 0.50 &&
                mean_value >= 130.0 && box.width >= 25 && box.height >= 25) {
                other_balloon = true;
                break;
            }
        }
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        double best_area = 0.0;
        cv::Rect best_box;
        bool round_balloon = false;
        for (const auto& contour : contours) {
            const double area = cv::contourArea(contour);
            const cv::Rect box = cv::boundingRect(contour);
            const double perimeter = cv::arcLength(contour, true);
            const double circularity = perimeter > 1.0 ? 4.0*M_PI*area/(perimeter*perimeter) : 0.0;
            cv::Mat value;
            cv::extractChannel(hsv(box), value, 2);
            const double mean_value = cv::mean(value, mask(box))[0];
            // run051: dark drone shadow produced 18/30 false blue "balloon"
            // frames at 2m. The actual blue balloon was bright (V≈181),
            // whereas the shadow blobs were V≈50–110.
            const double min_brightness = target_color_ == "blue" ? 145.0 : 130.0;
            const bool plausible = area >= 350.0 && area <= 8000.0 &&
                circularity >= 0.50 && mean_value >= min_brightness &&
                box.width >= 25 && box.height >= 25 &&
                static_cast<double>(box.width)/box.height > 0.55 &&
                static_cast<double>(box.width)/box.height < 1.7;
            round_balloon = round_balloon || plausible;
            if (area > best_area) { best_area = area; best_box = box; }
        }
        cv::Mat gray, blurred;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(7,7), 0);
        cv::Mat shadow_mask;
        cv::inRange(hsv, cv::Scalar(90, 80, 0), cv::Scalar(135, 255, 140), shadow_mask);
        bool localized_motion = false;
        {
            std::lock_guard<std::mutex> lock(down_mutex_);
            if (!down_previous_gray_.empty() && down_previous_gray_.size() == blurred.size()) {
                cv::Mat diff, moving;
                cv::absdiff(blurred, down_previous_gray_, diff);
                cv::threshold(diff, moving, 38, 255, cv::THRESH_BINARY);
                cv::Mat shadow_union;
                if (!down_previous_shadow_mask_.empty())
                    cv::bitwise_or(shadow_mask, down_previous_shadow_mask_, shadow_union);
                else shadow_union = shadow_mask;
                moving.setTo(0, shadow_union);
                cv::morphologyEx(moving, moving, cv::MORPH_OPEN,
                    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
                std::vector<std::vector<cv::Point>> move_contours;
                cv::findContours(moving, move_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                for (const auto& c : move_contours) {
                    const double area = cv::contourArea(c);
                    const cv::Rect box = cv::boundingRect(c);
                    if (area > 250 && area < 12000 && box.width > 20 && box.height > 15 &&
                        box.width < bgr.cols/2 && box.height < bgr.rows/2) {
                        localized_motion = true;
                        break;
                    }
                }
            }
            down_previous_gray_ = blurred;
            down_previous_shadow_mask_ = shadow_mask;
            ++down_verify_frames_;
            if (round_balloon) ++down_verify_balloon_frames_;
            if (other_balloon) ++down_verify_other_color_frames_;
            if (localized_motion) ++down_verify_motion_frames_;
            if (best_area >= 180.0) {
                down_seen_time_ = ros::Time::now();
                down_area_ = best_area;
            }
        }
        if (best_area < 180.0) return;
        // Auxiliary colour evidence only. Never use this hint alone as a hit
        // verdict or as permission for blind forward movement.
        std::ostringstream ss;
        ss << "{\"color\":\"" << target_color_ << "\",\"source\":\"down_hsv_aux\","
           << "\"bbox_px\":[" << best_box.x << ',' << best_box.y << ','
           << best_box.x + best_box.width << ',' << best_box.y + best_box.height << "],"
           << "\"area_px\":" << best_area << ",\"round_candidate\":"
           << (round_balloon ? "true" : "false") << ",\"localized_motion\":"
           << (localized_motion ? "true" : "false") << ",\"other_color_balloon\":"
           << (other_balloon ? "true" : "false") << ",\"stamp\":" << ros::Time::now().toSec() << '}';
        std_msgs::String hint;
        hint.data = ss.str();
        down_hint_pub_.publish(hint);
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(2.0, "[%s]: down camera conversion failed: %s", name().c_str(), e.what());
    }
}

// =====================================================================
// PD 控制器
// =====================================================================

double VisualServo::computeYawRate(double err_x, double dt) {
    // 目标在右侧(err_x>0) → 需要右转 → yaw_rate < 0 (NED 逆时针为正)
    double err = err_x;
    
    // 死区
    double err_after_dz = 0.0;
    if (std::abs(err) > deadzone_yaw_) {
        err_after_dz = err - std::copysign(deadzone_yaw_, err);
    }
    
    // PD
    double d_err = (dt > 1e-6) ? (err_after_dz - prev_err_x_) / dt : 0.0;
    double yaw_rate = -(k_p_yaw_rate_ * err_after_dz + k_d_yaw_rate_ * d_err);
    
    // 限幅
    yaw_rate = std::clamp(yaw_rate, -max_yaw_rate_, max_yaw_rate_);
    
    prev_err_x_ = err_after_dz;
    return yaw_rate;
}

double VisualServo::computeVz(double err_y, double dt) {
    // 目标在下方(err_y>0) → 需要下降 → vz < 0 (看飞控坐标系)
    double err = err_y;
    
    // 死区
    double err_after_dz = 0.0;
    if (std::abs(err) > deadzone_vz_) {
        err_after_dz = err - std::copysign(deadzone_vz_, err);
    }
    
    // PD
    double d_err = (dt > 1e-6) ? (err_after_dz - prev_err_y_) / dt : 0.0;
    double vz = -(k_p_vz_ * err_after_dz + k_d_vz_ * d_err);
    
    // 限幅
    vz = std::clamp(vz, -max_vz_, max_vz_);
    
    prev_err_y_ = err_after_dz;
    return vz;
}

// =====================================================================
// 辅助函数
// =====================================================================

void VisualServo::updateTargetData() {
    auto target_res = getInput<PixelTarget>("pixel_target");
    if (!target_res) return;

    PixelTarget current_target = target_res.value();
    if (current_target.is_detected) {
        const bool fresh_frame = current_target.timestamp > last_target_.timestamp + 0.001;
        if (fresh_frame) {
            if (has_ever_received_data_)
                last_pixel_dx_ = current_target.center_x - last_seen_center_x_;
            last_seen_center_x_ = current_target.center_x;
        }
        last_target_ = current_target;
        if (fresh_frame) last_valid_detection_time_ = ros::Time::now();
        has_ever_received_data_ = true;
        
        double img_area = static_cast<double>(img_w_) * img_h_;
        double ratio = last_target_.area() / img_area;
        if (ratio > max_area_ratio_) {
            max_area_ratio_ = ratio;
        }
    } else {
        // An actual detector miss breaks the consecutive-frame strike vote.
        // Keep the last pose only for a bounded stationary recovery scan.
        sprint_confirmations_ = 0;
        recovery_confirmations_ = 0;
        if (current_state_ == SPRINTING &&
            (ros::Time::now() - sprint_start_time_).toSec() > 0.10)
            sprint_front_disappeared_ = true;
    }
}

bool VisualServo::isTargetLost() {
    if (!has_ever_received_data_) return true;
    return (ros::Time::now() - last_valid_detection_time_).toSec() > data_timeout_;
}

bool VisualServo::isCentered() const {
    double err_x = last_target_.center_x - img_cx_;
    double err_y = last_target_.center_y - img_cy_;
    // The tolerance is a fraction of the image, not of the detected balloon.
    // A small/moving balloon otherwise requires ~2-4 px centering and the
    // controller can remain in ALIGN indefinitely while the car drives past.
    double tol_w = img_w_ * center_tolerance_;
    double tol_h = img_h_ * center_tolerance_;
    // Purple's balloon often sits below the camera centre at 40 cm flight
    // height. Yaw cannot remove that vertical pixel error; requiring both
    // axes left the aircraft rotating in place with the balloon visible.
    return std::abs(err_x) < tol_w &&
        (target_color_ == "purple" || std::abs(err_y) < tol_h);
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<VisualServo>("VisualServo");
}
