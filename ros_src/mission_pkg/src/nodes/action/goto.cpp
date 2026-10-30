/******************************************************************************
 * @file       goto.cpp
 * @brief      GoTo 行为树节点 - 飞往指定位置
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#include "nodes/action/goto.h"
#include "utils/pose_utils.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <algorithm>
#include <queue>
#include <vector>
#include <limits>

namespace {
double wrapYaw(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// Fallback for a return leg that cannot fit a full 1 m local sidestep.
// Build a small 2-D slice from EGO's inflated map plus the live front cloud.
// EGO inflates static obstacles by the requested 0.10 m; live depth points
// receive a separate 0.23 m physical footprint. The grid stays 0.1 m inboard.
bool planReturnWaypoint(const sensor_msgs::PointCloud2& map,
                        const sensor_msgs::PointCloud2& depth,
                        const geometry_msgs::Point& start,
                        const geometry_msgs::Point& goal,
                        bool prefer_right,
                        geometry_msgs::Point& waypoint) {
    constexpr double res=0.20, xmin=-0.90, ymin=-2.90;
    constexpr int nx=35, ny=30, n=nx*ny;
    std::vector<unsigned char> occupied(n,0);
    auto valid=[](int x,int y){return x>=0&&x<nx&&y>=0&&y<ny;};
    auto index=[](int x,int y){return y*nx+x;};
    auto gridX=[](double x){return static_cast<int>(std::lround((x-xmin)/res));};
    auto gridY=[](double y){return static_cast<int>(std::lround((y-ymin)/res));};
    auto addCloud=[&](const sensor_msgs::PointCloud2& cloud,double radius) {
        const int cells=static_cast<int>(std::ceil(radius/res));
        try {
            sensor_msgs::PointCloud2ConstIterator<float> px(cloud,"x"),py(cloud,"y"),pz(cloud,"z");
            for (;px!=px.end();++px,++py,++pz) {
                if (!std::isfinite(*px)||!std::isfinite(*py)||!std::isfinite(*pz)||
                    std::abs(*pz-start.z)>0.38) continue;
                const int cx=gridX(*px),cy=gridY(*py);
                if (!valid(cx,cy)) continue;
                for(int dy=-cells;dy<=cells;++dy) for(int dx=-cells;dx<=cells;++dx) {
                    if (std::hypot(dx*res,dy*res)>radius+0.01) continue;
                    if(valid(cx+dx,cy+dy)) occupied[index(cx+dx,cy+dy)]=1;
                }
            }
        } catch(const std::runtime_error&) { return false; }
        return true;
    };
    if(!addCloud(map,prefer_right ? 0.20 : 0.0)||!addCloud(depth,0.23)) return false;
    const int sx=gridX(start.x),sy=gridY(start.y),gx=gridX(goal.x),gy=gridY(goal.y);
    if(!valid(sx,sy)||!valid(gx,gy)) return false;
    const int source=index(sx,sy),target=index(gx,gy);
    // The aircraft may already be at the edge of an inflated cell. Permit
    // departure from its measured cell, but never traverse another occupied cell.
    occupied[source]=0;
    if(occupied[target]) return false;
    using Item=std::pair<double,int>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;
    std::vector<double> cost(n,std::numeric_limits<double>::infinity());
    std::vector<int> parent(n,-1);
    cost[source]=0;open.push({0,source});
    const int ox[8]={1,-1,0,0,1,1,-1,-1};
    const int oy[8]={0,0,1,-1,1,-1,1,-1};
    while(!open.empty()) {
        const auto [priority,u]=open.top();open.pop();
        if(priority>cost[u]+1e-6) continue;
        if(u==target) break;
        const int x=u%nx,y=u/nx;
        for(int k=0;k<8;++k) {
            const int xx=x+ox[k],yy=y+oy[k];
            if(!valid(xx,yy)||occupied[index(xx,yy)]) continue;
            if(k>=4 && (occupied[index(x+ox[k],y)] ||
                        occupied[index(x,y+oy[k])])) continue;
            const int v=index(xx,yy);
            const double home_dx=goal.x-start.x, home_dy=goal.y-start.y;
            const double home_len=std::max(0.01,std::hypot(home_dx,home_dy));
            const double right=(xx-x)*res*home_dy/home_len -
                               (yy-y)*res*home_dx/home_len;
            const double next=cost[u]+(k<4?1.0:1.41421356237)+
                (prefer_right ? 5.0*std::max(0.0,-right) : 0.0);
            if(next+1e-6<cost[v]) {cost[v]=next;parent[v]=u;open.push({next,v});}
        }
    }
    if(source!=target && parent[target]<0) return false;
    std::vector<int> route;
    for(int v=target;v!=source;v=parent[v]) {
        if(v<0||route.size()>static_cast<size_t>(n)) return false;
        route.push_back(v);
    }
    std::reverse(route.begin(),route.end());
    if(route.empty()) return false;
    auto lineFree=[&](int v) {
        const double ex=xmin+(v%nx)*res,ey=ymin+(v/nx)*res;
        const double length=std::hypot(ex-start.x,ey-start.y);
        for(double d=0.10;d<=length;d+=0.10) {
            const double t=d/std::max(length,0.01);
            const int cx=gridX(start.x+t*(ex-start.x));
            const int cy=gridY(start.y+t*(ey-start.y));
            if(!valid(cx,cy)||occupied[index(cx,cy)]) return false;
        }
        return true;
    };
    // Follow up to 1 m of a collision-free corridor, then replan.
    if(!lineFree(route.front())) return false;
    int chosen=route.front();
    for(int v:route) {
        const double d=std::hypot(xmin+(v%nx)*res-start.x,
                                  ymin+(v/nx)*res-start.y);
        if(d>1.05 || !lineFree(v)) break;
        chosen=v;
    }
    waypoint=start;
    waypoint.x=xmin+(chosen%nx)*res;
    waypoint.y=ymin+(chosen/nx)*res;
    waypoint.z=goal.z;
    return std::hypot(waypoint.x-start.x,waypoint.y-start.y)>0.12;
}
}

void GoTo::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (msg->header.frame_id != "world") return;
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_ = msg;
    cloud_received_ = ros::Time::now();
}

void GoTo::inflatedMapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (msg->header.frame_id != "world") return;
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_inflated_map_ = msg;
    inflated_map_received_ = ros::Time::now();
}

GoTo::GoTo(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config) {}

PortsList GoTo::providedPorts() {
    return { InputPort<Position3D>("goal", "目标位置 (x, y, z), 为空则保持当前位置"),
             InputPort<double>("yaw", "目标绝对航向 (rad), 为空则保持当前航向"),
             InputPort<double>("yaw_relative", "相对当前航向旋转角度(rad)"),
             InputPort<int>("frame", 0, "坐标系: 0=世界系(默认), 1=机体系"),
             InputPort<double>("tolerance", "到达容差"),
             InputPort<double>("yaw_tolerance", "偏航到达容差(rad)"),
             InputPort<double>("yaw_rate", 0.5, "最大偏航角速度 (rad/s)"),
             InputPort<double>("max_setpoint_distance", "限制单次位置设定点的前移距离 (m)"),
             InputPort<bool>("face_travel", false, "返航段先转向航点，再机头向前飞"),
             InputPort<bool>("front_avoidance", false, "使用前视深度点云避障"),
             InputPort<bool>("shortest_detour", false, "仅返航时选择最近可见局部绕行"),
             InputPort<bool>("prefer_right_detour", false, "返航时优先右侧可通行绕行"),
              InputPort<bool>("climb_before_travel", false, "返航前在当前位置垂直升至巡航高度"),
             InputPort<bool>("record_breadcrumbs", false, "记录实际安全飞行路径"),
             OutputPort<Position3D>("trail_1"),
             OutputPort<Position3D>("trail_2"),
             OutputPort<Position3D>("trail_3") };
}

NodeStatus GoTo::onStart() {
    // 获取当前位置和航向
    geometry_msgs::Point current_pos = fcu_odom_ptr->pose.pose.position;
    double current_yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);

    // 解析目标位置
    auto goal_res = getInput<Position3D>("goal");
    auto frame_res = getInput<int>("frame");
    int frame = frame_res.value();

    if (goal_res) {
        Position3D goal = goal_res.value();
        if (frame == 1) {  // 1 = 机体系
            // 机体坐标系转世界坐标系：
            // goal.x = 机体前进方向
            // goal.y = 机体左移方向
            // 需要根据当前yaw角度旋转到世界坐标系
            target_pos.x = current_pos.x + goal.x * std::cos(current_yaw) - goal.y * std::sin(current_yaw);
            target_pos.y = current_pos.y + goal.x * std::sin(current_yaw) + goal.y * std::cos(current_yaw);
            target_pos.z = current_pos.z + goal.z;
        } else {  // 0 = 世界系(默认)
            target_pos.x = goal.x;
            target_pos.y = goal.y;
            target_pos.z = goal.z;
        }
    } else {
        target_pos = current_pos;
    }

    // 解析航向
    face_travel_ = getInput<bool>("face_travel").value();
    front_avoidance_ = getInput<bool>("front_avoidance").value();
    shortest_detour_ = getInput<bool>("shortest_detour").value();
    prefer_right_detour_ = getInput<bool>("prefer_right_detour").value();
    climb_first_pending_ = getInput<bool>("climb_before_travel").value() &&
                           target_pos.z > current_pos.z + 0.15;
    climb_anchor_ = current_pos;
    climb_anchor_.z = target_pos.z;
    record_breadcrumbs_ = getInput<bool>("record_breadcrumbs").value();
    last_breadcrumb_pos_ = {current_pos.x, current_pos.y, current_pos.z};
    for (auto& mark : breadcrumbs_) mark = last_breadcrumb_pos_;
    avoiding_ = false;
    avoidance_attempts_ = 0;
    turn_anchor_ = current_pos;
    if (face_travel_ && target_pos.z >= 1.7)
        turn_anchor_.z = std::min(turn_anchor_.z, target_pos.z + 0.05);
    turning_ = face_travel_;
    braking_ = face_travel_;
    turn_settled_since_ = ros::Time(0);
    brake_settled_since_ = ros::Time(0);
    last_measured_pos_ = current_pos;
    auto yaw_res = getInput<double>("yaw");
    auto yaw_relative_res = getInput<double>("yaw_relative");
    if (yaw_res && yaw_relative_res) {
        ROS_ERROR("[%s]: yaw and yaw_relative cannot both be provided", name().c_str());
        return NodeStatus::FAILURE;
    }
    if (yaw_res) {
        target_yaw = yaw_res.value();
        check_yaw = true;
    } else if (yaw_relative_res) {
        target_yaw = wrapYaw(current_yaw + yaw_relative_res.value());
        check_yaw = true;
    } else if (face_travel_ && std::hypot(target_pos.x-current_pos.x,
                                          target_pos.y-current_pos.y) > 0.25) {
        target_yaw = std::atan2(target_pos.y-current_pos.y,
                                target_pos.x-current_pos.x);
        check_yaw = true;
    } else {
        target_yaw = current_yaw;
        check_yaw = false;
    }

    auto tol_res = getInput<double>("tolerance");
    if (tol_res) {
        tolerance = tol_res.value();
    } else {
        ros::NodeHandle nh;
        nh.param("tolerance", tolerance, 0.2);
    }
    
    auto yaw_tol_res = getInput<double>("yaw_tolerance");
    if (yaw_tol_res) {
        yaw_tolerance = yaw_tol_res.value();
    } else {
        ros::NodeHandle nh;
        nh.param("yaw_tolerance", yaw_tolerance, 0.15);
    }

    auto yaw_rate_res = getInput<double>("yaw_rate");
    yaw_rate_limit = yaw_rate_res.value();

    // A short moving position setpoint avoids issuing a multi-metre step to PX4.
    // It is derived from live odometry every tick, not a preset waypoint.
    ros::NodeHandle nh;
    nh.param("goto/max_setpoint_distance", max_setpoint_distance_, 0.45);
    if (auto step_res = getInput<double>("max_setpoint_distance"))
        max_setpoint_distance_ = step_res.value();
    double safety_z_max = 2.5;
    nh.param("safety/z_max", safety_z_max, 2.5);
    return_ceiling_guard_ = safety_z_max - 0.35;
    if (!std::isfinite(max_setpoint_distance_) || max_setpoint_distance_ <= 0.05 ||
        max_setpoint_distance_ > 1.0) {
        ROS_ERROR("[%s]: Invalid goto/max_setpoint_distance=%.3f", name().c_str(),
                  max_setpoint_distance_);
        return NodeStatus::FAILURE;
    }

    // 初始化偏航设定点为当前实际航向
    current_yaw_sp = current_yaw;
    last_loop_time = ros::Time::now();
    if (front_avoidance_) {
        ros::NodeHandle nh;
        front_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            "/cloud_registered", 1, &GoTo::cloudCallback, this);
        if (shortest_detour_) {
            // GoTo bypasses EGO's trajectory planner. Consume its already
            // inflated occupancy map for return-leg collision checks.
            inflated_map_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
                "/grid_map/occupancy_inflate", 1, &GoTo::inflatedMapCallback, this);
        }
    }

    ROS_INFO("[%s]: GoTo start -> target: (%.2f, %.2f, %.2f), yaw: %.2f, frame: %d, tol: %.2f, yaw_tol: %.2f, yaw_rate: %.2f", 
             name().c_str(), target_pos.x, target_pos.y, target_pos.z, target_yaw, frame, tolerance, yaw_tolerance, yaw_rate_limit);
    return NodeStatus::RUNNING;
}

NodeStatus GoTo::onRunning() {
    // 计算时间差
    ros::Time now = ros::Time::now();
    double dt = (now - last_loop_time).toSec();
    last_loop_time = now;

    const auto& current = fcu_odom_ptr->pose.pose.position;
    const double actual_yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    if (climb_first_pending_) {
        if (current.z < target_pos.z - 0.12 ||
            std::hypot(current.x-climb_anchor_.x, current.y-climb_anchor_.y) > 0.20) {
            publishPosition(climb_anchor_, actual_yaw);
            ROS_INFO_THROTTLE(1.0, "[%s]: climb in place before front-camera return, z=%.2f -> %.2f",
                              name().c_str(), current.z, target_pos.z);
            return NodeStatus::RUNNING;
        }
        climb_first_pending_ = false;
        turn_anchor_ = current;
        braking_ = turning_ = face_travel_;
        turn_settled_since_ = brake_settled_since_ = ros::Time(0);
    }
    if (record_breadcrumbs_ &&
        std::hypot(current.x-last_breadcrumb_pos_.x,
                   current.y-last_breadcrumb_pos_.y) >= 0.55) {
        breadcrumbs_[0] = breadcrumbs_[1];
        breadcrumbs_[1] = breadcrumbs_[2];
        breadcrumbs_[2] = {current.x, current.y, current.z};
        last_breadcrumb_pos_ = breadcrumbs_[2];
    }
    if (face_travel_ && current.z > return_ceiling_guard_ &&
        target_pos.z < return_ceiling_guard_) {
        geometry_msgs::Point altitude_recovery = current;
        altitude_recovery.z = target_pos.z;
        ROS_WARN_THROTTLE(1.0, "[%s]: return altitude %.2fm near ceiling; descend in place to %.2fm",
                          name().c_str(), current.z, target_pos.z);
        publishPosition(altitude_recovery, actual_yaw);
        return NodeStatus::RUNNING;
    }
    const double measured_speed = dt > 0.03 && dt < 0.7
        ? std::hypot(current.x-last_measured_pos_.x,
                     current.y-last_measured_pos_.y)/dt : 0.0;
    last_measured_pos_ = current;
    if (avoiding_ && std::hypot(avoid_target_.x-current.x,
                                avoid_target_.y-current.y) < (shortest_detour_ ? 0.08 : 0.17)) {
        avoiding_ = false;
        turning_ = true;
        braking_ = true;
        turn_anchor_ = current;
        if (target_pos.z >= 1.7)
            turn_anchor_.z = std::min(turn_anchor_.z, target_pos.z + 0.05);
        turn_settled_since_ = ros::Time(0);
        brake_settled_since_ = ros::Time(0);
        ROS_INFO("[%s]: front-camera detour complete; resuming return waypoint", name().c_str());
    }
    geometry_msgs::Point active_target = avoiding_ ? avoid_target_ : target_pos;
    const double horizontal = std::hypot(active_target.x-current.x,
                                         active_target.y-current.y);
    if (face_travel_ && horizontal > 0.25) {
        target_yaw = std::atan2(active_target.y-current.y,
                                active_target.x-current.x);
        check_yaw = true;
    }

    // Finish the previous leg's momentum before rotating toward the next
    // waypoint. Otherwise the aircraft can visibly drift backward during yaw.
    if (face_travel_ && braking_) {
        const double anchor_error = std::hypot(current.x-turn_anchor_.x,
                                               current.y-turn_anchor_.y);
        if (anchor_error < 0.14 && measured_speed < 0.12) {
            if (brake_settled_since_.isZero()) brake_settled_since_ = now;
            if ((now-brake_settled_since_).toSec() >= 0.25) braking_ = false;
        } else {
            brake_settled_since_ = ros::Time(0);
        }
        current_yaw_sp = actual_yaw;
        publishPosition(turn_anchor_, current_yaw_sp);
        return NodeStatus::RUNNING;
    }

    // 偏航角平滑插值 (Ramp)
    double yaw_error = target_yaw - current_yaw_sp;
    
    // 归一化误差到 [-PI, PI]
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    double max_step = yaw_rate_limit * dt;

    if (std::abs(yaw_error) <= max_step) {
        current_yaw_sp = target_yaw;
    } else {
        // 向目标方向步进
        current_yaw_sp += (yaw_error > 0 ? 1.0 : -1.0) * max_step;
    }

    // 归一化设定点 (防止长期累积溢出，虽然 cos/sin 无所谓，但保持规范)
    while (current_yaw_sp > M_PI) current_yaw_sp -= 2 * M_PI;
    while (current_yaw_sp < -M_PI) current_yaw_sp += 2 * M_PI;

    // Limit the position setpoint's distance from the measured pose. The
    // previous direct 4 m return setpoint overshot the x_min safety boundary.
    // Do not translate sideways/backwards while the nose is still turning.
    const double heading_error = std::abs(wrapYaw(target_yaw-actual_yaw));
    if (face_travel_ && horizontal > 0.25 && heading_error > 0.18 && !turning_) {
        // Lock the hover point when initiating a new turn. Continuously
        // chasing the current pose let inertial motion carry the UAV backward.
        turn_anchor_ = current;
        if (target_pos.z >= 1.7)
            turn_anchor_.z = std::min(turn_anchor_.z, target_pos.z + 0.05);
        turning_ = true;
        braking_ = true;
        turn_settled_since_ = ros::Time(0);
        brake_settled_since_ = ros::Time(0);
    }
    if (face_travel_ && turning_) {
        const double drift = std::hypot(current.x-turn_anchor_.x,
                                        current.y-turn_anchor_.y);
        if (heading_error < 0.14 && drift < 0.12) {
            if (turn_settled_since_.isZero()) turn_settled_since_ = now;
            if ((now-turn_settled_since_).toSec() >= 0.25) {
                turning_ = false;
            }
        } else {
            turn_settled_since_ = ros::Time(0);
        }
        if (turning_) {
            publishPosition(turn_anchor_, current_yaw_sp);
            return NodeStatus::RUNNING;
        }
    }
    if (face_travel_ && horizontal > 0.25 && heading_error > 0.18) {
        publishPosition(turn_anchor_, current_yaw_sp);
        return NodeStatus::RUNNING;
    }

    // A fresh world-frame point cloud comes from the forward depth camera.
    // Stop before a near obstacle, then choose a bounded clear lateral bypass.
    if (front_avoidance_ && horizontal > 0.30) {
        sensor_msgs::PointCloud2ConstPtr cloud;
        sensor_msgs::PointCloud2ConstPtr inflated_map;
        ros::Time receipt;
        ros::Time map_receipt;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            cloud = latest_cloud_;
            receipt = cloud_received_;
            inflated_map = latest_inflated_map_;
            map_receipt = inflated_map_received_;
        }
        if (!cloud || (now-receipt).toSec() > 0.8) {
            ROS_WARN_THROTTLE(2.0, "[%s]: front depth cloud stale; holding return leg", name().c_str());
            publishPosition(current, current_yaw_sp);
            return NodeStatus::RUNNING;
        }
        if (shortest_detour_ &&
            (!inflated_map || (now-map_receipt).toSec() > 0.8)) {
            ROS_WARN_THROTTLE(2.0, "[%s]: inflated occupancy map stale; holding return leg",
                              name().c_str());
            publishPosition(current, current_yaw_sp);
            return NodeStatus::RUNNING;
        }
        // The operator-requested map shell is 0.10 m at 0.05 m resolution.
        // Test the swept segment, not just the endpoint; 0.05 m here is one
        // map voxel and not an additional 20 cm safety boundary.
        auto mapBlocked = [&](double sx, double sy, double ex, double ey,
                              double altitude) {
            if (!shortest_detour_) return false;
            const double length = std::hypot(ex-sx, ey-sy);
            if (length < 0.01) return false;
            const double ux = (ex-sx)/length, uy = (ey-sy)/length;
            int hits = 0;
            try {
                sensor_msgs::PointCloud2ConstIterator<float> px(*inflated_map,"x"),
                    py(*inflated_map,"y"), pz(*inflated_map,"z");
                for (; px != px.end(); ++px,++py,++pz) {
                    if (!std::isfinite(*px) || !std::isfinite(*py) || !std::isfinite(*pz) ||
                        std::abs(*pz-altitude) > 0.38) continue;
                    const double dx = *px-sx, dy = *py-sy;
                    const double along = dx*ux+dy*uy;
                    const double side = std::abs(-dx*uy+dy*ux);
                    if (along > 0.12 && along < length+0.10 && side < 0.05 && ++hits >= 2)
                        return true;
                }
            } catch (const std::runtime_error&) { return true; }
            return false;
        };
        auto blocked = [&](double heading, double reach) {
            const double ux = std::cos(heading), uy = std::sin(heading);
            if (mapBlocked(current.x,current.y,
                           current.x+reach*ux,current.y+reach*uy,current.z))
                return true;
            int hits = 0;
            try {
                sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z");
                for (; x != x.end(); ++x, ++y, ++z) {
                    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
                    const double rx = *x-current.x, ry = *y-current.y;
                    const double forward = rx*ux+ry*uy;
                    const double side = -rx*uy+ry*ux;
                    if (forward > 0.15 && forward < reach &&
                        std::abs(side) < 0.23 &&
                        std::abs(*z-current.z) < (shortest_detour_ ? 0.38 : 0.25)) ++hits;
                }
            } catch (const std::runtime_error&) { return true; }
            return hits >= (shortest_detour_ ? 2 : 8);
        };
        const double heading = std::atan2(active_target.y-current.y,
                                          active_target.x-current.x);
        if (blocked(heading, std::min(0.46, horizontal))) {
            if (avoiding_ && shortest_detour_) {
                // A newly visible obstacle invalidates the active bypass.
                // Do not keep pushing into it; replan from measured odometry.
                avoiding_=false;
                turn_anchor_=current;
                ROS_WARN("[%s]: return bypass obstructed; replanning",name().c_str());
            }
            if (shortest_detour_) {
                geometry_msgs::Point route_point;
                if (avoidance_attempts_ < 30 &&
                    planReturnWaypoint(*inflated_map,*cloud,current,target_pos,prefer_right_detour_,route_point)) {
                    avoid_target_=route_point;
                    avoiding_=true;
                    turn_anchor_=current;
                    turning_=braking_=true;
                    turn_settled_since_=brake_settled_since_=ros::Time(0);
                    ++avoidance_attempts_;
                    ROS_WARN("[%s]: inflated-map return waypoint (%.2f,%.2f,%.2f), step %d",
                             name().c_str(),route_point.x,route_point.y,route_point.z,
                             avoidance_attempts_);
                } else {
                    ROS_WARN_THROTTLE(2.0,"[%s]: no collision-free return corridor; holding",
                                      name().c_str());
                }
                publishPosition(current,current_yaw_sp);
                return NodeStatus::RUNNING;
            }
            if (!avoiding_ && avoidance_attempts_ < (shortest_detour_ ? 30 : 2)) {
                const double direct = std::atan2(target_pos.y-current.y,
                                                 target_pos.x-current.x);
                if (!shortest_detour_) {
                    // Retain the previously flown exit-to-hover route: left-first
                    // 0.55 m bypass, not the newer dynamic side selector.
                    for (int side : {1, -1}) {
                        const double turn = direct + side*M_PI/2.0;
                        const double nx = current.x + 0.55*std::cos(turn);
                        const double ny = current.y + 0.55*std::sin(turn);
                        if (nx < -0.70 || nx > 5.70 || ny < -2.70 || ny > 2.70 ||
                            blocked(turn, 0.65)) continue;
                        avoid_target_ = current;
                        avoid_target_.x = nx; avoid_target_.y = ny;
                        avoid_target_.z = target_pos.z;
                        avoiding_ = true;
                        turn_anchor_ = current;
                        if (target_pos.z >= 1.7)
                            turn_anchor_.z = std::min(turn_anchor_.z, target_pos.z + 0.05);
                        turning_ = true;
                        braking_ = true;
                        turn_settled_since_ = ros::Time(0);
                        brake_settled_since_ = ros::Time(0);
                        ++avoidance_attempts_;
                        ROS_WARN("[%s]: front obstacle; original detour 0.55m to %s",
                                 name().c_str(), side > 0 ? "left" : "right");
                        break;
                    }
                } else {
                double best_cost = std::numeric_limits<double>::infinity();
                geometry_msgs::Point best = current;
                int best_side = 0;
                double best_step = 0.0;
                // Return only: after the nose turns 90 degrees, fly a full
                // 1 m lateral bypass before turning back toward home. Check
                // that entire 1 m corridor in fresh front depth and bounds.
                for (double step : {1.0}) {
                  for (int side : {-1, 1}) {
                    const double turn = direct + side*M_PI/2.0;
                    const double nx = current.x + step*std::cos(turn);
                    const double ny = current.y + step*std::sin(turn);
                    if (nx < -0.70 || nx > 5.70 || ny < -2.70 || ny > 2.70 ||
                        blocked(turn, step+0.10)) continue;
                    const double onward = std::hypot(target_pos.x-nx, target_pos.y-ny);
                    const double ux = (target_pos.x-nx)/std::max(onward, 0.01);
                    const double uy = (target_pos.y-ny)/std::max(onward, 0.01);
                    if (mapBlocked(nx,ny,nx+ux*std::min(onward,0.75),
                                   ny+uy*std::min(onward,0.75),current.z)) continue;
                    double clearance = 1.0;
                    int onward_hits = 0;
                    try {
                      sensor_msgs::PointCloud2ConstIterator<float> px(*cloud,"x"), py(*cloud,"y"), pz(*cloud,"z");
                      for (; px != px.end(); ++px,++py,++pz) {
                        if (!std::isfinite(*px) || !std::isfinite(*py) || !std::isfinite(*pz) ||
                            std::abs(*pz-current.z) > 0.38) continue;
                        clearance = std::min(clearance, std::hypot(*px-nx,*py-ny));
                        const double fx = (*px-nx)*ux+(*py-ny)*uy;
                        const double sy = -(*px-nx)*uy+(*py-ny)*ux;
                        if (fx > 0.10 && fx < 0.90 && std::abs(sy) < 0.23) ++onward_hits;
                      }
                    } catch (const std::runtime_error&) { continue; }
                    if (clearance < (prefer_right_detour_ ? 0.40 : 0.30)) continue;
                    const double edge = std::min({nx+0.70,5.70-nx,ny+2.70,2.70-ny});
                    const double cost = step + onward + 0.25/std::max(clearance,0.1) +
                        0.15/std::max(edge,0.1) + 0.06*std::min(onward_hits,10);
                    if (cost + 0.01 < best_cost ||
                        (std::abs(cost-best_cost) <= 0.01 &&
                         side == (avoidance_attempts_%2 == 0 ? -1 : 1))) {
                      best_cost = cost;
                      best = current;
                      best.x = nx; best.y = ny; best.z = target_pos.z;
                      best_side = side;
                      best_step = step;
                    }
                  }
                }
                if (best_side != 0) {
                    avoid_target_ = best;
                    avoiding_ = true;
                    turn_anchor_ = current;
                    if (target_pos.z >= 1.7)
                        turn_anchor_.z = std::min(turn_anchor_.z, target_pos.z + 0.05);
                    turning_ = true;
                    braking_ = true;
                    turn_settled_since_ = ros::Time(0);
                    brake_settled_since_ = ros::Time(0);
                    ++avoidance_attempts_;
                    ROS_WARN("[%s]: front obstacle; shortest visible local detour %.2fm to %s (cost %.2f)",
                             name().c_str(), best_step, best_side > 0 ? "left" : "right", best_cost);
                } else {
                    geometry_msgs::Point route_point;
                    if (planReturnWaypoint(*inflated_map,*cloud,current,target_pos,prefer_right_detour_,route_point)) {
                        avoid_target_=route_point;
                        avoiding_=true;
                        turn_anchor_=current;
                        turning_=braking_=true;
                        turn_settled_since_=brake_settled_since_=ros::Time(0);
                        ++avoidance_attempts_;
                        ROS_WARN("[%s]: 1m sidestep blocked; occupancy-map return waypoint (%.2f,%.2f,%.2f)",
                                 name().c_str(),route_point.x,route_point.y,route_point.z);
                    } else {
                        ROS_WARN_THROTTLE(2.0,"[%s]: no collision-free return route in inflated map; holding",
                                          name().c_str());
                    }
                }
                }
            }
            publishPosition(current, current_yaw_sp);
            return NodeStatus::RUNNING;
        }
    }

    geometry_msgs::Point limited = active_target;
    const double dx = active_target.x - current.x;
    const double dy = active_target.y - current.y;
    const double dz = active_target.z - current.z;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (dist > max_setpoint_distance_) {
        const double scale = max_setpoint_distance_ / dist;
        limited.x = current.x + dx*scale;
        limited.y = current.y + dy*scale;
        limited.z = current.z + dz*scale;
    }
    publishPosition(limited, current_yaw_sp);

    // 到达判断 (只有在明确传入了yaw参数时，才检查yaw是否到达)
    double arrival_yaw = check_yaw ? target_yaw : std::numeric_limits<double>::quiet_NaN();
    if (checkArrival(target_pos, tolerance, arrival_yaw, yaw_tolerance)) {
        if (record_breadcrumbs_) {
            setOutput("trail_1", breadcrumbs_[0]);
            setOutput("trail_2", breadcrumbs_[1]);
            setOutput("trail_3", breadcrumbs_[2]);
        }
        ROS_INFO("[%s]: GoTo completed (pos & yaw).", name().c_str());
        sendHoverCommand();
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<GoTo>("GoTo");
}
