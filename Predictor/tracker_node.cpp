// Copyright 2022 Chen Jun
#include "tracker_node.hpp"
#include "opencv2/core/types.hpp"

// STD
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

namespace rm_auto_aim
{

int ArmorTrackerNode::declare_Parameter(std::string s, int a)
{
    std::cout << s << ":" << a << std::endl;
    return a;
}

double ArmorTrackerNode::declare_Parameter(std::string s, double a)
{
    std::cout << s << ":" << a << std::endl;
    return a;
}

ArmorTrackerNode::ArmorTrackerNode(const int &options)
{
    // Maximum allowable armor distance in the XOY plane
    max_armor_distance_ = this->declare_Parameter("max_armor_distance", 10.0);

    // Tracker
    double max_match_distance = this->declare_Parameter("tracker.max_match_distance", 0.15);
    double max_match_yaw_diff = this->declare_Parameter("tracker.max_match_yaw_diff", 1.0);
    tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
    tracker_->tracking_thres = this->declare_Parameter("tracker.tracking_thres", 5);
    lost_time_thres_ = this->declare_Parameter("tracker.lost_time_thres", 0.3);

    float k = this->declare_Parameter("tracker.k", 0.092);
    int bias_time = this->declare_Parameter("tracker.bias_time", 100);
    float s_bias = this->declare_Parameter("tracker.s_bias", 0.19133);
    float z_bias = this->declare_Parameter("tracker.z_bias", 0.21265);
    gaf_solver = std::make_unique<SolveTrajectory>(k, bias_time, s_bias, z_bias);

    // EKF
    // xa = x_armor, xc = x_robot_center
    // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
    // measurement: xa, ya, za, yaw
    // f - Process function
    auto f = [this](const Eigen::VectorXd &x) {
        Eigen::VectorXd x_new = x;
        x_new(0) += x(1) * dt_;
        x_new(2) += x(3) * dt_;
        x_new(4) += x(5) * dt_;
        x_new(6) += x(7) * dt_;
        return x_new;
    };
    // J_f - Jacobian of process function
    auto j_f = [this](const Eigen::VectorXd &) {
        Eigen::MatrixXd f(9, 9);
        // clang-format off
    f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
          0,   1,   0,   0,   0,   0,   0,   0,   0,
          0,   0,   1,   dt_, 0,   0,   0,   0,   0, 
          0,   0,   0,   1,   0,   0,   0,   0,   0,
          0,   0,   0,   0,   1,   dt_, 0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   dt_, 0,
          0,   0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
        // clang-format on
        return f;
    };
    // h - Observation function
    auto h = [](const Eigen::VectorXd &x) {
        Eigen::VectorXd z(4);
        double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
        z(0) = xc - r * cos(yaw); // xa
        z(1) = yc - r * sin(yaw); // ya
        z(2) = x(4);              // za
        z(3) = x(6);              // yaw
        return z;
    };
    // J_h - Jacobian of observation function
    auto j_h = [](const Eigen::VectorXd &x) {
        Eigen::MatrixXd h(4, 9);
        double yaw = x(6), r = x(8);
        // clang-format off
    //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
    h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
          0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
          0,   0,   0,   0,   1,   0,   0,          0,   0,
          0,   0,   0,   0,   0,   0,   1,          0,   0;
        // clang-format on
        return h;
    };
    // update_Q - process noise covariance matrix
    s2qxyz_ = declare_Parameter("ekf.sigma2_q_xyz", 20.0);
    s2qyaw_ = declare_Parameter("ekf.sigma2_q_yaw", 100.0);
    s2qr_ = declare_Parameter("ekf.sigma2_q_r", 800.0);
    auto u_q = [this]() {
        Eigen::MatrixXd q(9, 9);
        double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
        double q_r = pow(t, 4) / 4 * r;
        // clang-format off
    //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
          0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
          0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
          0,      0,      0,      0,      0,      0,      0,      0,      q_r;
        // clang-format on
        return q;
    };
    // update_R - measurement noise covariance matrix
    r_xyz_factor = declare_Parameter("ekf.r_xyz_factor", 0.05);
    r_yaw = declare_Parameter("ekf.r_yaw", 0.02);
    auto u_r = [this](const Eigen::VectorXd &z) {
        Eigen::DiagonalMatrix<double, 4> r;
        double x = r_xyz_factor;
        r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
        return r;
    };
    // P - error estimate covariance matrix
    Eigen::DiagonalMatrix<double, 9> p0;
    p0.setIdentity();
    tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

    target_frame_ = declare_Parameter("target_frame", "odom");
}

void ArmorTrackerNode::velocityCallback(const std::shared_ptr<msg::Velocity> velocity_msg)
{

    gaf_solver->init(velocity_msg);
}

void ArmorTrackerNode::armorsCallback(const std::shared_ptr<msg::Armors> armors_msg, msg::TrackerInfo info_msg,
                                      msg::Target target_msg, msg::Send send_msg)
{
    // Tranform armor position from image frame to world coordinate
    for (auto &armor : armors_msg->armors)
    {
        msg::PoseStamped ps;
        ps.pose.orientation = armor.pose.orientation;
        ps.pose.position = armor.pose.position;
        // armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    }

    // Filter abnormal armors
    armors_msg->armors.erase(
        std::remove_if(armors_msg->armors.begin(), armors_msg->armors.end(),
                       [this](const msg::Armor &armor) {
                           return abs(armor.pose.position.z) > 1.2 ||
                                  Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                                      max_armor_distance_;
                       }),
        armors_msg->armors.end());

    // Init message
    // msg::TrackerInfo info_msg;
    // msg::Target target_msg;
    // msg::Send send_msg;
    target_msg.timestamp = armors_msg->timestamp;
    // target_msg.frame_id = target_frame_;

    // Update tracker
    if (tracker_->tracker_state == Tracker::LOST)
    {
        tracker_->init(armors_msg);
        target_msg.tracking = false;
    }
    else
    {
        dt_ = std::chrono::duration_cast<std::chrono::milliseconds>(time_ - last_time_).count();
        tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);
        tracker_->update(armors_msg);

        // Publish Info
        info_msg.position_diff = tracker_->info_position_diff;
        info_msg.yaw_diff = tracker_->info_yaw_diff;
        info_msg.position.x = tracker_->measurement(0);
        info_msg.position.y = tracker_->measurement(1);
        info_msg.position.z = tracker_->measurement(2);
        info_msg.yaw = tracker_->measurement(3);
        // publish(info_msg);

        if (tracker_->tracker_state == Tracker::DETECTING)
        {
            target_msg.tracking = false;
        }
        else if (tracker_->tracker_state == Tracker::TRACKING || tracker_->tracker_state == Tracker::TEMP_LOST)
        {
            target_msg.tracking = true;
            // Fill target message
            const auto &state = tracker_->target_state;
            target_msg.id = tracker_->tracked_id;
            target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
            target_msg.position.x = state(0);
            target_msg.velocity.x = state(1);
            target_msg.position.y = state(2);
            target_msg.velocity.y = state(3);
            target_msg.position.z = state(4);
            target_msg.velocity.z = state(5);
            target_msg.yaw = state(6);
            target_msg.v_yaw = state(7);
            target_msg.radius_1 = state(8);
            target_msg.radius_2 = tracker_->another_r;
            target_msg.dz = tracker_->dz;

            float pitch = 0, yaw = 0, aim_x = 0, aim_y = 0, aim_z = 0;
            auto msg = std::make_shared<msg::Target>(target_msg);
            gaf_solver->autoSolveTrajectory(pitch, yaw, aim_x, aim_y, aim_z, msg);

            send_msg.tracking = target_msg.tracking; // Set the tracking value based on the actual condition
            send_msg.position.x = aim_x;
            send_msg.position.y = aim_y;
            send_msg.position.z = aim_z;
            send_msg.pitch = pitch;
            send_msg.yaw = yaw;
        }
    }

    last_time_ = time_;
}

} // namespace rm_auto_aim
