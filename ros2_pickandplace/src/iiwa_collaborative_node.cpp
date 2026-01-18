#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "kdl_robot.h"
#include "kdl_parser/kdl_parser.hpp"
#include "robot_control.h"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using BoolMsg = std_msgs::msg::Bool;
using namespace std::chrono_literals;

Eigen::VectorXd toEigen(const std::vector<double>& vec) {
    return Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size());
}

class IiwaCollaborativeNode : public rclcpp::Node {
public:
    IiwaCollaborativeNode() : Node("iiwa_collaborative_node") {
        this->declare_parameter("robot_prefix", "left_");
        this->declare_parameter("base_link", "left_link_0");
        this->declare_parameter("tool_link", "left_tool0");

        robot_prefix_ = this->get_parameter("robot_prefix").as_string();
        base_link_ = this->get_parameter("base_link").as_string();
        tool_link_ = this->get_parameter("tool_link").as_string();
        std::string partner_prefix = (robot_prefix_ == "left_") ? "right_" : "left_";

        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) return;
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});
        KDL::Tree robot_tree;
        kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree);
        robot_ = std::make_shared<KDLRobot>(robot_tree, base_link_, tool_link_);

        robot_controller_ = std::make_shared<RobotController>(robot_);
        
        this->declare_parameter("control.max_vel_joint", 1.5);
        this->declare_parameter("control.max_acc_joint", 2.5);
        this->declare_parameter("control.kp_joint", 3.0);
        this->declare_parameter("control.kp_cartesian", 3.0);
        this->declare_parameter("control.ko_cartesian", 0.5);
        this->declare_parameter("control.null_space_gain", 10.0);

        this->declare_parameter("logic.tool_offset", 0.15);
        this->declare_parameter("logic.bot_height", 0.18);
        this->declare_parameter("logic.drop_margin", 0.05);
        this->declare_parameter("logic.gripper_close", -0.02);
        this->declare_parameter("logic.gripper_open", 0.02);
        this->declare_parameter("logic.search_amplitude", 1.8);

        ControlParams params;
        params.max_vel_joint = this->get_parameter("control.max_vel_joint").as_double();
        params.max_acc_joint = this->get_parameter("control.max_acc_joint").as_double();
        params.kp_joint = this->get_parameter("control.kp_joint").as_double();
        params.kp_cartesian = this->get_parameter("control.kp_cartesian").as_double();
        params.ko_cartesian = this->get_parameter("control.ko_cartesian").as_double();
        params.null_space_gain = this->get_parameter("control.null_space_gain").as_double();
        robot_controller_->setParams(params);

        tool_offset_ = this->get_parameter("logic.tool_offset").as_double();
        bot_height_ = this->get_parameter("logic.bot_height").as_double();
        drop_margin_ = this->get_parameter("logic.drop_margin").as_double();
        gripper_close_val_ = this->get_parameter("logic.gripper_close").as_double();
        gripper_open_val_ = this->get_parameter("logic.gripper_open").as_double();
        search_amplitude_ = this->get_parameter("logic.search_amplitude").as_double();

        joint_positions_.resize(7); joint_velocities_.resize(7);
        joint_velocities_cmd_ = Eigen::VectorXd::Zero(7);

        q_watch_base_ = {0.0, 1.007, 0.0, -0.534, 0.0, 1.57, 0.0};
        if (robot_prefix_ == "right_") {
            center_angle_ = 1.57;
            q_watch_base_ = {0.0, -1.007, 0.0, 0.534, 0.0, -1.57, 0.0};
            my_turn_ = false;
        } else {
            center_angle_ = -1.57;
            my_turn_ = true;
        }

        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&IiwaCollaborativeNode::joint_state_cb, this, std::placeholders::_1));
        arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&IiwaCollaborativeNode::aruco_cb, this, std::placeholders::_1));
        partnerBusySubscriber_ = this->create_subscription<BoolMsg>(
            "/" + partner_prefix + "busy", 10, std::bind(&IiwaCollaborativeNode::partner_busy_cb, this, std::placeholders::_1));
        turtleReadySubscriber_ = this->create_subscription<BoolMsg>(
            "/turtlebot_ready", 10, std::bind(&IiwaCollaborativeNode::turtle_ready_cb, this, std::placeholders::_1));

        cmdPublisher_ = this->create_publisher<FloatArray>("/" + robot_prefix_ + "velocity_controller/commands", 10);
        gripperPublisher_ = this->create_publisher<FloatArray>("/" + robot_prefix_ + "gripper_controller/commands", 10);
        busyPublisher_ = this->create_publisher<BoolMsg>("/" + robot_prefix_ + "busy", 10);
        transferPublisher_ = this->create_publisher<BoolMsg>("/transfer_trigger", 10);

        last_aruco_time_ = this->now();
        timer_ = this->create_wall_timer(50ms, std::bind(&IiwaCollaborativeNode::control_loop, this));
        fixed_rotation_ = KDL::Rotation::RPY(M_PI, 0.0, 0.0);
        
        RCLCPP_INFO(this->get_logger(), "Iiwa collaborative node started.");
    }

private:
    void aruco_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        aruco_pos_image_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        aruco_detected_ = true;
        last_aruco_time_ = this->now();
    }
    void partner_busy_cb(const BoolMsg::SharedPtr msg) { partner_is_busy_ = msg->data; }
    void turtle_ready_cb(const BoolMsg::SharedPtr msg) { turtlebot_is_ready_ = msg->data; }

    double get_target_z(double current_z, double surface_h, double tool_offset, double margin) {
        double limit_z = surface_h + tool_offset + margin; 
        return std::max(current_z + tool_offset, limit_z);
    }

    void control_loop() {
        if (!joint_state_available_) return;
        if ((this->now() - last_aruco_time_) > 500ms) aruco_detected_ = false;

        BoolMsg busy_msg; busy_msg.data = is_working_;
        busyPublisher_->publish(busy_msg);

        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame ee_frame = robot_->getEEFrame();

        if (robot_state_ == 1 && aruco_detected_) {
            if (aruco_pos_image_.norm() < 0.9) {
                has_seen_cube_ = true;
                double safe_z = std::max(0.0, aruco_pos_image_(2));
                last_known_pos_ = KDL::Vector(aruco_pos_image_(0), aruco_pos_image_(1), safe_z);
                
                pick_pose_angle_ = joint_positions_(0);
                robot_state_ = 2; 
                is_working_ = true;
                alignment_counter_ = 0;
                RCLCPP_INFO(this->get_logger(), "Target detected. Aligning for pick.");
            }
        }

        KDL::Frame target_frame;
        bool use_cartesian = false;

        switch (robot_state_) {
            case 0: 
                use_cartesian = false; gripper_val_ = gripper_open_val_; is_working_ = false; search_initialized_ = false;
                joint_velocities_cmd_ = robot_controller_->computeTrapezoidalJointControl(toEigen(q_watch_base_), joint_positions_.data, 0.05);

                if (partner_was_busy_ && !partner_is_busy_) my_turn_ = true;
                partner_was_busy_ = partner_is_busy_;
                
                if (my_turn_ && !partner_is_busy_) {
                    if (robot_controller_->isJointTargetReached(toEigen(q_watch_base_), joint_positions_.data, 0.1)) {
                        robot_state_ = 1;
                        RCLCPP_INFO(this->get_logger(), "Starting scan.");
                    }
                }
                break;

            case 1: 
                use_cartesian = false; gripper_val_ = gripper_open_val_; is_working_ = true;
                if (!search_initialized_) { search_start_time_ = this->now().seconds(); search_initialized_ = true; }
                {
                    double elapsed = this->now().seconds() - search_start_time_;
                    std::vector<double> q_search = q_watch_base_;
                    double sweep = search_amplitude_ * std::pow(std::sin(elapsed * 0.15), 2);
                    q_search[0] = (robot_prefix_ == "right_") ? sweep : -sweep;
                    
                    joint_velocities_cmd_ = robot_controller_->computeTrapezoidalJointControl(toEigen(q_search), joint_positions_.data, 0.05);
                }
                break;

            case 2: 
                is_working_ = true; gripper_val_ = gripper_open_val_;
                
                if (aruco_detected_) {
                    double safe_z = std::max(0.0, aruco_pos_image_(2));
                    last_known_pos_ = KDL::Vector(aruco_pos_image_(0), aruco_pos_image_(1), safe_z);
                }

                {
                    KDL::Vector tracking_target(last_known_pos_.x(), last_known_pos_.y(), ee_frame.p.z());
                    target_frame = KDL::Frame(KDL::Rotation::RPY(M_PI, 0.0, 0.0), tracking_target);

                    double err_xy = std::hypot(ee_frame.p.x() - tracking_target.x(), ee_frame.p.y() - tracking_target.y());
                    
                    use_cartesian = true;
                    if (err_xy > 0.005) alignment_counter_ = 0; else alignment_counter_++;

                    bool signal_ok = false;
                    if (std::abs(joint_positions_(0)) < 0.5) signal_ok = turtlebot_is_ready_ && (alignment_counter_ > 10);
                    else signal_ok = (alignment_counter_ > 10);

                    if (signal_ok && err_xy < 0.02) {
                        RCLCPP_INFO(this->get_logger(), "Alignment complete. Executing pick.");
                        pick_target_ = tracking_target;
                        
                        double surface_z = (last_known_pos_.z() > 0.15) ? bot_height_ : 0.0;
                        double target_z = get_target_z(last_known_pos_.z(), surface_z, tool_offset_, 0.0);
                        pick_target_.z(target_z);

                        double r, p, y; ee_frame.M.GetRPY(r, p, y);
                        fixed_rotation_ = KDL::Rotation::RPY(M_PI, 0.0, y);
                        
                        robot_state_ = 3;
                    }
                }
                break;

            case 3: 
                target_frame = KDL::Frame(fixed_rotation_, pick_target_);
                use_cartesian = true; gripper_val_ = gripper_open_val_;
                if (robot_controller_->isCartesianTargetReached(target_frame, 0.02, 100.0)) { 
                    RCLCPP_INFO(this->get_logger(), "Position reached. Closing gripper.");
                    wait_counter_ = 0; robot_state_ = 4; 
                }
                break;

            case 4: 
                target_frame = KDL::Frame(fixed_rotation_, pick_target_);
                use_cartesian = true; gripper_val_ = gripper_close_val_;
                if (++wait_counter_ > 20) {
                    RCLCPP_INFO(this->get_logger(), "Object picked.");
                    robot_state_ = 5;
                }
                break;

            case 5: 
                use_cartesian = false; gripper_val_ = gripper_close_val_;
                {
                    std::vector<double> q_safe = q_watch_base_;
                    q_safe[0] = joint_positions_(0);
                    q_safe[6] = joint_positions_(6);
                    joint_velocities_cmd_ = robot_controller_->computeTrapezoidalJointControl(toEigen(q_safe), joint_positions_.data, 0.05);

                    if (robot_controller_->isJointTargetReached(toEigen(q_safe), joint_positions_.data, 0.2)) {
                        robot_state_ = 6; wait_counter_ = 0;
                        if (std::abs(pick_pose_angle_) < 0.5) {
                            BoolMsg trg; trg.data = true; transferPublisher_->publish(trg);
                        }
                    }
                }
                break;

            case 6: 
                use_cartesian = false; gripper_val_ = gripper_close_val_;
                {
                    std::vector<double> q_hold = q_watch_base_;
                    q_hold[0] = joint_positions_(0);
                    q_hold[6] = joint_positions_(6);
                    
                    joint_velocities_cmd_ = robot_controller_->computeTrapezoidalJointControl(toEigen(q_hold), joint_positions_.data, 0.05);
                    if (++wait_counter_ > 20) {
                        RCLCPP_INFO(this->get_logger(), "Moving to delivery zone.");
                        robot_state_ = 7;
                    }
                }
                break;

            case 7: 
                use_cartesian = false; gripper_val_ = gripper_close_val_;
                {
                    std::vector<double> q_drop = q_watch_base_;
                    bool going_center = (std::abs(pick_pose_angle_) < 0.5);
                    q_drop[0] = going_center ? center_angle_ : 0.0;
                    q_drop[6] = joint_positions_(6);

                    joint_velocities_cmd_ = robot_controller_->computeTrapezoidalJointControl(toEigen(q_drop), joint_positions_.data, 0.05);

                    if (std::abs(q_drop[0] - joint_positions_(0)) < 0.05) {
                        if (!going_center && !turtlebot_is_ready_) {
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Waiting for Turtlebot...");
                        } else {
                            double surface_z = going_center ? 0.0 : bot_height_;
                            drop_target_ = ee_frame.p;
                            
                            double target_z = get_target_z(0.0, surface_z, tool_offset_, drop_margin_);
                            drop_target_.z(target_z);

                            double r, p, y; ee_frame.M.GetRPY(r, p, y);
                            fixed_rotation_ = KDL::Rotation::RPY(M_PI, 0.0, y);
                            
                            RCLCPP_INFO(this->get_logger(), "Approaching delivery point.");
                            robot_state_ = 8;
                        }
                    }
                }
                break;

            case 8: 
                use_cartesian = true; gripper_val_ = gripper_close_val_;
                target_frame = KDL::Frame(fixed_rotation_, drop_target_);
                if (robot_controller_->isCartesianTargetReached(target_frame, 0.02, 100.0)) { 
                    wait_counter_ = 0; robot_state_ = 9; 
                }
                break;

            case 9: 
                use_cartesian = true; target_frame = KDL::Frame(fixed_rotation_, drop_target_);
                wait_counter_++;
                if (wait_counter_ < 20) {
                    gripper_val_ = gripper_close_val_;
                } else {
                    if (wait_counter_ == 20) RCLCPP_INFO(this->get_logger(), "Object released.");
                    gripper_val_ = gripper_open_val_;
                }

                if (wait_counter_ > 45) {
                    RCLCPP_INFO(this->get_logger(), "Returning home.");
                    robot_state_ = 10;
                }
                break;

            case 10: 
                use_cartesian = false; gripper_val_ = gripper_open_val_;
                {
                    std::vector<double> q_lift = q_watch_base_;
                    q_lift[0] = joint_positions_(0); 
                    
                    joint_velocities_cmd_ = robot_controller_->computeTrapezoidalJointControl(toEigen(q_lift), joint_positions_.data, 0.05);
                    if (robot_controller_->isJointTargetReached(toEigen(q_lift), joint_positions_.data, 0.1)) { 
                        wait_counter_ = 0; robot_state_ = 11; 
                    }
                }
                break;

            case 11: 
                use_cartesian = false; gripper_val_ = gripper_open_val_;
                {
                    joint_velocities_cmd_ = robot_controller_->computeTrapezoidalJointControl(toEigen(q_watch_base_), joint_positions_.data, 0.05);
                    if (robot_controller_->isJointTargetReached(toEigen(q_watch_base_), joint_positions_.data, 0.05)) { 
                        wait_counter_ = 0; robot_state_ = 12; 
                    }
                }
                break;

            case 12: 
                use_cartesian = false; gripper_val_ = gripper_open_val_;
                joint_velocities_cmd_ = Eigen::VectorXd::Zero(7);

                if (++wait_counter_ > 20) {
                    if (std::abs(pick_pose_angle_) < 0.5) { 
                        RCLCPP_INFO(this->get_logger(), "Cycle complete. Waiting for next turn.");
                        my_turn_ = false; is_working_ = false; robot_state_ = 0; 
                    } else { 
                        RCLCPP_INFO(this->get_logger(), "Cycle complete. Searching for next target.");
                        my_turn_ = true; is_working_ = true; robot_state_ = 1; 
                    }
                    has_seen_cube_ = false; search_initialized_ = false; wait_counter_ = 0;
                }
                break;
        }

        if (use_cartesian) {
            joint_velocities_cmd_ = robot_controller_->computeCartesianWithNullSpace(target_frame, joint_positions_.data, 0.05);
        }

        publish_cmd();
        publish_gripper_cmd();
    }

    void joint_state_cb(const sensor_msgs::msg::JointState& msg) {
        int found = 0;
        for (int j = 0; j < 7; j++) {
            std::string name = robot_prefix_ + "joint_a" + std::to_string(j + 1);
            for (size_t i = 0; i < msg.name.size(); i++) {
                if (msg.name[i] == name) {
                    joint_positions_(j) = msg.position[i];
                    joint_velocities_(j) = msg.velocity[i];
                    found++; break;
                }
            }
        }
        if (found == 7) joint_state_available_ = true;
    }

    void publish_cmd() {
        FloatArray msg; for (int i = 0; i < 7; i++) msg.data.push_back(joint_velocities_cmd_(i));
        cmdPublisher_->publish(msg);
    }
    void publish_gripper_cmd() {
        FloatArray msg; msg.data.push_back(gripper_val_); msg.data.push_back(gripper_val_);
        gripperPublisher_->publish(msg);
    }

    std::string robot_prefix_, base_link_, tool_link_;
    double center_angle_;
    std::shared_ptr<KDLRobot> robot_;
    std::shared_ptr<RobotController> robot_controller_;

    double tool_offset_;
    double bot_height_;
    double drop_margin_;
    double gripper_close_val_;
    double gripper_open_val_;
    double search_amplitude_;

    KDL::JntArray joint_positions_, joint_velocities_;
    Eigen::VectorXd joint_velocities_cmd_;
    bool joint_state_available_ = false;
    int robot_state_ = 0;
    bool aruco_detected_ = false, has_seen_cube_ = false, is_working_ = false;
    bool partner_is_busy_ = false, partner_was_busy_ = false, my_turn_ = false;
    bool search_initialized_ = false, turtlebot_is_ready_ = false;
    Eigen::Vector3d aruco_pos_image_;
    rclcpp::Time last_aruco_time_;
    double search_start_time_ = 0.0, pick_pose_angle_ = 0.0;
    KDL::Vector last_known_pos_, pick_target_, drop_target_;
    KDL::Rotation fixed_rotation_;
    std::vector<double> q_watch_base_;
    int wait_counter_ = 0, alignment_counter_ = 0;
    double gripper_val_ = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
    rclcpp::Subscription<BoolMsg>::SharedPtr partnerBusySubscriber_, turtleReadySubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_, gripperPublisher_;
    rclcpp::Publisher<BoolMsg>::SharedPtr busyPublisher_, transferPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IiwaCollaborativeNode>());
    rclcpp::shutdown();
    return 0;
}
