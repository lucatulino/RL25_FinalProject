#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class TurtlebotPatrolNode : public rclcpp::Node
{
public:
    TurtlebotPatrolNode() : Node("turtlebot_patrol")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        ready_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/turtlebot_ready", 10);

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TurtlebotPatrolNode::odom_callback, this, std::placeholders::_1));
        
        aruco_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single_turtle/pose", 10, std::bind(&TurtlebotPatrolNode::aruco_pose_callback, this, std::placeholders::_1));

        sub_transfer_trigger_ = this->create_subscription<std_msgs::msg::Bool>(
            "/transfer_trigger", 10, std::bind(&TurtlebotPatrolNode::trigger_callback, this, std::placeholders::_1));

        this->declare_parameter("patrol.max_linear_vel", 0.22);
        this->declare_parameter("patrol.scan_rotation_speed", 0.35);
        this->declare_parameter("patrol.visual_servo_kp", 0.8);
        this->declare_parameter("patrol.timeout_duration", 0.5);

        max_linear_vel_ = this->get_parameter("patrol.max_linear_vel").as_double();
        scan_speed_ = this->get_parameter("patrol.scan_rotation_speed").as_double();
        visual_servo_kp_ = this->get_parameter("patrol.visual_servo_kp").as_double();
        timeout_duration_ = this->get_parameter("patrol.timeout_duration").as_double();

        timer_ = this->create_wall_timer(50ms, std::bind(&TurtlebotPatrolNode::control_loop, this));

        last_aruco_time_ = this->now();
        x_start_mission_ = 0.0;
        current_linear_cmd_ = 0.0;
        marker_yaw_error_ = 0.0;
        state_ = 2; 

        RCLCPP_INFO(this->get_logger(), "System initialized, parameters loaded.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        try {
            current_x_ = msg->pose.pose.position.x;
            
            tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double r, p, y;
            m.getRPY(r, p, y);
            current_yaw_ = y; 
            odom_received_ = true;
        } catch (...) {
        }
    }

    void aruco_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        try {
            tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                              msg->pose.orientation.z, msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double r, p, y;
            m.getRPY(r, p, y);
            
            marker_yaw_error_ = p; 
            
            last_aruco_time_ = this->now();
            aruco_detected_ = true;
        } catch (...) {
        }
    }

    void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) transfer_triggered_ = true;
    }

    void control_loop()
    {
        if (!odom_received_) return;

        double seconds_since_aruco = (this->now() - last_aruco_time_).seconds();
        if (seconds_since_aruco > timeout_duration_) {
            aruco_detected_ = false;
        }

        geometry_msgs::msg::Twist cmd_msg;
        std_msgs::msg::Bool ready_msg;

        const double MAX_LIN_ACCEL = 0.05 * 0.05; 

        switch (state_)
        {
        case 1: 
            ready_msg.data = false; ready_publisher_->publish(ready_msg);
            {
                double dist_to_target = 1.45 - std::abs(current_x_ - x_start_mission_);
                double target_vel = 0.0;

                if (dist_to_target > 0.02) {
                    if (dist_to_target < 0.5) target_vel = std::max(0.4 * dist_to_target, 0.02);
                    else target_vel = max_linear_vel_;
                }

                if (target_vel > current_linear_cmd_) current_linear_cmd_ += MAX_LIN_ACCEL;
                else current_linear_cmd_ -= MAX_LIN_ACCEL;
                
                current_linear_cmd_ = std::clamp(current_linear_cmd_, 0.0, max_linear_vel_);

                cmd_msg.linear.x = current_linear_cmd_;
                cmd_msg.angular.z = 0.0;

                if (dist_to_target <= 0.02 && current_linear_cmd_ < 0.01) {
                    cmd_msg.linear.x = 0.0;
                    current_linear_cmd_ = 0.0;
                    

                    RCLCPP_INFO(this->get_logger(), "Position reached. Starting area scan.");
                    state_ = 3; 
                    detection_persistence_ = 0;
                }
            }
            break;

        case 3: 
            ready_msg.data = false; ready_publisher_->publish(ready_msg);
            cmd_msg.linear.x = 0.0;

            if (aruco_detected_) {
                cmd_msg.angular.z = 0.0; 
                
                detection_persistence_++;
                if (detection_persistence_ > 3) {

                    RCLCPP_INFO(this->get_logger(), "Target acquired. Precision aligning.");
                    state_ = 4;
                    detection_persistence_ = 0;
                }
            } else {
                detection_persistence_ = 0;
                cmd_msg.angular.z = scan_speed_; 
            }
            break;

        case 4: 
            ready_msg.data = false; ready_publisher_->publish(ready_msg);
            cmd_msg.linear.x = 0.0;

            if (aruco_detected_) {
                double err = marker_yaw_error_;
                
                if (std::abs(err) > 0.005) {
                    double v_z = std::clamp(visual_servo_kp_ * err, -0.2, 0.2);
                    
                    if (std::abs(v_z) < 0.05) v_z = (v_z > 0) ? 0.05 : -0.05;
                    
                    double sign_correction = (current_x_ < 0.75) ? -1.0 : 1.0; 
                    cmd_msg.angular.z = v_z * sign_correction;
                    
                    alignment_counter_ = 0;
                } else {
                    cmd_msg.angular.z = 0.0;
                    if (++alignment_counter_ > 20) { 

                        RCLCPP_INFO(this->get_logger(), "Alignment finished. Standing by.");
                        state_ = 2; 
                    }
                }
            } else {
                cmd_msg.angular.z = 0.0;
                if (++alignment_counter_ > 40) { 

                    RCLCPP_WARN(this->get_logger(), "Target tracking lost. Resuming scan.");
                    state_ = 3; 
                }
            }
            break;

        case 2: 
            ready_msg.data = true; 
            ready_publisher_->publish(ready_msg);
            cmd_msg.linear.x = 0.0; 
            cmd_msg.angular.z = 0.0;

            if (transfer_triggered_) {
                x_start_mission_ = current_x_;
                transfer_triggered_ = false;
                current_linear_cmd_ = 0.0; 
                state_ = 1; 
               
                RCLCPP_INFO(this->get_logger(), "Transfer signal received. Moving out.");
            }
            break;
        }

        publisher_->publish(cmd_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_transfer_trigger_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_subscriber_;
    
    bool odom_received_ = false;
    bool transfer_triggered_ = false;
    bool aruco_detected_ = false;
    
    double current_x_ = 0.0, current_yaw_ = 0.0;
    double x_start_mission_ = 0.0;
    double current_linear_cmd_ = 0.0;
    double marker_yaw_error_ = 0.0;

    double max_linear_vel_;
    double scan_speed_;
    double visual_servo_kp_;
    double timeout_duration_;
    
    int state_ = 2; 
    int detection_persistence_ = 0;
    int alignment_counter_ = 0;
    
    rclcpp::Time last_aruco_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotPatrolNode>());
    rclcpp::shutdown();
    return 0;
}
