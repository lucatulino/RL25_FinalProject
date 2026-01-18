#include "robot_control.h"
#include <iostream>
#include <cmath>
#include <algorithm> 

RobotController::RobotController(std::shared_ptr<KDLRobot> robot) 
    : robot_(robot) 
{
    last_cmd_vel_ = Eigen::VectorXd::Zero(7);
}

void RobotController::setParams(const ControlParams& params) {
    params_ = params;
}

Eigen::VectorXd RobotController::computeTrapezoidalJointControl(const Eigen::VectorXd &q_des, 
                                                                const Eigen::VectorXd &q_current,
                                                                double dt)
{

    if (last_cmd_vel_.size() != 7) last_cmd_vel_ = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd q_dot_cmd(7);

   
    double slow_down_factor = 0.7; 

    
    double base_acc = (params_.max_acc_joint > 0.001) ? params_.max_acc_joint : 0.5;
    double base_vel = (params_.max_vel_joint > 0.001) ? params_.max_vel_joint : 0.5;

    double global_max_acc = base_acc * slow_down_factor;
    double global_max_vel = base_vel * slow_down_factor;


   
    double max_dist = 0.0;
    for(int i=0; i<7; i++) {
        double dist = std::abs(q_des(i) - q_current(i));
        if(dist > max_dist) max_dist = dist;
    }

   
    if (max_dist < 0.001) max_dist = 0.001; 

 
    for (int i = 0; i < 7; i++) {
       
        if (std::isnan(q_current(i)) || std::isnan(q_des(i))) {
            q_dot_cmd(i) = 0.0;
            continue;
        }

        double error = q_des(i) - q_current(i);
        double dist = std::abs(error);
        double direction = (error > 0) ? 1.0 : -1.0;

        double joint_specific_max_vel = global_max_vel * (dist / max_dist);

        if (joint_specific_max_vel < 0.01) joint_specific_max_vel = 0.01;

        double current_vel_abs = std::abs(last_cmd_vel_(i));
        double brake_dist = (current_vel_abs * current_vel_abs) / (2.0 * global_max_acc);

        double v_target = 0.0;

        if (dist <= brake_dist + 0.01) {
         
            v_target = std::sqrt(2.0 * global_max_acc * dist) * direction;
            
            if (dist < 0.05) {
                 v_target = error * params_.kp_joint; 
            }
        } 

        else {

            v_target = joint_specific_max_vel * direction;
        }


        double max_vel_change = global_max_acc * dt;
        double v_diff = v_target - last_cmd_vel_(i);
        
        v_diff = std::clamp(v_diff, -max_vel_change, max_vel_change);
        
        q_dot_cmd(i) = last_cmd_vel_(i) + v_diff;
        

        q_dot_cmd(i) = std::clamp(q_dot_cmd(i), -global_max_vel, global_max_vel);
    }

    last_cmd_vel_ = q_dot_cmd; 
    return q_dot_cmd;
}

Eigen::VectorXd RobotController::computeCartesianWithNullSpace(const KDL::Frame &des_frame, 
                                                               const Eigen::VectorXd &q_current,
                                                               double dt)
{

    KDL::Frame ee_frame = robot_->getEEFrame();
    Eigen::Vector3d p_des = toEigen(des_frame.p);
    Eigen::Vector3d p_curr = toEigen(ee_frame.p);
    
    Eigen::Vector3d e_p = p_des - p_curr;
    Eigen::Matrix3d R_d = toEigen(des_frame.M);
    Eigen::Matrix3d R_e = toEigen(ee_frame.M);
    Eigen::Vector3d e_o = computeOrientationError(R_d, R_e);

    Eigen::VectorXd v_des(6);
    v_des.head(3) = params_.kp_cartesian * e_p;
    v_des.tail(3) = params_.ko_cartesian * e_o;

    Eigen::MatrixXd J = robot_->getEEJacobian().data; 
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse(); 

    Eigen::VectorXd q_dot_primary = J_pinv * v_des;


    Eigen::VectorXd q_dot_secondary = Eigen::VectorXd::Zero(7);
    Eigen::MatrixXd limits = robot_->getJntLimits(); 


    bool limits_ok = (limits.rows() == 7) && (limits.cols() >= 2);

    if (limits_ok) {
        for (int i = 0; i < 7; i++) {
            double qi = q_current(i);
            double q_min = limits(i, 0);
            double q_max = limits(i, 1);
            
            double range = q_max - q_min;
            double center = (q_max + q_min) / 2.0;
            

            if (std::abs(range) < 0.001) {
                q_dot_secondary(i) = 0.0;
            } else {
                double dist_norm = (qi - center) / (range / 2.0); 
                q_dot_secondary(i) = -params_.null_space_gain * dist_norm; 
            }
        }
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
    Eigen::MatrixXd N = I - (J_pinv * J);
    
    Eigen::VectorXd q_dot_target = q_dot_primary + (N * q_dot_secondary);

    double safe_acc = (params_.max_acc_joint > 0.001) ? params_.max_acc_joint : 0.5;
    double max_vel_change = safe_acc * dt;
    
    Eigen::VectorXd q_dot_final(7);

    for(int i=0; i<7; i++) {
        double v_diff = q_dot_target(i) - last_cmd_vel_(i);
        
        v_diff = std::clamp(v_diff, -max_vel_change, max_vel_change);
        
        q_dot_final(i) = last_cmd_vel_(i) + v_diff;
    }

    double safe_vel = (params_.max_vel_joint > 0.001) ? params_.max_vel_joint : 0.5;
    double max_calc_vel = q_dot_final.cwiseAbs().maxCoeff();
    
    if (max_calc_vel > safe_vel) {
        q_dot_final *= (safe_vel / max_calc_vel);
    }

    if (q_dot_final.hasNaN()) {
        std::cerr << "[RobotControl] EMERGENCY: NaN detected. Stopping robot." << std::endl;
        last_cmd_vel_ = Eigen::VectorXd::Zero(7);
        return Eigen::VectorXd::Zero(7);
    }

    last_cmd_vel_ = q_dot_final; 
    return q_dot_final;
}

bool RobotController::isJointTargetReached(const Eigen::VectorXd &q_des, 
                                           const Eigen::VectorXd &q_current, 
                                           double tolerance) {
    return (q_des - q_current).norm() < tolerance;
}

bool RobotController::isCartesianTargetReached(const KDL::Frame &des_frame, 
                                               double tolerance_pos, 
                                               double tolerance_rot) {
    KDL::Frame ee_frame = robot_->getEEFrame();
    double pos_err = (des_frame.p - ee_frame.p).Norm();
    
    Eigen::Matrix3d R_d = toEigen(des_frame.M);
    Eigen::Matrix3d R_e = toEigen(ee_frame.M);
    double rot_err = computeOrientationError(R_d, R_e).norm();

    return (pos_err < tolerance_pos) && (rot_err < tolerance_rot);
}
