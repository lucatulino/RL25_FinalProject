#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "kdl_robot.h"
#include "utils.h" 
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>


struct ControlParams {
    double max_vel_joint = 1.5;    
    double max_acc_joint = 2.5;   
    double kp_joint = 2.5;         
    
    double kp_cartesian = 3.0;    
    double ko_cartesian = 3.0;   
    double null_space_gain = 10.0;
};

class RobotController {
public:
    RobotController(std::shared_ptr<KDLRobot> robot);
    void setParams(const ControlParams& params);

    Eigen::VectorXd computeTrapezoidalJointControl(const Eigen::VectorXd &q_des, 
                                                   const Eigen::VectorXd &q_current,
                                                   double dt);


    Eigen::VectorXd computeCartesianWithNullSpace(const KDL::Frame &des_frame, 
                                              const Eigen::VectorXd &q_current,
                                              double dt); 

    bool isJointTargetReached(const Eigen::VectorXd &q_des, 
                              const Eigen::VectorXd &q_current, 
                              double tolerance = 0.02);

    bool isCartesianTargetReached(const KDL::Frame &des_frame, 
                                  double tolerance_pos = 0.02, 
                                  double tolerance_rot = 0.05);

private:
    std::shared_ptr<KDLRobot> robot_;
    ControlParams params_;
    
    Eigen::VectorXd last_cmd_vel_; 
};

#endif
