#include "kdl_robot.h"

KDLRobot::KDLRobot(){}

KDLRobot::KDLRobot(KDL::Tree &robot_tree)
{
    createChain(robot_tree);
    initialize();
}

KDLRobot::KDLRobot(KDL::Tree &robot_tree, const std::string& base_link, const std::string& tool_link)
{
    if(!robot_tree.getChain(base_link, tool_link, chain_))
    {
        std::cout << "[ERROR] Failed to create KDL chain from " << base_link << " to " << tool_link << std::endl;
        n_ = 0;
        return;
    }
    std::cout << "[INFO] KDL robot model created from " << base_link << " to " << tool_link << std::endl;
    std::cout << "with " << chain_.getNrOfJoints() << " joints" << std::endl;
    std::cout << "and " << chain_.getNrOfSegments() << " segments" << std::endl;

    initialize();
}

void KDLRobot::initialize()
{
    n_ = chain_.getNrOfJoints();
    if (n_ == 0) return; 

    grav_ = KDL::JntArray(n_);
    s_J_ee_ = KDL::Jacobian(n_);
    b_J_ee_ = KDL::Jacobian(n_);
    s_J_dot_ee_ = KDL::Jacobian(n_);
    b_J_dot_ee_ = KDL::Jacobian(n_);
    s_J_ee_.data.setZero();
    b_J_ee_.data.setZero();
    s_J_dot_ee_.data.setZero();
    b_J_dot_ee_.data.setZero();
    jntArray_ = KDL::JntArray(n_);
    jntVel_ = KDL::JntArray(n_);
    coriol_ = KDL::JntArray(n_);
    dynParam_ = new KDL::ChainDynParam(chain_,KDL::Vector(0,0,-9.81));
    jacSol_ = new KDL::ChainJntToJacSolver(chain_);
    jntJacDotSol_ = new KDL::ChainJntToJacDotSolver(chain_);
    fkSol_ = new KDL::ChainFkSolverPos_recursive(chain_);
    fkVelSol_ = new KDL::ChainFkSolverVel_recursive(chain_);
    idSolver_ = new KDL::ChainIdSolver_RNE(chain_,KDL::Vector(0,0,-9.81));
    jsim_.resize(n_);
    grav_.resize(n_);
    q_min_.data.resize(n_);
    q_max_.data.resize(n_);
    
    ikVelSol_ = new KDL::ChainIkSolverVel_pinv(chain_); 
}

void KDLRobot::createChain(KDL::Tree &robot_tree)
{
    if(!robot_tree.getChain(robot_tree.getRootSegment()->first, 
        std::prev(std::prev(robot_tree.getSegments().end()))->first, chain_))
    {
        std::cout << "Failed to create KDL robot" << std::endl;
        return;
    }
}

void KDLRobot::getInverseKinematics(KDL::Frame &f, KDL::JntArray &q){
    if (n_ == 0) return;
    int ret = ikSol_->CartToJnt(jntArray_,f,q);
    if(ret != 0) {std::cout << ikSol_->strError(ret) << std::endl;};
}

void KDLRobot::setJntLimits(KDL::JntArray &q_low, KDL::JntArray &q_high)
{
    if (n_ == 0) return;
    q_min_ = q_low; q_max_ = q_high;
    ikSol_ = new KDL::ChainIkSolverPos_NR_JL(chain_,
                                             *fkSol_,
                                             *ikVelSol_,
                                             100,1e-6);
    ikSol_->setJointLimits(q_min_,q_max_);
}

void KDLRobot::update(std::vector<double> _jnt_values, std::vector<double> _jnt_vel)
{
    if (n_ == 0) return; 

    int err;
    updateJnts(_jnt_values, _jnt_vel);

    KDL::Twist s_T_f;
    KDL::Frame s_F_f;
    KDL::Jacobian s_J_f(n_);
    KDL::Jacobian s_J_dot_f(n_);
    KDL::FrameVel s_Fv_f;
    KDL::JntArrayVel jntVel(jntArray_,jntVel_);
    KDL::Twist s_J_dot_q_dot_f;

    err = dynParam_->JntToMass(jntArray_, jsim_); if(err != 0) {std::cout << strError(err);};
    err = dynParam_->JntToCoriolis(jntArray_, jntVel_, coriol_); if(err != 0) {std::cout << strError(err);};
    err = dynParam_->JntToGravity(jntArray_, grav_); if(err != 0) {std::cout << strError(err);};

    err = fkVelSol_->JntToCart(jntVel, s_Fv_f); if(err != 0) {std::cout << strError(err);};
    s_T_f = s_Fv_f.GetTwist();
    s_F_f = s_Fv_f.GetFrame();
    err = jacSol_->JntToJac(jntArray_, s_J_f); if(err != 0) {std::cout << strError(err);};
    err = jntJacDotSol_->JntToJacDot(jntVel, s_J_dot_q_dot_f); if(err != 0) {std::cout << strError(err);};
    err = jntJacDotSol_->JntToJacDot(jntVel, s_J_dot_f); if(err != 0) {std::cout << strError(err);};

    s_F_ee_ = s_F_f*f_F_ee_;
    KDL::Vector s_p_f_ee = s_F_ee_.p - s_F_f.p;
    KDL::changeRefPoint(s_J_f, s_p_f_ee, s_J_ee_);
    KDL::changeRefPoint(s_J_dot_f, s_p_f_ee, s_J_dot_ee_);
    KDL::changeBase(s_J_ee_, s_F_ee_.M.Inverse(), b_J_ee_);
    KDL::changeBase(s_J_dot_ee_, s_F_ee_.M.Inverse(), b_J_dot_ee_);
    s_V_ee_ = s_T_f.RefPoint(s_p_f_ee);
}

unsigned int KDLRobot::getNrJnts() { return n_; }
unsigned int KDLRobot::getNrSgmts() { return chain_.getNrOfSegments(); }

void KDLRobot::updateJnts(std::vector<double> _jnt_pos, std::vector<double> _jnt_vel)
{
    for (unsigned int i = 0; i < n_; i++)
    {
        jntArray_(i) = _jnt_pos[i];
        jntVel_(i) = _jnt_vel[i];
    }
}
Eigen::VectorXd KDLRobot::getJntValues() { return jntArray_.data; }
Eigen::VectorXd KDLRobot::getJntVelocities() { return jntVel_.data; }
Eigen::MatrixXd KDLRobot::getJntLimits()
{
    Eigen::MatrixXd jntLim; jntLim.resize(n_,2);
    jntLim.col(0) = q_min_.data; jntLim.col(1) = q_max_.data;
    return jntLim;
}
Eigen::MatrixXd KDLRobot::getJsim() { return jsim_.data; }
Eigen::VectorXd KDLRobot::getCoriolis() { return coriol_.data; }
Eigen::VectorXd KDLRobot::getGravity() { return grav_.data; }

Eigen::VectorXd KDLRobot::getID(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, const KDL::Wrenches &f_ext)
{
    if(n_==0) return Eigen::VectorXd();
    Eigen::VectorXd t; t.resize(n_);
    KDL::JntArray torques(n_);
    int r = idSolver_->CartToJnt(q,q_dot,q_dotdot,f_ext,torques);
    // std::cout << "idSolver result: " << idSolver_->strError(r) << std::endl;
    t = torques.data;
    return t;
}

KDL::Frame KDLRobot::getEEFrame() { return s_F_ee_; }
KDL::Twist KDLRobot::getEEVelocity() { return s_V_ee_; }
KDL::Twist KDLRobot::getEEBodyVelocity() { return s_V_ee_; }
KDL::Jacobian KDLRobot::getEEJacobian() { return s_J_ee_; }
KDL::Jacobian KDLRobot::getEEBodyJacobian() { return b_J_ee_; }
Eigen::VectorXd KDLRobot::getEEJacDotqDot() { return s_J_dot_ee_.data; }

void KDLRobot::addEE(const KDL::Frame &_f_F_ee)
{
    f_F_ee_ = _f_F_ee;
    if (n_ > 0) this->update(toStdVector(this->jntArray_.data), toStdVector(this->jntVel_.data));
}

std::string KDLRobot::strError(const int error) {
  switch(error) {
  case KDL::SolverI::E_NOERROR: return "No error \n"; break;
  case KDL::SolverI::E_NO_CONVERGE: return "[ERROR] Failed to converge \n"; break;
  case KDL::SolverI::E_UNDEFINED: return "[ERROR] Undefined value \n"; break;
  case KDL::SolverI::E_DEGRADED: return "[ERROR] Converged but degraded solution \n"; break;
  case KDL::SolverI::E_NOT_UP_TO_DATE: return "[ERROR] Internal data structures not up to date with Chain \n"; break;
  case KDL::SolverI::E_SIZE_MISMATCH: return "[ERROR] The size of the input does not match the internal state \n"; break;
  case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED: return "[ERROR] The maximum number of iterations is exceeded \n"; break;
  case KDL::SolverI::E_OUT_OF_RANGE: return "[ERROR] The requested index is out of range \n"; break;
  case KDL::SolverI::E_NOT_IMPLEMENTED: return "[ERROR] The requested function is not yet implemented \n"; break;
  case KDL::SolverI::E_SVD_FAILED: return "[ERROR] SVD failed \n"; break;
  default: return "UNKNOWN ERROR";
  }
}
