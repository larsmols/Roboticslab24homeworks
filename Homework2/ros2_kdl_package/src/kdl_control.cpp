#include "kdl_control.h"
#include "utils.h"


KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}


Eigen::VectorXd KDLController::PD_Plus(KDL::JntArray &_qd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();
 
    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    // Eigen::VectorXd de = _dqd.data - dq;
 
    return  -_Kd*dq +_Kp*e + robot_->getGravity();
}


/*
Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    // === Gain Matrices ===
    Eigen::Matrix<double, 6, 6> Kp = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> Kd = Eigen::Matrix<double, 6, 6>::Zero();
    Kp.block<3, 3>(0, 0) = _Kpp * Eigen::Matrix3d::Identity(); // Position proportional gain
    Kp.block<3, 3>(3, 3) = _Kpo * Eigen::Matrix3d::Identity(); // Orientation proportional gain
    Kd.block<3, 3>(0, 0) = _Kdp * Eigen::Matrix3d::Identity(); // Position derivative gain
    Kd.block<3, 3>(3, 3) = _Kdo * Eigen::Matrix3d::Identity(); // Orientation derivative gain

    // === Read Current State ===
    KDL::Jacobian JEE = robot_->getEEJacobian();
    Eigen::Matrix<double, 6, Eigen::Dynamic> J = toEigen(JEE); // Converts KDL::Jacobian to Eigen
    Eigen::Matrix<double, 7, 7> M = robot_->getJsim(); // Joint-space inertia matrix
    Eigen::Matrix<double, 7, 6> Jpinv = weightedPseudoInverse(M, J); // Dynamically consistent pseudo-inverse

    // End-effector pose and velocity
    KDL::Frame currPose = robot_->getEEFrame();
    KDL::Twist currTwist = robot_->getEEVelocity();

    // === Position and Orientation Errors ===
    Eigen::Vector3d p_d(_desPos.p.data);  // Desired position
    Eigen::Vector3d p_e(currPose.p.data); // Current position
    Eigen::Vector3d e_p = computeLinearError(p_d, p_e); // Linear position error

    Eigen::Matrix3d R_d(_desPos.M.data); // Desired orientation (rotation matrix)
    Eigen::Matrix3d R_e(currPose.M.data); // Current orientation (rotation matrix)
    Eigen::Vector3d e_o = computeOrientationError(R_d, R_e); // Orientation error

    // === Velocity Errors ===
    Eigen::Vector3d dot_p_d(_desVel.vel.data); // Desired linear velocity
    Eigen::Vector3d dot_p_e(currTwist.vel.data); // Current linear velocity
    Eigen::Vector3d dot_e_p = computeLinearError(dot_p_d, dot_p_e); // Linear velocity error

    Eigen::Vector3d omega_d(_desVel.rot.data); // Desired angular velocity
    Eigen::Vector3d omega_e(currTwist.rot.data); // Current angular velocity
    Eigen::Vector3d dot_e_o = computeOrientationVelocityError(omega_d, omega_e, R_d, R_e); // Angular velocity error

    // Concatenate errors
    Eigen::Matrix<double, 6, 1> x_tilde;
    x_tilde << e_p, e_o;

    Eigen::Matrix<double, 6, 1> dot_x_tilde;
    dot_x_tilde << dot_e_p, dot_e_o;

    // === Desired Accelerations ===
    Eigen::Matrix<double, 6, 1> xdd_des;
    xdd_des << Eigen::Vector3d(_desAcc.vel.data), Eigen::Vector3d(_desAcc.rot.data);

    // === Compute Control Output ===
    Eigen::Matrix<double, 6, 1> y = xdd_des 
                                  + Kd * dot_x_tilde 
                                  + Kp * x_tilde 
                                  - robot_->getEEJacDotqDot() * robot_->getJntVelocities();

    // === Null-Space Control ===
    double cost;
    Eigen::VectorXd grad = gradientJointLimits(robot_->getJntValues(), robot_->getJntLimits(), cost);

    Eigen::Matrix<double, 7, 1> null_space_ctrl = -grad; // Placeholder: add your own weighting if needed

    // === Compute Torques ===
    Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
    Eigen::VectorXd tau = M * (Jpinv * y + (I - Jpinv * J) * null_space_ctrl) 
                          + robot_->getCoriolis() 
                          + robot_->getGravity();

    return tau;
}
*/
