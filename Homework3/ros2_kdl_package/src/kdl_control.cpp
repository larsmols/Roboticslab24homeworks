#include "kdl_control.h"

// Constructor for KDLController, initializes the robot pointer
KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

// Inverse Dynamics Controller for Joint Arrays
Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double Kp_gain, double Kd_gain)
{
    // Obtain current joint positions and velocities
    Eigen::VectorXd current_q = robot_->getJntValues();
    Eigen::VectorXd current_dq = robot_->getJntVelocities();

    // Compute position and velocity errors
    Eigen::VectorXd position_error = _qd.data - current_q;
    Eigen::VectorXd velocity_error = _dqd.data - current_dq;

    Eigen::VectorXd desired_ddq = _ddqd.data;

    // Calculate torque commands using inverse dynamics
    return robot_->getJsim() * (desired_ddq + Kd_gain * velocity_error + Kp_gain * position_error)
           + robot_->getCoriolis();
}

// Inverse Dynamics Controller for Operational Space (End-Effector Control)
Eigen::VectorXd KDLController::idCntr(
    KDL::Frame &_desiredPos,
    KDL::Twist &_desiredVel,
    KDL::Twist &_desiredAcc,
    double Kp_linear, double Kp_angular,
    double Kd_linear, double Kd_angular)
{
    // Initialize gain matrices as zero
    Eigen::Matrix<double, 6, 6> Kp_matrix = Eigen::MatrixXd::Zero(6, 6);
    Eigen::Matrix<double, 6, 6> Kd_matrix = Eigen::MatrixXd::Zero(6, 6);

    // Set linear proportional gains
    Kp_matrix.block(0, 0, 3, 3) = Kp_linear * Eigen::Matrix3d::Identity();
    // Set angular proportional gains
    Kp_matrix.block(3, 3, 3, 3) = Kp_angular * Eigen::Matrix3d::Identity();
    // Set linear derivative gains
    Kd_matrix.block(0, 0, 3, 3) = Kd_linear * Eigen::Matrix3d::Identity();
    // Set angular derivative gains
    Kd_matrix.block(3, 3, 3, 3) = Kd_angular * Eigen::Matrix3d::Identity();

    // Retrieve the current Jacobian of the end-effector
    KDL::Jacobian jacobian_ee = robot_->getEEJacobian();

    // Identity matrix for nullspace projection
    Eigen::Matrix<double, 7, 7> identity_matrix = Eigen::Matrix<double, 7, 7>::Identity();

    // Inertia matrix of the robot
    Eigen::Matrix<double, 7, 7> inertia_matrix = robot_->getJsim();

    // Compute the pseudoinverse of the Jacobian
    Eigen::Matrix<double, 7, 6> jacobian_pseudo_inv = pseudoinverse(robot_->getEEJacobian().data);

    // Current and desired positions
    KDL::Frame current_cart_pose = robot_->getEEFrame();
    Eigen::Vector3d desired_position(_desiredPos.p.data);
    Eigen::Vector3d current_position(current_cart_pose.p.data);

    // Desired and current rotation matrices
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> desired_rot(_desiredPos.M.data);
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> current_rot(current_cart_pose.M.data);
    desired_rot = matrixOrthonormalization(desired_rot);
    current_rot = matrixOrthonormalization(current_rot);

    // Current and desired velocities
    KDL::Twist current_twist = robot_->getEEVelocity();
    Eigen::Vector3d desired_linear_vel(_desiredVel.vel.data);
    Eigen::Vector3d current_linear_vel(current_twist.vel.data);
    Eigen::Vector3d desired_angular_vel(_desiredVel.rot.data);
    Eigen::Vector3d current_angular_vel(current_twist.rot.data);

    // Desired accelerations
    Eigen::Matrix<double, 6, 1> desired_accel;
    Eigen::Vector3d desired_linear_accel(_desiredAcc.vel.data);
    Eigen::Vector3d desired_angular_accel(_desiredAcc.rot.data);
    desired_accel << desired_linear_accel, desired_angular_accel;

    // Calculate linear and angular position errors
    Eigen::Vector3d pos_error = computeLinearError(desired_position, current_position);
    Eigen::Vector3d linear_vel_error = computeLinearError(desired_linear_vel, current_linear_vel);
    Eigen::Vector3d orient_error = computeOrientationError(desired_rot, current_rot);
    Eigen::Vector3d angular_vel_error = computeOrientationVelocityError(desired_angular_vel, current_angular_vel, desired_rot, current_rot);

    // Combine position and orientation errors into a single state vector
    Eigen::Matrix<double, 6, 1> error_state;  
    error_state << pos_error, orient_error;

    // Combine velocity errors into a single state vector
    Eigen::Matrix<double, 6, 1> velocity_error_state;
    velocity_error_state << linear_vel_error, angular_vel_error; 

    // Desired acceleration vector
    Eigen::Matrix<double, 6, 1> desired_acceleration = desired_accel;

    // Compute the operational space control term
    Eigen::Matrix<double, 6, 1> control_term = desired_acceleration 
                                               - robot_->getEEJacDot().data * robot_->getJntVelocities()
                                               + Kd_matrix * velocity_error_state 
                                               + Kp_matrix * error_state;

    // Compute and return the torque commands
    return inertia_matrix * (jacobian_pseudo_inv * control_term) + robot_->getCoriolis();
}
