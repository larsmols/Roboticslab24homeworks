#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/frames.hpp"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class VisionControlNode : public rclcpp::Node
{
public:
    VisionControlNode()
    : Node("ros2_kdl_vision_control"),
      node_ptr_(std::shared_ptr<VisionControlNode>(this))
    {
        // Initialize command interface parameter with default "velocity"
        declare_parameter<std::string>("cmd_interface", "velocity");
        get_parameter("cmd_interface", cmd_interface_);
    
        RCLCPP_INFO(get_logger(), "Current cmd interface is: '%s'", cmd_interface_.c_str());

        if (!(cmd_interface_ == "velocity" || cmd_interface_ == "effort" || cmd_interface_ == "effort_cartesian"))
        {
            RCLCPP_INFO(get_logger(), "Selected cmd interface is invalid!"); 
            return;
        }

        // Initialize task parameter with default "positioning"
        declare_parameter<std::string>("task", "positioning");
        get_parameter("task", task_);
        RCLCPP_INFO(get_logger(), "Current task is: '%s'", task_.c_str());

        if (!(task_ == "positioning" || task_ == "look-at-point"))
        {
            RCLCPP_INFO(get_logger(), "Selected task is invalid!"); 
            return;
        }

        iter_count_ = 0;
        current_time_ = 0;
        has_joint_state_ = false;
        has_aruco_data_ = false;

        // Subscribe to ARUCO pose messages
        aruco_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10, std::bind(&VisionControlNode::handle_aruco_pose, this, std::placeholders::_1));

        // Wait until ARUCO data is received
        while(!has_aruco_data_){
            RCLCPP_INFO(this->get_logger(), "Awaiting ARUCO data...");
            rclcpp::spin_some(node_ptr_);
        }

        // Fetch robot_description parameter from robot_state_publisher
        auto params_client = std::make_shared<rclcpp::SyncParametersClient>(node_ptr_, "robot_state_publisher");
        while (!params_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Shutting down.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "Service unavailable, retrying...");
        }
        auto robot_desc_param = params_client->get_parameters({"robot_description"});

        // Build KDL tree from robot description
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(robot_desc_param[0].value_to_string(), robot_tree)){
            std::cout << "Unable to retrieve robot_description parameter!";
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);  
        
        // Set joint limits (TODO: Extract from URDF)
        unsigned int num_joints = robot_->getNrJnts();
        KDL::JntArray q_min(num_joints), q_max(num_joints);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96;
        q_max.data << 2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96;
        robot_->setJntLimits(q_min, q_max);            

        // Initialize joint state arrays
        joint_positions_.resize(num_joints);
        joint_velocities_.resize(num_joints);
        qd.resize(num_joints);
        dqd.resize(num_joints);
        qdi.resize(num_joints);
        joint_acceleration_desired_.resize(num_joints);
        previous_joint_velocities_.resize(num_joints);
        torque_cmds_.resize(num_joints);
        q_desired.resize(num_joints);
        dq_desired.resize(num_joints);

        // Subscribe to joint state messages
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&VisionControlNode::handle_joint_states, this, std::placeholders::_1));

        // Wait until joint states are received
        while(!has_joint_state_){
            RCLCPP_INFO(this->get_logger(), "Awaiting joint states...");
            rclcpp::spin_some(node_ptr_);
        }

        // Update robot with current joint positions and velocities
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Refresh robot state
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Get initial end-effector pose
        initial_frame_ = robot_->getEEFrame();
        Eigen::Vector3d initial_pos = toEigen(initial_frame_.p);

        KDL::Chain kdl_chain = robot_->getChain();
        fk_solver_ = new KDL::ChainFkSolverPos_recursive(kdl_chain);

        // Initialize controller
        KDLController controller_obj(*robot_);

        // Calculate current Jacobian
        KDL::Jacobian jacobian_cam = robot_->getEEJacobian();

        // Define object frame relative to base with positional and rotational offset
        KDL::Frame object_in_base(marker.M * KDL::Rotation::RotY(-1.57),
                                  KDL::Vector(marker.p.data[0] + 0.03, marker.p.data[1], marker.p.data[2] - 0.24));
        base_to_object_ = robot_->getEEFrame() * object_in_base;

        // Apply positional offset
        base_to_object_.p = base_to_object_.p; // + KDL::Vector(0.2, 0.04, p_offset);
        base_to_object_.M = base_to_object_.M;

        Eigen::Vector3d target_position;

        if(task_ == "positioning"){
            target_position = toEigen(base_to_object_.p);
        }
        else{
            target_position << initial_pos[0], -initial_pos[1], initial_pos[2];
        }

        double trajectory_time = 1.5, acceleration_time = 0.5, time_elapsed = 0.0;
        
        planner_ = KDLPlanner(trajectory_time, acceleration_time, initial_pos, target_position);

        trajectory_point traj_point = planner_.compute_trajectory(time_elapsed);

        // Compute positional error
        Eigen::Vector3d position_error = computeLinearError(traj_point.pos, initial_pos);

        // Perform inverse kinematics
        robot_->getInverseKinematics(initial_frame_, qdi);

        if(task_ == "positioning" && has_aruco_data_ && has_joint_state_ ){
            KDL::Frame aruco_world = robot_->getEEFrame() * marker;
            std::cout << "ARUCO Position: " << aruco_world.p << std::endl;
            std::cout << "ARUCO Orientation: " << aruco_world.M << std::endl;
            KDL::Vector nominal_position(1.30, -0.35, 0.62);
            // KDL::Vector3d nominal_orientation = (1.57, 0.01, 2.16);
            std::cout << "ARUCO Position Error: " << aruco_world.p - nominal_position << std::endl;

            // Initialize command publisher for velocity
            cmd_publisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&VisionControlNode::publish_commands, this));
        
            // Initialize desired commands with zero velocities
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = 0.0;
            }

        }
        else if(task_ == "look-at-point" && has_aruco_data_ && has_joint_state_ ){
            
            if(cmd_interface_ == "velocity"){
                // Initialize command publisher for velocity
                cmd_publisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&VisionControlNode::publish_commands, this));
                
                // Set desired joint velocities
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
                
            }
            else if(cmd_interface_ == "effort" || cmd_interface_ == "effort_cartesian"){
                // Initialize command publisher for effort
                cmd_publisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                            std::bind(&VisionControlNode::publish_commands, this));
                
                // Initialize desired commands with zero effort
                for (long int i = 0; i < num_joints; ++i) {
                    desired_commands_[i] = 0;
                }
    
            }
            else{
                std::cout << "Invalid command interface!" << std::endl;
            }
        }

        // Publish initial command
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmd_publisher_->publish(cmd_msg);

        RCLCPP_INFO(this->get_logger(), "Initiating trajectory execution...");
    }

private:

    // Variables to store previous ARUCO marker position
    Eigen::Vector3d previous_marker_pos_ = Eigen::Vector3d::Zero();  // Initialize previous marker position
    double marker_speed_threshold_ = 0.001;  // Threshold for ARUCO marker movement

    // Function to determine if ARUCO marker is in motion
    bool is_marker_moving(const Eigen::Vector3d& current_pos) {
        Eigen::Vector3d position_delta = current_pos - previous_marker_pos_;
        double speed = position_delta.norm();
        // Update previous position
        previous_marker_pos_ = current_pos;
        // Check if speed exceeds threshold
        return speed > marker_speed_threshold_;
    }
    
    void publish_commands() {
        iter_count_ += 1;

        // Define trajectory parameters
        double total_duration = 1.5; 
        int trajectory_steps = 150; 
        int loop_frequency = trajectory_steps / total_duration;
        double delta_t = 1.0 / loop_frequency;
        current_time_ += delta_t;
        Eigen::Vector3d desired_dir;
        desired_dir << 0, 0, 1;
        double gain = -10;

        // Get current end-effector frame
        KDL::Frame current_cart_pos = robot_->getEEFrame();           

        // Define desired frame
        KDL::Frame desired_frame; 
        desired_frame.M = base_to_object_.M; 
        desired_frame.p = base_to_object_.p;

        KDL::Frame cam_object_frame(marker.M, marker.p); 

        // Calculate current Jacobian
        KDL::Jacobian jacobian_cam = robot_->getEEJacobian();
        
        // Calculate transformation matrices
        Eigen::Matrix<double,3,1> cam_pos = toEigen(cam_object_frame.p);
        Eigen::Matrix<double,3,1> s_vector = cam_pos / cam_pos.norm();
        Eigen::Matrix<double,3,3> rotation_matrix = toEigen(robot_->getEEFrame().M);
        Eigen::Matrix<double,3,3> linear_block = (-1 / cam_pos.norm()) * (Eigen::Matrix<double,3,3>::Identity() - s_vector * s_vector.transpose());
        Eigen::Matrix<double,3,6> linear_matrix = Eigen::Matrix<double,3,6>::Zero();
        Eigen::Matrix<double,6,6> rotation_big = Eigen::Matrix<double,6,6>::Zero(); 
        rotation_big.block(0,0,3,3) = rotation_matrix;        
        rotation_big.block(3,3,3,3) = rotation_matrix;
        linear_matrix.block(0,0,3,3) = linear_block;        
        linear_matrix.block(0,3,3,3) = skew(s_vector);
        linear_matrix = linear_matrix * rotation_big;

        // Calculate nullspace projector
        Eigen::MatrixXd LJ = linear_matrix * (jacobian_cam.data);
        Eigen::MatrixXd LJ_pseudo_inv = LJ.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd null_space = Eigen::Matrix<double,7,7>::Identity() - (LJ_pseudo_inv * LJ);

        // Compute positional and orientation errors
        Eigen::Vector3d pos_error = computeLinearError(Eigen::Vector3d(base_to_object_.p.data), Eigen::Vector3d(current_cart_pos.p.data));
        Eigen::Vector3d orient_error = computeOrientationError(toEigen(current_cart_pos.M), toEigen(base_to_object_.M));
        std::cout << "Position error norm: " << pos_error.norm() << std::endl;

        // Compute error magnitude
        double error_magnitude = pos_error.norm();
        
        // Define error threshold
        double error_thresh = 0.05;  // 5 cm

        KDLController controller_obj(*robot_);

        if(task_ == "positioning"){

            if (current_time_ < total_duration){
                // Get current trajectory point
                trajectory_point traj_pt = planner_.compute_trajectory(current_time_);

                // Calculate differential IK
                Vector6d cart_vel; 
                cart_vel << 0.05 * traj_pt.vel + 5 * pos_error, 0.1 * orient_error;
                joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data) * cart_vel;
                joint_positions_.data += joint_velocities_.data * delta_t;

                // Update robot state
                robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data)); 
                  
                // Assign joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }

                // Publish velocity commands
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmd_publisher_->publish(cmd_msg);
            }
            else{
                // Stop movement by setting velocities to zero
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                    std::cout << "Stopping joint velocities: " << joint_velocities_.data << std::endl;
                }

                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmd_publisher_->publish(cmd_msg);
            }
        }
        else if(task_ == "look-at-point"){

            // Add logic to check if ARUCO is moving
            if (is_marker_moving(toEigen(cam_object_frame.p))) {
                
                // Execute trajectory only if ARUCO is moving
                trajectory_point traj_pt = planner_.compute_trajectory(current_time_);
                
                if(cmd_interface_ == "velocity"){
                    
                    dqd.data = gain * LJ_pseudo_inv * desired_dir - null_space * (-qdi.data + joint_positions_.data);
                
                }
                else if(cmd_interface_ == "effort" || cmd_interface_ == "effort_cartesian"){
                    
                    dqd.data = gain * LJ_pseudo_inv * desired_dir - null_space * (-qdi.data + joint_positions_.data);

                    qd.data = qdi.data + dqd.data * delta_t;

                    double cos_theta = s_vector.dot(desired_dir); // Dot product between vectors
                    cos_theta = std::max(-1.0, std::min(1.0, cos_theta)); // Clamp to [-1, 1]
                    // double angular_error = std::acos(cos_theta);
                    double cam_orient_error = std::acos(cos_theta);
                    Eigen::Vector3d orientation_error = cam_orient_error * s_vector;
                    
                    fk_solver_->JntToCart(qd, fd);

                    previous_joint_velocities_.data = joint_velocities_.data;

                    Vector6d cart_vel; cart_vel << traj_pt.vel + pos_error, orientation_error;
                    // Update desired joint velocities using Jacobian pseudoinverse
                    dq_desired.data = pseudoinverse(robot_->getEEJacobian().data) * cart_vel;

                    // Update desired joint positions
                    q_desired.data = joint_positions_.data + dqd.data * delta_t;

                    // Compute joint acceleration
                    joint_acceleration_desired_.data = (joint_velocities_.data - previous_joint_velocities_.data) / delta_t;
                    
                    // Use inverse dynamics controller to calculate torques
                    torque_cmds_ = controller_obj.idCntr(q_desired, dqd, joint_acceleration_desired_, _Kp, _Kd);
                }
                else if(cmd_interface_ == "effort_cartesian"){
                    
                    Vector6d cart_acc; 
                    cart_acc << traj_pt.acc + pos_error / delta_t, 0, 0, 0;
                    desVel = KDL::Twist(KDL::Vector(traj_pt.vel[0], traj_pt.vel[1], traj_pt.vel[2]), KDL::Vector::Zero());
                    desAcc = KDL::Twist(KDL::Vector(traj_pt.acc[0], traj_pt.acc[1], traj_pt.acc[2]), KDL::Vector::Zero());
                    desPos.M = desired_frame.M;
                    desPos.p = desired_frame.p; // Possibly multiply by Re
                        
                    // Use inverse dynamics controller to calculate torques
                    torque_cmds_ = controller_obj.idCntr(desPos, desVel, desAcc, _Kpp, _Kpo, _Kdp, _Kdo);

                }
                else{
                    std::cout << "Invalid command interface!" << std::endl;
                }

                // Update robot state
                robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));  

                if(cmd_interface_ == "velocity"){
                   
                    // Assign joint velocity commands
                    for (long int i = 0; i < dqd.data.size(); ++i){
                        desired_commands_[i] = dqd(i);
                    }

                }
                else if(cmd_interface_ == "effort" || cmd_interface_ == "effort_cartesian"){
                    
                    // Assign torque commands
                    for (long int i = 0; i < torque_cmds_.size(); ++i) {
                        desired_commands_[i] = torque_cmds_(i);
                    }

                }
                else{
                    std::cout << "Invalid command interface!" << std::endl;
                }

                // Publish command messages
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmd_publisher_->publish(cmd_msg);


            }
            else{

                if(cmd_interface_ == "effort" || cmd_interface_ == "effort_cartesian" ){
                    
                    KDLController controller_obj_effort(*robot_);
                    q_desired.data = joint_positions_.data;
                    // Zero out desired joint velocities
                    dq_desired.data = Eigen::VectorXd::Zero(7,1);
                    // Zero out desired joint accelerations
                    joint_acceleration_desired_.data = Eigen::VectorXd::Zero(7,1);

                    torque_cmds_ = controller_obj_effort.idCntr(q_desired, dq_desired, joint_acceleration_desired_, _Kp, _Kd);
                    
                    // Update robot state
                    robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));  
                    
                    for (long int i = 0; i < torque_cmds_.size(); ++i) {
                        desired_commands_[i] = torque_cmds_(i);
                        // std::cout << "Torque commands: " << torque_cmds_ << std::endl;
                    }
                }
                else{
                     // Set joint velocities to zero
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                        std::cout << "Setting joint velocity to zero: " << joint_velocities_.data << std::endl;
                    }
                }

                // Create and send command message
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmd_publisher_->publish(cmd_msg);
            }
        
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Unrecognized task: %s", task_.c_str());
            return;
        }
    }

    void handle_aruco_pose(const geometry_msgs::msg::PoseStamped& pose_msg){ 

        has_aruco_data_ = true;
        double x, y, z, qx, qy, qz, qw;
        x = pose_msg.pose.position.x;
        y = pose_msg.pose.position.y;
        z = pose_msg.pose.position.z;
        qx = pose_msg.pose.orientation.x;
        qy = pose_msg.pose.orientation.y;
        qz = pose_msg.pose.orientation.z;
        qw = pose_msg.pose.orientation.w;
        KDL::Rotation rotation = KDL::Rotation::Quaternion(qx, qy, qz, qw);
        KDL::Vector translation(x, y, z);

        marker.p = translation;
        marker.M = rotation;
    }

    void handle_joint_states(const sensor_msgs::msg::JointState& sensor_msg){

        has_joint_state_ = true;
        for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }

    // Subscribers and publishers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr sub_timer_;
    rclcpp::Node::SharedPtr node_ptr_;

    // Command variables
    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray qd;
    KDL::JntArray dqd;
    KDL::JntArray qdi;
    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;
    std::string task_, cmd_interface_;
    KDL::Frame marker;
    KDL::Frame initial_frame_;
    KDL::Frame fd;
    KDL::Frame base_to_object_;
    KDL::JntArray joint_acceleration_desired_;
    KDL::ChainFkSolverPos_recursive* fk_solver_;
    KDL::JntArray previous_joint_velocities_;
    Eigen::VectorXd torque_cmds_;
    KDL::JntArray q_desired;
    KDL::JntArray dq_desired;
    KDL::Twist desVel;
    KDL::Twist desAcc;
    KDL::Frame desPos;
    
    // Controller gains
    double _Kp = 100;  // Proportional gain
    double _Kd = 43;   // Derivative gain
    double _Kpp = 85;
    double _Kpo = 85;
    double _Kdp = 2 * sqrt(_Kpp);
    double _Kdo = 2 * sqrt(_Kpo);
    double current_time_;
    int iter_count_;
    bool has_joint_state_;
    bool has_aruco_data_;

};

// Main function to initialize and spin the node
int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionControlNode>());
    rclcpp::shutdown();
    return 0;
}
