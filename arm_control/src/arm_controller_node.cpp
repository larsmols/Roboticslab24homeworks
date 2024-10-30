#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode() : Node("arm_controller_node") {
        // Subscriber to read joint states
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "gui_joint_states", 10,
            std::bind(&ArmControllerNode::jointStateCallback, this, std::placeholders::_1));
        
        // Publisher to send joint position commands
        position_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);

        RCLCPP_INFO(this->get_logger(), "Arm Controller Node has been started.");
    }

private:
    // Callback function to print joint positions and send commands
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Current positions:");
        for (size_t i = 0; i < msg->position.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Joint %ld: %f", i, msg->position[i]);
        }

        // Example command to set positions (this can be modified as needed)
        std_msgs::msg::Float64MultiArray command_msg;
        command_msg.data = msg->position; // Adjust these values as desired
        position_command_publisher_->publish(command_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmControllerNode>());
    rclcpp::shutdown();
    return 0;
}
