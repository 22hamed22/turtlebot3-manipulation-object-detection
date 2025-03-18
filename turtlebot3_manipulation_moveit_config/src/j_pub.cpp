#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>

class MotionPlanNode : public rclcpp::Node
{
public:
    MotionPlanNode() : Node("motion_plan")  // Changed node name to motion_plan
    {   
        this->declare_parameter("robot_description_kinematics.arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
        this->declare_parameter("robot_description_kinematics.arm.kinematics_solver_search_resolution", 0.1);
        this->declare_parameter("robot_description_kinematics.arm.kinematics_solver_timeout", 0.1);
        this->declare_parameter("robot_description_kinematics.arm.position_only_ik", true);
        
        RCLCPP_INFO(this->get_logger(), "MotionPlanNode initialized.");
    }

    void initializeMoveGroup()
    {
        // Initialize MoveGroupInterface for the "arm" group
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");

        // Subscribe to joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MotionPlanNode::jointStateCallback, this, std::placeholders::_1));
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Received joint states, set them as the target for move group
        RCLCPP_INFO(this->get_logger(), "Received joint states: %zu", msg->position.size());

        // Set the joint target based on the received joint states
        move_group_->setJointValueTarget(msg->position);
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create a shared pointer to the MotionPlanNode
    auto node = std::make_shared<MotionPlanNode>();

    // Initialize MoveGroupInterface after node construction
    node->initializeMoveGroup();

    // Spin the node to process callbacks
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
