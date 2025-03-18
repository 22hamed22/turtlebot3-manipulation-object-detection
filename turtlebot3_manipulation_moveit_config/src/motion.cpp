#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
  // Initialize ROS node with default options
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({
      {"use_sim_time", rclcpp::ParameterValue(true)}  // Important for simulation time
  });

  // Create a logger instance
  rclcpp::Logger logger = rclcpp::get_logger("motion_u");

  // Set the logger level
  logger.set_level(rclcpp::Logger::Level::Warn); // Corrected here

  // Node name 'motion_u' is registered without explicit logger use
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("motion_u", "", node_options);

  // Declare kinematics parameters
  node->declare_parameter("robot_description_kinematics.arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
  node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_search_resolution", 0.1);
  node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_timeout", 0.1);
  node->declare_parameter("robot_description_kinematics.arm.position_only_ik", true);

  // Sleep for a while to let everything initialize
  rclcpp::sleep_for(std::chrono::seconds(2));  // 2 seconds for a more stable state

  // Initialize MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

  // Start state monitor!
  move_group.startStateMonitor();

  // Sleep to let state monitor receive the first joint states
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Print current end-effector pose (without logging)
  geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
  
  // Just the necessary code, no logging here
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.1;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.3;

  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, -M_PI / 2);  // 90 degree rotation around Z-axis
  target_pose.orientation = tf2::toMsg(tf2_quat);

  move_group.setPoseTarget(target_pose);

  // Plan to the target pose
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::MoveItErrorCode success = move_group.plan(my_plan);

  if (success == moveit::core::MoveItErrorCode::SUCCESS) {
    // Execute the plan
    move_group.execute(my_plan);
  } else {
    // Optionally, print the error directly
    std::cerr << "Planning failed with error code: " << success.val << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
