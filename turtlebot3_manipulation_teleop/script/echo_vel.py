#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import GripperCommandActionGoal

class CommandListener(Node):
    def __init__(self):
        super().__init__('command_listener_node')

        # Subscription to the arm controller topic
        self.arm_subscription = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.arm_listener_callback,
            10
        )

        # Subscription to the gripper controller topic
        self.gripper_subscription = self.create_subscription(
            GripperCommandActionGoal,
            '/gripper_controller/gripper_cmd',
            self.gripper_listener_callback,
            10
        )

    def arm_listener_callback(self, msg):
        # This will show the received arm trajectory commands
        self.get_logger().info(f"Received arm joint trajectory: {msg}")

    def gripper_listener_callback(self, msg):
        # This will show the received gripper commands
        self.get_logger().info(f"Received gripper command: {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
