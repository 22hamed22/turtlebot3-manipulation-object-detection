#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import GripperCommand  # Corrected import

class EchoVelocityNode(Node):
    def __init__(self):
        super().__init__('echo_velocity_node')

        # Subscribe to the arm trajectory topic
        self.arm_subscription = self.create_subscription(
            JointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            self.arm_listener_callback,
            10
        )

        # Subscribe to the gripper command topic
        self.gripper_subscription = self.create_subscription(
            GripperCommand,  # Corrected message type
            '/gripper_controller/gripper_cmd',
            self.gripper_listener_callback,
            10
        )

        # Log when the node is initialized
        self.get_logger().info("EchoVelocityNode initialized and waiting for messages.")

    def arm_listener_callback(self, msg):
        self.get_logger().info("Received a JointTrajectory message from arm controller.")
        for point in msg.points:
            if point.velocity != 0.0:
                self.get_logger().info(f"Arm Non-zero velocity detected: {point.velocity}")
            else:
                self.get_logger().info("Arm Zero velocity.")

    def gripper_listener_callback(self, msg):
        self.get_logger().info("Received a GripperCommand message from gripper controller.")
        # Log gripper command details
        self.get_logger().info(f"Gripper Command: Position: {msg.position}, Effort: {msg.effort}")

def main(args=None):
    rclpy.init(args=args)
    node = EchoVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
