#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image,
            '/pi_camera/image_raw',  # Change topic name if needed
            self.image_callback,
            10)

        # Publisher to announce detected object color
        self.publisher = self.create_publisher(String, 'detected_object', 10)

        # OpenCV Bridge
        self.bridge = CvBridge()
        self.get_logger().info("Object Detection Node Started")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert image to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color ranges for detection
        green_lower = np.array([40, 50, 50])
        green_upper = np.array([80, 255, 255])

        blue_lower = np.array([100, 50, 50])
        blue_upper = np.array([140, 255, 255])

        # Create masks
        mask_green = cv2.inRange(hsv, green_lower, green_upper)
        mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)

        # Find contours
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_color = None

        if contours_green:
            detected_color = "green"
        elif contours_blue:
            detected_color = "blue"

        if detected_color:
            self.get_logger().info(f"Detected Object: {detected_color}")
            self.publisher.publish(String(data=detected_color))

        # Optional: Show images for debugging
        cv2.imshow("Camera View", cv_image)
        cv2.imshow("Green Mask", mask_green)
        cv2.imshow("Blue Mask", mask_blue)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
