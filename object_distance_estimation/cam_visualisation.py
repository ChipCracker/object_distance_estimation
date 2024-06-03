#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CamVisualisation(Node):
    def __init__(self):
        super().__init__('cam_visualisation')
        self.get_logger().info("CamVisualisation node started")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Display the image
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)  # Display the image for 1 ms
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    cam_visualisation = CamVisualisation()
    rclpy.spin(cam_visualisation)

    # Clean up OpenCV resources and destroy the node
    cv2.destroyAllWindows()
    cam_visualisation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

