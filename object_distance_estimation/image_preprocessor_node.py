#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
import pickle
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

class ImagePreProcessor(Node):
    def __init__(self):
        super().__init__('image_preprocessor')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.image_publisher = self.create_publisher(
            Image,
            '/image/undistorted',
            10)

        print(os.listdir())
        # Load camera calibration parameters
        with open('~/ros_ws/src/object_distance_estimation/object_distance_estimation/calibration.pkl', 'rb') as f:
            self.camera_matrix, self.dist_coeffs = pickle.load(f)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Undistort the image
            undistorted_img = cv.undistort(current_frame, self.camera_matrix, self.dist_coeffs)

            # Convert OpenCV image back to ROS message and publish
            undistorted_msg = self.bridge.cv2_to_imgmsg(undistorted_img, "bgr8")
            self.image_publisher.publish(undistorted_msg)

            # Optionally display the image
            #cv.imshow("Undistorted Image", undistorted_img)
            #cv.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImagePreProcessor()
    rclpy.spin(image_processor)

    # Clean up OpenCV resources and destroy the node
    cv.destroyAllWindows()
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
