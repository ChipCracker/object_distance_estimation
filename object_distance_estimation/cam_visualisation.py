#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CamVisualisation(Node):

    def __init__(self):
        super().__init__('cam_visualisation')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        try:
            # Convert compressed image data to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is None:
                self.get_logger().error('Failed to decode image')
                return
            
            # Display the image
            cv2.imshow("Camera Image", image)
            cv2.waitKey(1)  # Display the image for 1 ms
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    cam_visualisation = CamVisualisation()
    rclpy.spin(cam_visualisation)

    # Destroy the node explicitly (optional - done automatically when garbage collected)
    cam_visualisation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
