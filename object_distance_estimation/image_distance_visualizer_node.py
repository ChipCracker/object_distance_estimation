import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt

class ImageDistanceVisualizer(Node):
    def __init__(self):
        super().__init__('image_distance_visualizer')
        self.bridge = CvBridge()
        self.current_image = None
        self.current_distances = None
        
        # Subscribe to the image and distance data topics
        self.image_sub = self.create_subscription(
            Image, '/image/undistorted', self.image_callback, 10)
        self.distance_sub = self.create_subscription(
            Float32MultiArray, '/lidar_pixel_distances', self.distance_callback, 10)
        
        # Initialize the colormap (you can choose any other like 'viridis', 'plasma', etc.)
        self.colormap = plt.get_cmap('jet')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            self.display_image_with_distances()
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

    def distance_callback(self, msg):
        # Process the incoming distance data
        distance_data = np.array(msg.data, dtype=np.float32).reshape(-1, 3)
        self.current_distances = distance_data
        self.display_image_with_distances()

    def display_image_with_distances(self):
        if self.current_image is not None and self.current_distances is not None:
            # Get maximum distance for normalization
            max_distance = np.max(self.current_distances[:, 2]) if self.current_distances[:, 2].size > 0 else 1
            
            # Overlay color on the image based on distance
            for point in self.current_distances:
                x, y, distance = int(point[0]), int(point[1]), point[2]
                if 0 <= x < self.current_image.shape[1] and 0 <= y < self.current_image.shape[0]:
                    # Normalize the distance and map to color
                    color = self.colormap(distance / max_distance)[:3]  # get RGB values
                    color = tuple(int(c * 255) for c in color)  # Convert to OpenCV BGR color format
                    cv2.circle(self.current_image, (x, y), 3, color, -1)  # Draw a filled circle

            # Display the image
            cv2.imshow('Distance Visualization', self.current_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageDistanceVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()