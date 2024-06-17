import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import open3d as o3d
from geometry_msgs.msg import TransformStamped

class LidarCameraOverlay(Node):
    def __init__(self):
        super().__init__('lidar_camera_overlay')
        self.bridge = CvBridge()

        # Image dimensions
        image_width = 640
        image_height = 480

        # Camera principal points
        cx = image_width / 2
        cy = image_height / 2

        # Field of View
        fov = 55  # degrees

        # Convert FOV from degrees to radians for calculation
        fov_rad = np.deg2rad(fov)

        # Calculate focal length based on horizontal FOV
        fx = image_width / (2 * np.tan(fov_rad / 2))
        fy = fx  # Assuming aspect ratio is 1 (square pixels)

        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.zeros((4, 1))  # Zero distortion coefficients

        # Initial transformation settings
        self.tx = -0.04
        self.ty = 0
        self.tz = 0.08
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # Transformation matrix from LiDAR to camera coordinates
        self.transformation_matrix = self.calculate_transformation_matrix()

        # Subscriptions to image and LiDAR data
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        # Storage for sensor data
        self.current_image = None
        self.lidar_points = None

    def calculate_transformation_matrix(self):
        # Compute the rotation matrix from Euler angles
        R = o3d.geometry.get_rotation_matrix_from_xyz((self.roll, self.pitch, self.yaw))
        # Create transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [self.tx, self.ty, self.tz]
        return T

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            if self.lidar_points is not None:
                self.overlay_lidar_on_image()
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        # Filter out points with 'inf' or 'nan' values which can mess up transformations
        finite_ranges = np.isfinite(ranges)
        ranges = ranges[finite_ranges]
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
    
        points = np.vstack((np.cos(angles) * ranges, np.sin(angles) * ranges, np.zeros_like(ranges)))
        points = np.vstack((points, np.ones(points.shape[1])))  # Homogeneous coordinates


        # Transform LiDAR points to camera coordinates
        transformed_points = np.dot(self.transformation_matrix, points)
        self.lidar_points = transformed_points
        if self.current_image is not None:
            self.overlay_lidar_on_image()

    def overlay_lidar_on_image(self):
        image_points, _ = cv2.projectPoints(self.lidar_points[:3].T.reshape(-1, 1, 3),
                                            np.zeros((3, 1)), np.zeros((3, 1)),
                                            self.camera_matrix, self.dist_coeffs)
        
        
        for pt in image_points:
            x, y = pt.ravel()
            print(x,y)
            cv2.circle(self.current_image, (int(x), int(y)), 2, (0, 255, 0), -1)

        # Display the image
        cv2.imshow('LiDAR overlay', self.current_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraOverlay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()