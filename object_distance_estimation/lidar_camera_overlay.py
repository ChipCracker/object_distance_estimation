import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import open3d as o3d
import matplotlib.pyplot as plt

def cam_matrix_1():
    # Image dimensions 4:3 aspect ratio
    image_width = 640
    image_height = 480

    # Camera principal points
    cx = image_width / 2
    cy = image_height / 2

    # Field of View http://vrguy.blogspot.com/2013/04/converting-diagonal-field-of-view-and.html
    fov = 57.5  # degrees

    # Convert FOV from degrees to radians for calculation
    fov_rad = np.deg2rad(fov)

    # Calculate focal length based on horizontal FOV
    fx = image_width / (2 * np.tan(fov_rad / 2))
    fy = fx  # Assuming aspect ratio is 1 (square pixels)

    print(fx, fy)

    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])
    return camera_matrix
    #[[772.54833996   0.         320.        ]
    #[  0.         772.54833996 240.        ]
    #[  0.           0.           1.        ]]


class LidarCameraOverlay(Node):
    def __init__(self):
        super().__init__('lidar_camera_overlay')
        self.bridge = CvBridge()

        print(cam_matrix_1())

        self.camera_matrix = cam_matrix_1()
        
        self.dist_coeffs = np.zeros((4, 1))  # Zero distortion coefficients

        # Initial transformation settings
        self.tx = -0.07 # 7cm behind the camera
        self.ty = 0
        self.tz = 0.0
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
        print(T)
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

        # remove points outside of the camera's field of view

        angles_deg = np.degrees(angles)
        #print(angles_deg)

        angles_min = 155 - 180
        angles_max = 205 - 180

        angles_min = np.deg2rad(angles_min)
        angles_max = np.deg2rad(angles_max)

        filtered_ranges = []
        filtered_angles = []

        for i in range(len(angles)):
            if not (angles[i] > angles_min and angles[i] < angles_max):
                filtered_ranges.append(ranges[i])
                filtered_angles.append(angles[i])


        ranges = np.array(filtered_ranges)
        angles = np.array(filtered_angles)

        points = np.vstack((np.cos(angles) * ranges, np.sin(angles) * ranges, np.zeros_like(ranges)))
        points = np.vstack((points, np.ones(points.shape[1])))  # Homogeneous coordinates

        # Transform LiDAR points to camera coordinates
        transformed_points = np.dot(self.transformation_matrix, points)

        self.lidar_points = transformed_points
        if self.current_image is not None:
            self.overlay_lidar_on_image()

    def calculate_depth_color(self, distances, max_distance):
        # Normalize the distances based on the maximum expected distance
        normalized = distances / max_distance
        # Convert normalized distance to a colormap (e.g., red close, blue far)
        colors = plt.cm.jet(normalized)[:, :3]  # Ignore alpha channel, take RGB
        return colors * 255  # Scale colors to 0-255 for OpenCV

    def overlay_lidar_on_image(self):
        if self.lidar_points is None:
            return

         # Extract depth from points (assuming depth is in the z-coordinate of camera space)
        depth = np.linalg.norm(self.lidar_points[:3], axis=0)
        max_depth = np.max(depth)  # You might want to set this manually
        colors = self.calculate_depth_color(depth, max_depth)

        points_in_camera_coords = np.array([
                self.lidar_points[1] * -1,
                self.lidar_points[2] * -1,
                self.lidar_points[0]])
        
        # Project points to 2D image plane
        image_points, _ = cv2.projectPoints(
            points_in_camera_coords[:3].T.reshape(-1, 1, 3),
            np.zeros((3, 1)), np.zeros((3, 1)),
            self.camera_matrix, self.dist_coeffs)
        

        print(image_points.shape)
        # Subtract 25 from the y-coordinate of each point
        image_points[:, :, 1] -= 30

        for i, pt in enumerate(image_points):
            x, y = pt.ravel()
            if 0 <= x < self.current_image.shape[1] and 0 <= y < self.current_image.shape[0]:
                color = tuple(int(c) for c in colors[i])  # Convert color to tuple
                cv2.circle(self.current_image, (int(x), int(y)), 2, color, -1)
    

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