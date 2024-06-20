import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import open3d as o3d
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

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

class Lidar_to_Camera_Transformation_Node(Node):
    def __init__(self):
        super().__init__('lidar_camera_overlay')
        # Initialize publishers
        self.distance_pub = self.create_publisher(Float32MultiArray, 'lidar_pixel_distances', 10)
        # Existing initializations
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
        self.overlay_lidar_on_image()



    def overlay_lidar_on_image(self):
        if self.lidar_points is None:
            return
        depth = np.linalg.norm(self.lidar_points[:3], axis=0)
        points_in_camera_coords = np.array([
            self.lidar_points[1] * -1,
            self.lidar_points[2] * -1,
            self.lidar_points[0]])

        image_points, _ = cv2.projectPoints(
            points_in_camera_coords[:3].T.reshape(-1, 1, 3),
            np.zeros((3, 1)), np.zeros((3, 1)),
            self.camera_matrix, self.dist_coeffs)

        image_points[:, :, 1] -= 30

        # Combine image points and depth into a single array
        image_points_depth = np.hstack((image_points.reshape(-1, 2), depth[:, None]))

        # print min max of depth
        # print(np.min(image_points_depth[:, 2]), np.max(image_points_depth[:, 2]))

        # Ensure all elements are floats
        image_points_depth = image_points_depth.astype(np.float32)

        # Flatten the array and convert to list
        flat_list = image_points_depth.flatten().tolist()

        # Prepare the data for publishing
        distance_array = Float32MultiArray()
        distance_array.data = flat_list
        self.distance_pub.publish(distance_array)

        

def main(args=None):
    rclpy.init(args=args)
    node = Lidar_to_Camera_Transformation_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()