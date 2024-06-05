import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError

class LidarCameraOverlay(Node):
    def __init__(self):
        super().__init__('lidar_camera_overlay')
        self.bridge = CvBridge()

        # Camera intrinsic matrix with new focal lengths
        fx = 1107.275337695818
        fy = 1185.2143866315462
        cx = 640  # Center x-coordinate
        cy = 360  # Center y-coordinate
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.zeros((4, 1))  # Zero distortion coefficients

        # Initial transformation settings
        self.tx = -1.6#-0.04
        self.ty = -0.88 # 0
        self.tz = 3.8#0.08 
        self.roll = 102
        self.pitch = 70
        self.yaw = -2
        self.update_transformation_matrix()

        # Subscriptions to image and LIDAR data
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan/fov', self.lidar_callback, 10)
        
        self.current_image = None

    def update_transformation_matrix(self):
        # Create rotation matrices from current roll, pitch, yaw
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(np.radians(self.roll)), -np.sin(np.radians(self.roll))],
            [0, np.sin(np.radians(self.roll)), np.cos(np.radians(self.roll))]
        ])
        
        Ry = np.array([
            [np.cos(np.radians(self.pitch)), 0, np.sin(np.radians(self.pitch))],
            [0, 1, 0],
            [-np.sin(np.radians(self.pitch)), 0, np.cos(np.radians(self.pitch))]
        ])
        
        Rz = np.array([
            [np.cos(np.radians(self.yaw)), -np.sin(np.radians(self.yaw)), 0],
            [np.sin(np.radians(self.yaw)), np.cos(np.radians(self.yaw)), 0],
            [0, 0, 1]
        ])

        # Combined rotation matrix
        R = Rz @ Ry @ Rx

        self.transformation_matrix = np.array([
            [R[0, 0], R[0, 1], R[0, 2], self.tx],
            [R[1, 0], R[1, 1], R[1, 2], self.ty],
            [R[2, 0], R[2, 1], R[2, 2], self.tz],
            [0, 0, 0, 1]
        ])

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {str(e)}')

    def lidar_callback(self, msg):
        if self.current_image is not None:
            overlay_image = self.overlay_lidar_on_image(msg, self.current_image.copy())
            cv2.imshow("LIDAR overlay on Image", overlay_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
            self.handle_key_input(key)

    def handle_key_input(self, key):
        translation_step = 0.2  # Step size for translation adjustments
        rotation_step = 2  # Step size for rotation adjustments (in degrees)

        if key == ord('w'):
            self.tz += translation_step
        elif key == ord('s'):
            self.tz -= translation_step
        elif key == ord('a'):
            self.tx -= translation_step
        elif key == ord('d'):
            self.tx += translation_step
        elif key == ord('r'):
            self.ty += translation_step
        elif key == ord('f'):
            self.ty -= translation_step

        elif key == ord('t'):
            self.pitch += rotation_step
        elif key == ord('g'):
            self.pitch -= rotation_step
        elif key == ord('h'):
            self.yaw += rotation_step
        elif key == ord('j'):
            self.yaw -= rotation_step
        elif key == ord('i'):
            self.roll += rotation_step
        elif key == ord('k'):
            self.roll -= rotation_step

        self.update_transformation_matrix()
        print(f"Updated transformation: tx={self.tx}, ty={self.ty}, tz={self.tz}, roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}")

    def overlay_lidar_on_image(self, lidar_data, image):
        for angle, distance in zip(np.arange(lidar_data.angle_min, lidar_data.angle_max, lidar_data.angle_increment), np.array(lidar_data.ranges)):
            if np.isfinite(distance) and distance > 0:
                x_lidar = distance * np.cos(angle)
                y_lidar = distance * np.sin(angle)
                z_lidar = 0.08  # Adjust if needed for actual LIDAR height
                point_lidar = np.array([x_lidar, y_lidar, z_lidar, 1])
                point_camera = np.dot(self.transformation_matrix, point_lidar)
                if point_camera[2] > 0:
                    point_image = np.dot(self.camera_matrix, point_camera[:3])
                    if point_image[2] != 0:
                        point_image /= point_image[2]
                        x_img, y_img = int(point_image[0]), int(point_image[1])
                        if 0 <= x_img < image.shape[1] and 0 <= y_img < image.shape[0]:
                            cv2.circle(image, (x_img, y_img), 2, (0, 255, 0), -1)
                        else:
                            print(f"Point out of image bounds: {x_img}, {y_img}")
                else:
                    print(f"Point behind camera or on camera plane: {point_camera}")
        return image

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraOverlay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
