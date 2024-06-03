#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches

class LidarSectorSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_sector_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.size = 1

        # Initialize the plot
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro', markersize=5)
        self.ax.set_xlim(-self.size, self.size)
        self.ax.set_ylim(-self.size, self.size)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('LIDAR Scan - 155° to 205°')

        # Define the angular sector
        self.min_angle = np.deg2rad(155)  # 25 degrees left of 180
        self.max_angle = np.deg2rad(205)  # 25 degrees right of 180

        # Add FOV cones
        self.add_fov_cones()

        plt.ion()
        plt.show()

    def add_fov_cones(self):
        # Add cones to visualize the field of view
        fov_range = self.size

        # Calculate the end points for the left and right FOV lines
        left_x, left_y = fov_range * np.cos(self.min_angle), fov_range * np.sin(self.min_angle)
        right_x, right_y = fov_range * np.cos(self.max_angle), fov_range * np.sin(self.max_angle)

        # Add lines for left and right boundaries of the FOV
        self.ax.plot([0, left_x], [0, left_y], 'g--', label='Left FOV boundary')
        self.ax.plot([0, right_x], [0, right_y], 'g--', label='Right FOV boundary')

        # Add a shaded area to represent the FOV visually
        fov = patches.Wedge(center=(0, 0), r=fov_range, theta1=np.rad2deg(self.min_angle),
                            theta2=np.rad2deg(self.max_angle), color='green', alpha=0.1)
        self.ax.add_patch(fov)

    def listener_callback(self, msg):
        # Normalize angles to 0° to 360° range for comparison
        angles = np.degrees(msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment) % 360
        normalized_min_angle = np.degrees(self.min_angle) % 360
        normalized_max_angle = np.degrees(self.max_angle) % 360

        # Calculate ranges
        ranges = np.array(msg.ranges)

        # Filter angles within the specific sector accounting for angle wrapping
        sector_mask = ((angles >= normalized_min_angle) & (angles <= normalized_max_angle)) if normalized_min_angle < normalized_max_angle \
                      else ((angles >= normalized_min_angle) | (angles <= normalized_max_angle))  # Handle wrapping around 360°
        sector_angles = angles[sector_mask]
        sector_ranges = ranges[sector_mask]

        # Convert sector angles back to radians for plotting
        sector_angles = np.radians(sector_angles)

        # Calculate x and y coordinates for the sector
        x = sector_ranges * np.cos(sector_angles)
        y = sector_ranges * np.sin(sector_angles)

        # Update the plot
        self.ln.set_data(x, y)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    lidar_sector_subscriber = LidarSectorSubscriber()
    rclpy.spin(lidar_sector_subscriber)

    # Destroy the node explicitly (optional - done automatically when garbage collected)
    lidar_sector_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
