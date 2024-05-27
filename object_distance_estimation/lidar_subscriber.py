#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize the plot
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'bo', markersize=2)
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('LIDAR Scan')

        # Add angle markers
        self.add_angle_markers()

        plt.ion()
        plt.show()

    def add_angle_markers(self):
        # Add lines at 0, 90, 180, 270 degrees
        angles = [0, 90, 180, 270]
        for angle in angles:
            rad = np.deg2rad(angle)
            x = [0, 10 * np.cos(rad)]
            y = [0, 10 * np.sin(rad)]
            self.ax.plot(x, y, 'r--')

        # Add annotations for the angles
        self.ax.text(10, 0, '0°', color='red', fontsize=12, ha='center')
        self.ax.text(0, 10, '90°', color='red', fontsize=12, ha='center')
        self.ax.text(-10, 0, '180°', color='red', fontsize=12, ha='center')
        self.ax.text(0, -10, '270°', color='red', fontsize=12, ha='center')
        self.ax.text(10, 0, '360°', color='red', fontsize=12, ha='center')  # 360° is same as 0°

    def listener_callback(self, msg):
        # Calculate angles
        angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))])
        ranges = np.array(msg.ranges)

        # Calculate x and y coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Update the plot
        self.ln.set_data(x, y)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)

    # Destroy the node explicitly (optional - done automatically when garbage collected)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
