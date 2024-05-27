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
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('LIDAR Scan')
        plt.ion()
        plt.show()

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
