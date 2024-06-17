#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarFOVPreProcessor(Node):
    def __init__(self):
        super().__init__('lidar_fov_preprocessor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10)

        # Define the angular sector
        self.min_angle = np.deg2rad(155)  # 25 degrees left of 180
        self.max_angle = np.deg2rad(205)  # 25 degrees right of 180

    def listener_callback(self, msg):
        # Normalize angles to 0° to 360° range for comparison
        angles = np.degrees(msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment) % 360
        normalized_min_angle = np.degrees(self.min_angle) % 360
        normalized_max_angle = np.degrees(self.max_angle) % 360

        # Filter angles within the specific sector accounting for angle wrapping
        sector_mask = ((angles >= normalized_min_angle) & (angles <= normalized_max_angle)) if normalized_min_angle < normalized_max_angle \
                      else ((angles >= normalized_min_angle) | (angles <= normalized_max_angle))

        # Prepare a new LaserScan message with filtered data
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = self.min_angle
        filtered_scan.angle_max = self.max_angle
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = [float('inf')] * len(msg.ranges)
        filtered_scan.intensities = []

        # Apply the sector mask to ranges
        indices = np.where(sector_mask)[0]
        for i in indices:
            filtered_scan.ranges[i] = msg.ranges[i]

        # Optionally handle intensities if available
        if msg.intensities:
            filtered_scan.intensities = [0.0] * len(msg.intensities)
            for i in indices:
                filtered_scan.intensities[i] = msg.intensities[i]

        # Publish the filtered LaserScan data
        self.publisher.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFOVPreProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
