import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray

import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray

class DepthFusionNode(Node):
    def __init__(self):
        super().__init__('depth_fusion_node')
        self.lidar_depth_sub = self.create_subscription(Float32MultiArray, '/lidar_pixel_distances', self.lidar_callback, 10)
        self.estimated_depth_sub = self.create_subscription(Float32MultiArray, '/depth_data', self.estimated_depth_callback, 10)
        self.fused_depth_pub = self.create_publisher(Float32MultiArray, '/fused_depth_data', 10)
        self.image_width = 640  # Example width
        self.image_height = 480  # Example height
        self.lidar_depth_map = np.zeros((self.image_height, self.image_width))
        self.estimated_depth_map = None

    def lidar_callback(self, msg):
        data = np.array(msg.data).reshape((-1, 3))
        self.lidar_depth_map.fill(0)  # Reset the depth map

        # Update the depth map with available LiDAR data, ignoring NaN values
        for x, y, depth in data:
            if 0 <= int(x) < self.image_width and 0 <= int(y) < self.image_height and not np.isnan(depth) and not np.isinf(depth):
                self.lidar_depth_map[int(y), int(x)] = depth

        interpolate = True  # Set to True to interpolate missing depth values
        if interpolate:
            # Interpolate missing depths in each row
            for y in range(self.lidar_depth_map.shape[0]):
                row = self.lidar_depth_map[y, :]
                known_depths = np.where(row > 0)[0]  # Indices of known depths

                if known_depths.size > 1:  # Ensure there are at least two points to interpolate between
                    for i in range(len(known_depths) - 1):
                        start = known_depths[i]
                        end = known_depths[i + 1]
                        
                        # Perform linear interpolation between known depth points
                        if end - start > 1:  # Check if there is a gap to fill
                            interp_values = np.linspace(row[start], row[end], num=(end - start + 1))
                            row[start:end+1] = interp_values

            # Optionally, you can fill in the edges if they have missing values
            # For example, extrapolate using the first or last known value
            if known_depths.size > 0:
                if known_depths[0] != 0:
                    row[:known_depths[0]] = row[known_depths[0]]
                if known_depths[-1] != len(row) - 1:
                    row[known_depths[-1]+1:] = row[known_depths[-1]]
        
            self.lidar_depth_map[y, :] = row  # Update the row in the depth map

        
        #print (np.min(self.lidar_depth_map), np.max(self.lidar_depth_map))

    def estimated_depth_callback(self, msg):
        self.estimated_depth_map = np.array(msg.data).reshape((self.image_height, self.image_width))
        self.compute_and_fuse_depths()

    def compute_and_fuse_depths(self):
        # Check if the estimated depth map has been initialized
        if self.estimated_depth_map is None:
            return  # Exit the function if no estimated depth data is available

        # Initialize a new depth map with zeros that matches the size of the LiDAR depth map
        fused_depth_map = np.zeros_like(self.lidar_depth_map)

        #print(np.min(self.lidar_depth_map), np.max(self.lidar_depth_map))

        # Iterate over each vertical column in the depth map
        for x in range(self.image_width):
            # Extract the depth values for the current column from both LiDAR and estimated depth maps
            column = self.lidar_depth_map[:, x]
            estimated_column = self.estimated_depth_map[:, x]

            # Find the index of the first non-zero depth in the LiDAR column, which is our reference point
            reference_index = np.argmax(column > 0)
            
            # If the reference point has a depth of zero, skip this column because there is no LiDAR data to use as a reference
            if column[reference_index] == 0:
                continue

            # Store the depth value from LiDAR and the estimated depth map at the reference point
            reference_depth = column[reference_index]
            reference_estimated_depth = estimated_column[reference_index]

            # Iterate over all pixels in the column to adjust depths based on the reference point
            for y in range(self.image_height):
                # Only update pixels that do not already have a LiDAR depth measurement
                if column[y] == 0:
                    # Calculate the relative depth difference between the current pixel's estimated depth and the reference estimated depth
                    relative_depth = estimated_column[y] - reference_estimated_depth

                    # Instead of this calculation use a kalman filter to estimate the depth
                    # Adjust the depth of the current pixel based on the relative depth and the scale of the reference LiDAR depth
                    # This assumes a linear relationship between the change in estimated depth and actual depth
                    column[y] = reference_depth + (relative_depth * reference_depth / reference_estimated_depth)
                    
                    #if reference_estimated_depth != 0:
                    #    scaling_factor = reference_depth / reference_estimated_depth
                    #    column[y] = reference_depth + (relative_depth * scaling_factor)
                    #else:
                    #    column[y] = reference_depth  # Default to LiDAR depth if no valid estimated depth is available

            # Update the fused depth map with the adjusted column values
            fused_depth_map[:, x] = column


        # print(np.min(fused_depth_map), np.max(fused_depth_map))
        #print(min(fused_depth_map), max(fused_depth_map))

        # After processing all columns, publish the fused depth map and visualize it
        self.publish_and_visualize_depth(fused_depth_map)

    
    def publish_and_visualize_depth(self, depth_map):
        # Normalize for visualization
        depth_display = cv2.normalize(depth_map, None, 0, 255, norm_type=cv2.NORM_MINMAX)
        depth_colormap = cv2.applyColorMap(depth_display.astype(np.uint8), cv2.COLORMAP_JET)
        cv2.imshow('Fused Depth Visualization', depth_colormap)
        cv2.waitKey(1)

        # Publish fused depth
        fused_depth_msg = Float32MultiArray()
        fused_depth_msg.data = depth_map.flatten().tolist()
        self.fused_depth_pub.publish(fused_depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  # Ensure all OpenCV windows are closed on shutdown

if __name__ == '__main__':
    main()