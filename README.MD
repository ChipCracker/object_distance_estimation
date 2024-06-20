# Project (WIP)

This package, object_distance_estimation, provides tools for estimating the distance of objects detected through LiDAR and visualized on camera images in a ROS 2 environment. The nodes in this package integrate LiDAR data with camera feeds to compute and display the distances of objects in real-time.

# Package Setup

1.	Dependencies: \
	•	ROS 2 (Humble or later recommended) \
	•	OpenCV \
	•	Open3D \
	•	Matplotlib \
	•	Numpy
2.	Installation: \
Ensure ROS 2 and the aforementioned Python libraries are installed on your system. Then, clone this package into your ROS workspace and compile it using colcon:
``` bash
cd ~/ros_workspace/src
git clone <repository_url> object_distance_estimation
cd ~/ros_workspace
colcon build --packages-select object_distance_estimation
source install/setup.bash

```

# Nodes Description

1.	start_camera.sh
	•	This script initializes the camera and starts publishing image data to the /image_raw topic.  \
	•	Usage: `./start_camera.sh`
2.	start_lidar.sh
	•	This script starts the LiDAR sensor and ensures that it publishes scan data to the /scan topic. \
	•	Usage: `./start_lidar.sh`
3.	lidar_to_camera_transformation_node
	•	Converts LiDAR scan data to a camera coordinate system and publishes pixel distance data on /lidar_pixel_distances. \
	•	Run Command: `ros2 run object_distance_estimation lidar_to_camera_transformation_node`
4.	image_distance_visualizer_node
	•	Subscribes to /image_raw and /lidar_pixel_distances, overlays distance information using colors on the camera images, and displays them. \
	•	Run Command: `ros2 run object_distance_estimation image_distance_visualizer_node`


# Running the System

To operate the system, follow these steps in separate terminal windows:

1.	Start the camera:
`./start_camera.sh`

2.	Start the LiDAR:
`./start_lidar.sh`

3.	Run the transformation node:
`ros2 run object_distance_estimation lidar_to_camera_transformation_node`

4.	Run the visualization node:
`ros2 run object_distance_estimation image_distance_visualizer_node`




# Overview of Node Interactions and Data Flow

## Main Data Flow:

1.	LiDAR and Camera Data Acquisition:
- `lidar_to_camera_transformation_node` processes raw LiDAR scans from the `/scan` topic and transforms these points into camera coordinates, publishing to `/lidar_pixel_distances`.
- `depth_estimator_node` processes raw images from `/image_raw` to estimate depth, publishing depth estimations to `/depth_data`.
2.	Depth Data Fusion (WIP):
- `depth_fusion_node` subscribes to `/lidar_pixel_distances` and `/depth_data,` fusing this information to produce a comprehensive depth map, which it publishes to `/fused_depth_data`.

## Visualization and Debugging:

#### Intermediate Visualizations:
- `lidar_camera_overlay overlays` LiDAR points onto camera images for visual verification of alignment and accuracy.
- `image_distance_visualizer_node` visualizes distances directly on the camera images for real-time feedback.
- `depth_visualizer_node` displays depth images based on the output from `depth_estimator_node.`
#### Debugging and Additional Processing:
- `lidar_2d_plotter_node` provides a 2D plot of the LiDAR scan, useful for debugging and analysis.
- `image_preprocessor_node` calibrates and preprocesses camera images, the data is published under `/image/undistorted` (this topic will later be used instead of the `/image/raw` topic)



# Troubleshooting
Ensure all hardware devices are properly connected and configured. Check the ROS topics using ros2 topic list and ensure that data flows through the topics as expected. If visualization does not appear, verify the dependencies, especially the OpenCV and Matplotlib installations.

# Maintainers
- Christopher Witzl
- Email: christopher.witzl@gmx.de

Feel free to contact for support or to contribute to this package. For more detailed documentation on specific nodes or functionalities, refer to the code comments or additional documentation provided in the package.