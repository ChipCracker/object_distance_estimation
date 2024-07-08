# Project (WIP)

This package, object_distance_estimation, provides tools for estimating the distance of objects detected through LiDAR and visualized on camera images in a ROS 2 environment. The nodes in this package integrate LiDAR data with camera feeds to compute and display the distances of objects in real-time.

# Package Setup

## 1.	Dependencies: 
	•	ROS 2 (Humble or later recommended) 
	•	OpenCV 
	•	Open3D 
	•	Matplotlib 
	•	Numpy 
    •	Torch 
    •	Torchvision 
    •	YOLOv5 
	•	sllidar node 
	•	v2l4 node
	•	CamerCalibration scripts

## 1.1. Install the python dependencies
``` bash
sudo apt install python3-pip -y

pip3 install torch open3d torchvision pandas timm numpy==1.26.4
```
	
## 2. Sllidar ros2 node installation
1. Visit the official repository (https://github.com/Slamtec/sllidar_ros2)
2. Follow the install instructions

## 3. v2l4 ros2 node installation
1. Visit the official v2l4 ros2  node repository (https://gitlab.com/boldhearts/ros2_v4l2_camera)
2. Follow the install instructions

## 4. Download and Calibration of the webcam
1. clone this repo (https://github.com/ChipCracker/CameraCalibration)
2. capture the calibration images
3. perform the calibration

## 5. Installation : 
Ensure ROS 2 and the aforementioned Python libraries are installed on your system. Then, clone this package into your ROS workspace and compile it using colcon:
``` bash
cd ~/ros_workspace/src
git clone <repository_url> object_distance_estimation
```
Now copy the `calibration.pkl` file, that has been created from the calibration step, to this directory `~/ros_workspace/src/object_distance_estimation/object_distance_estimation`.\
If you want to skip this step a default one is used for the [hp320 FHD webcam](https://www.hp.com/de-de/shop/product.aspx?id=53x26aa&opt=abb&sel=acc)

Then execute this:

``` bash
cd ~/ros_workspace
colcon build --packages-select object_distance_estimation
source install/setup.bash
```

# Nodes Description

#### start_camera.sh
	•	This script initializes the camera and starts publishing image data to the /image_raw topic. 
	•	Usage: `./start_camera.sh`

#### start_lidar.sh
	•	This script starts the LiDAR sensor and ensures that it publishes scan data to the /scan topic. 
	•	Usage: `./start_lidar.sh`

#### lidar_to_camera_transformation_node
	•	Converts LiDAR scan data to a camera coordinate system and publishes pixel distance data on /lidar_pixel_distances. 
	•	Run Command: `ros2 run object_distance_estimation lidar_to_camera_transformation_node`

#### image_distance_visualizer_node
	•	Subscribes to /image_raw and /lidar_pixel_distances, overlays distance information using colors on the camera images, and displays them. 
	•	Run Command: `ros2 run object_distance_estimation image_distance_visualizer_node`

#### yolov5_node
	•	Performs object detection on images from the /image_raw topic and publishes detected objects to /detected_objects in JSON format.
	•	Run Command: `ros2 run object_distance_estimation yolov5_node`
	
#### visualization_node
	•	Subscribes to /image_raw, /detected_objects, and /fused_depth_data. It visualizes detected objects and their estimated distances on the images.
	•	Run Command: `ros2 run object_distance_estimation visualization_node`

# Running the System
To operate the system, execute the `start_depth_estimation.sh script` in the scripts folder, this will start all necessary nodes.

## Main Data Flow:

#### 1.	LiDAR and Camera Data Acquisition:
- `lidar_to_camera_transformation_node` processes raw LiDAR scans from the `/scan` topic and transforms these points into camera coordinates, publishing to `/lidar_pixel_distances`.
- `depth_estimator_node` processes raw images from `/image_raw` to estimate depth, publishing depth estimations to `/depth_data`.
#### 2.	Depth Data Fusion (WIP):
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
- Maximilian Werzinger
- Email: max.werzinger@franken-online.org

Feel free to contact for support or to contribute to this package. For more detailed documentation on specific nodes or functionalities, refer to the code comments or additional documentation provided in the package.