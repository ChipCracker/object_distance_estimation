#!/bin/bash

# Skript zur Initialisierung des ROS 2-Systems und Starten mehrerer Nodes

echo "Berechtigungen für ttyUSB0 setzen..."
sudo chmod 777 /dev/ttyUSB0

echo "SLLidar ROS2 Launch..."
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py &

echo "Starte v4l2_camera_node..."
ros2 run v4l2_camera v4l2_camera_node &

echo "Starte image_preprocessor_node aus object_distance_estimation..."
ros2 run object_distance_estimation image_preprocessor_node &

echo "Starte depth_estimator_node aus object_distance_estimation..."
ros2 run object_distance_estimation depth_estimator_node &

echo "Starte lidar_to_camera_transformation_node aus object_distance_estimation..."
ros2 run object_distance_estimation lidar_to_camera_transformation_node &

echo "Starte depth_fusion_node aus object_distance_estimation..."
ros2 run object_distance_estimation depth_fusion_node &

echo "Starte image_distance_visualizer_node aus object_distance_estimation..."
ros2 run object_distance_estimation image_distance_visualizer_node &

echo "Starte yolov5_node aus object_distance_estimation..."
ros2 run object_distance_estimation yolov5_node &

echo "Starte visualization_node aus object_distance_estimation..."
ros2 run object_distance_estimation visualization_node &

echo "Alle Prozesse gestartet. Zum Beenden Ctrl+C drücken."
wait