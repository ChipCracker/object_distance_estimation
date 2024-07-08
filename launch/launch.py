from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_distance_estimation',
            executable='lidar_to_camera_transformation_node',
            name='lidar_camera_transformation',
            output='screen'
        ),
        Node(
            package='object_distance_estimation',
            executable='depth_estimator_node',
            name='depth_estimator',
            output='screen'
        ),
        Node(
            package='object_distance_estimation',
            executable='depth_fusion_node',
            name='depth_fusion',
            output='screen'
        ),
        Node(
            package='object_distance_estimation',
            executable='image_distance_visualizer_node',
            name='image_distance_visualizer',
            output='screen'
        ),
        Node(
            package='object_distance_estimation',
            executable='yolov5_node',
            name='yolov5',
            output='screen'
        ),
        Node(
            package='object_distance_estimation',
            executable='visualization_node',
            name='visualization',
            output='screen'
        )
    ])