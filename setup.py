from setuptools import find_packages, setup

package_name = 'object_distance_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christopher Witzl',
    maintainer_email='christopher.witzl@gmx.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        # for plotting the 2D LiDAR scan
		'lidar_2d_plotter_node = object_distance_estimation.lidar_2d_plotter_node:main',
        # for preprocessing the image and undistorting it
        'image_preprocessor_node = object_distance_estimation.image_preprocessor_node:main',
        # for overlaying the LiDAR points on the camera image within one step
        'lidar_camera_overlay = object_distance_estimation.lidar_camera_overlay:main',
        # for transforming LiDAR points to camera coordinates
        'lidar_to_camera_transformation_node = object_distance_estimation.lidar_to_camera_transformation_node:main',
        # for visualizing the distance between the camera and the object
        'image_distance_visualizer_node = object_distance_estimation.image_distance_visualizer_node:main',
        ],
    },
)
