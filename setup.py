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
		'lidar_subscriber = object_distance_estimation.lidar_subscriber:main',
        'lidar_camera_fov_viz = object_distance_estimation.lidar_camera_fov_viz:main',
		'cam_visualisation = object_distance_estimation.cam_visualisation:main',
        'image_preprocessor_node = object_distance_estimation.image_preprocessor_node:main',
        'lidar_camera_overlay = object_distance_estimation.lidar_camera_overlay:main',
        'lidar_to_camera_transformation_node = object_distance_estimation.lidar_to_camera_transformation_node:main',
        'image_distance_visualizer_node = object_distance_estimation.image_distance_visualizer_node:main',
        'lidar_fov_preprocessor_node = object_distance_estimation.lidar_fov_preprocessor_node:main',
        ],
    },
)
