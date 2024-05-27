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
    maintainer='puzzlebot',
    maintainer_email='codres_ali@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'lidar_subscriber = object_distance_estimation.lidar_subscriber:main',
		'cam_visualization = object_distance_estimation.cam_visualisation:main'
        ],
    },
)
