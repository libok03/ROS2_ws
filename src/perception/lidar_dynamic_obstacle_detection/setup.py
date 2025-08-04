from setuptools import find_packages, setup

package_name = 'lidar_dynamic_obstacle_detection'

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
    maintainer='hyeon',
    maintainer_email='hyeon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = lidar_dynamic_obstacle_detection.node:main',
            'detector_map_node = lidar_dynamic_obstacle_detection.node_odom:main',
        ],
    },
)
