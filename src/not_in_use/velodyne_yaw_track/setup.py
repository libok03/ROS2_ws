from setuptools import find_packages, setup

package_name = 'velodyne_yaw_track'

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
    maintainer='libok',
    maintainer_email='libok1117@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "yaw_track = velodyne_yaw_track.lidar_heading_estimator:main",
            "debugger = velodyne_yaw_track.debug:main",
        ],
    },
)
