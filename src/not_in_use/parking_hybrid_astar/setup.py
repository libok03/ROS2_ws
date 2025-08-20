from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'parking_hybrid_astar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['parking_hybrid_astar', 'parking_hybrid_astar.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            'path_planner = parking_hybrid_astar.path_planner:main',
            "erp_42_msg_shooter = parking_hybrid_astar.erp_42_msg_shooter:main"
        ],
    },
)
