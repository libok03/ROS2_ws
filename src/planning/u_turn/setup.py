from setuptools import find_packages, setup

package_name = 'u_turn'

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
            "dwa_planner = u_turn.dwa_planner:main",
            "delau_planner = u_turn.delau_planner:main",
        ],
    },
)
