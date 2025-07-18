from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parking',  # 패키지 이름
            executable='parking',  # Python 노드 이름
            name='parking',
            output='screen',
        ),
        Node(
            package='parking',  # 패키지 이름
            executable='pose_filter',  # Python 노드 이름
            name='pose_filter',
            output='screen',
        ),
    ])