from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cropbox',  # 패키지 이름
            executable='cropbox',  # Python 노드 이름
            name='cropbox',
            output='screen',
        ),
    ])