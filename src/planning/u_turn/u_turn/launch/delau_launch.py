from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='u_turn',  # 패키지 이름
            executable='delau_planner',  # Python 노드 이름
            name='delau_planner',
            output='screen',
        ),
    ])