from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hw2',  # 패키지 이름
            executable='pub_sub_test',  # Python 노드 이름
            name='pub_sub_test',
            output='screen',
        ),
    ])