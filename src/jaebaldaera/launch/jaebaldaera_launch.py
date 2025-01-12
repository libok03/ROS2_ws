from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jaebaldaera',
            executable='publisher_node',
            name='publisher_node',
            output="screen"
        ),
        Node(
            package='jaebaldaera',
            executable='subscriber_node',
            name='subscriber_node',
            output="screen",
        )
    ])