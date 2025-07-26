from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # plane_ground_filter.launch.py 경로
    plane_ground_filter_launch_path = os.path.join(
        get_package_share_directory("plane_ground_filter"),
        "launch",
        "plane_ground_filter.launch.py",
    )

    # adaptive_clustering.launch.py 경로
    adaptive_clustering_launch_path = os.path.join(
        get_package_share_directory("adaptive_clustering"),
        "launch",
        "adaptive_clustering.launch.py",
    )
    
    return LaunchDescription(
        [
            # cropbox 노드 실행
            Node(
                package="fusion",
                executable="cropbox",
                name="cropbox_node",
                output="screen",
            ),
            # plane_ground_filter 런치 파일 실행
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(plane_ground_filter_launch_path)
            ),
            # adaptive_clustering 런치 파일 실행
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(adaptive_clustering_launch_path)
            ),
            # parking_hybrid_astar파일 실행
            Node(
                package="no_gps_path",
                executable="costmap_publisher",
                name= "costmap_publisher",
                output = "screen"
            ),
            # parking_hybrid_astar파일 실행
            Node(
                package="no_gps_path",
                executable="dwa_publisher",
                name= "dwa_publisher",
                output = "screen"
            )
        ]
    )