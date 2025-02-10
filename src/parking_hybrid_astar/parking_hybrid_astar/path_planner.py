import rclpy
import numpy as np
from geometry_msgs.msg import Quaternion
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, PoseStamped
from scipy.spatial.distance import cdist
from nav_msgs.msg import Path
import os
import sys
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



# 현재 파일의 디렉토리 경로
current_dir = os.path.dirname(os.path.abspath(__file__))

# 프로젝트 루트 경로 추가
project_root = os.path.abspath(os.path.join("/home/libok/dev_ws/src/parking_hybrid_astar/parking_hybrid_astar"))
sys.path.append(project_root)

# 상대 경로 import
from PathPlanning.HybridAStar.hybrid_a_star import (
    hybrid_a_star_planning,
    XY_GRID_RESOLUTION,
    YAW_GRID_RESOLUTION
)



class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # 현재 위치, 맵, filtered_cone_poses를 저장할 변수
        self.current_pose = None
        self.filtered_cone_poses = None

        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.obstacle_pub = self.create_publisher(PoseArray, '/obstacle_poses', 10)
        self.marker_pub = self.create_publisher(Marker, '/filter_area_marker', 10)

        # Odometry 구독
        self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odom_callback,
            10)

        # Cone Poses 구독
        self.create_subscription(
            PoseArray,
            '/cone_poses',
            self.cone_poses_callback,
            10)

        # 주기적으로 경로 계획을 수행하는 타이머 생성
        self.create_timer(0.033, self.plan_path)

        self.ox=[-17,2]
        self.oy=[2,8]
        self.obstacle_list = []

        self.flag_find_route = False
        self.flag_start_point = False
        self.get_logger().info('Path Planner Node has been initialized.')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.get_logger().debug('Received current pose')
        ######################################localization point#######################################
        if cdist([[self.current_pose.position.x,self.current_pose.position.y]],[[2, 1]]) <= 2:
        ###############################################################################################
            self.flag_start_point = True


    def cone_poses_callback(self, msg):
        # 고정 박스 좌표 설정 (절대 좌표계 사용)
        BOX_X_MIN = -16.0   # X 최소값 (변경 금지)
        BOX_X_MAX = 1.0   # X 최대값 (변경 금지)
        BOX_Y_MIN = 3.0    # Y 최소값 (변경 금지)
        BOX_Y_MAX = 7.0    # Y 최대값 (변경 금지)

        if msg.poses:
            # 모든 콘 위치 추출
            new_cones = [[pose.position.x, pose.position.y] for pose in msg.poses]

            # 박스 필터링 (절대 좌표 기준)
            box_filtered = [cone for cone in new_cones if BOX_X_MIN <= cone[0] <= BOX_X_MAX and BOX_Y_MIN <= cone[1] <= BOX_Y_MAX]

            # 기존 장애물과 거리 계산
            if self.obstacle_list:
                final_cones = []
                for cone in box_filtered:
                    min_dist = min(((cone[0] - obs[0])**2 + (cone[1] - obs[1])**2)**0.5 for obs in self.obstacle_list)
                    if min_dist > 0.8:
                        final_cones.append(cone)
            else:
                final_cones = box_filtered

            # 장애물 리스트 업데이트
            if final_cones:
                self.obstacle_list.extend(final_cones)
                self.ox.extend([cone[0] for cone in final_cones])
                self.oy.extend([cone[1] for cone in final_cones])

            self.get_logger().info(f"New obstacles added: {len(final_cones)}")
            self.publish_obstacle_poses()



    def publish_obstacle_poses(self):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"

        for obstacle in self.obstacle_list:
            pose = PoseStamped().pose
            pose.position.x = float(obstacle[0])
            pose.position.y = float(obstacle[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0  # 기본 방향 설정
            pose_array.poses.append(pose)

        self.obstacle_pub.publish(pose_array)

    def euler_from_quaternion(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)
        return q


    def plan_path(self):
        if not self.flag_find_route and self.flag_start_point:
            self.get_logger().info("ok lets start path finding")

            # 장애물 리스트를 NumPy 배열로 변환
            obstacles = np.array(self.obstacle_list)
            self.get_logger().info(f"obstacle number: {len(obstacles)}")

            # 차량의 현재 위치 정보 추출
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y

            # 필터 박스 범위 (차량 기준 상대 좌표)
            # 여기에 local_x_min, local_x_max, local_y_min, local_y_max는 차량 기준 값 (회전 변환 없이 적용)
            local_x_min, local_x_max = -3.0, -1.0   # 차량 전방이 아니라, 차량 기준 예: 후방 부분: -3m ~ -1m
            local_y_min, local_y_max = 0.0, 3.0      # 차량 우측: 0m ~ 3m

            # 절대 좌표계에서 필터 영역을 계산 (현재 차량 포지션 기준으로 오프셋 적용)
            abs_x_min = current_x + local_x_min
            abs_x_max = current_x + local_x_max
            abs_y_min = current_y + local_y_min
            abs_y_max = current_y + local_y_max

            # 필터 박스 범위 내 장애물 확인 (회전 변환 없이)
            if obstacles:
                in_x_range = (obstacles[:,0] >= abs_x_min) & (obstacles[:,0] <= abs_x_max)
                in_y_range = (obstacles[:,1] >= abs_y_min) & (obstacles[:,1] <= abs_y_max)
                in_range = in_x_range & in_y_range

                is_ob = 1 if np.any(in_range) else 0
                self.get_logger().info(f"장애물 필터링: 전체 장애물 {len(obstacles)}개 중, 박스 내 {np.sum(in_range)}개 있음")
                if not is_ob:
                    #TODO: ESTOP 구현
                    self.get_logger().info('No filtered cone poses... so start finding paths')
                    start_yaw = self.euler_from_quaternion(self.current_pose.orientation)
                    start = [
                        self.current_pose.position.x - 10,
                        self.current_pose.position.y,
                        start_yaw]
    
                    # 목표점 설정 (현재 위치에서 3m 앞)######
                    # ########################localization point ###############################
                    goal = [self.current_pose.position.x - 2 , self.current_pose.position.y + 3.0, start_yaw]
                    #########################localization point###############################
    
                    # Hybrid A* 경로 계획
    
                    path = hybrid_a_star_planning(start, goal, self.ox, self.oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

                if path:
                    # ROS Path 메시지로 변환
                    self.ros_path = Path()
                    self.ros_path.header.frame_id = 'map'
                    self.ros_path.header.stamp = self.get_clock().now().to_msg()

                    for x, y, yaw in zip(path.x_list, path.y_list, path.yaw_list):
                        pose = PoseStamped()
                        pose.header = self.ros_path.header
                        pose.pose.position.x = x
                        pose.pose.position.y = y
                        pose.pose.orientation = self.quaternion_from_yaw(yaw)
                        self.ros_path.poses.append(pose)

                    # 경로 발행
                    self.path_publisher.publish(self.ros_path)
                    self.get_logger().info(f'Path published with {len(path.x_list)} points')
                    self.flag_find_route=True
                else:
                    self.get_logger().warn('No path found')
            else:
                # TODO: 앞으로 가는 코드 짜기
                self.get_logger().debug("go ahead idiot")
                pass
        else:
            self.get_logger().debug("it already make paths")



def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

