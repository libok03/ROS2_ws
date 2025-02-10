#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path, Odometry
from scipy.spatial import KDTree
import math

class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')
        # 장애물 정보는 PoseArray 형식으로 수신 (각 Pose의 position.x, y 이용)
        self.subscription = self.create_subscription(
            PoseArray,
            'cone_poses_map',
            self.obstacles_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odom_callback,
            10
        )
        # 생성된 경로는 nav_msgs/Path 메시지로 퍼블리시
        self.publisher = self.create_publisher(Path, 'planned_path', 10)


        # 장애물 리스트 (실수 좌표 튜플)
        self.obstacles = []

        # 시작 및 목표 위치 (예제에서는 (0,0)에서 (10,10))
        self.start = None
        self.goal_point = (21.4, 108.95)

        # 잠재력 기반 경로 생성을 위한 파라미터들
        self.k_att = 2.0            # 인력 계수 (목표를 향한 힘)
        self.k_rep = 5.0          # 반발력 계수 (장애물로부터의 힘)
        self.repulsive_range = 3  # 장애물이 영향을 주는 거리 범위
        self.step_size = 0.1    # 한 스텝 당 이동 거리
        self.threshold = 2      # 목표 도달 시 오차 범위

        # 주기적으로 경로 생성 실행 (1초)
        self.timer = self.create_timer(1.0, self.plan_path)

    def obstacles_callback(self, msg):
        if msg.poses:
            # 모든 콘의 위치를 [x, y] 형식의 리스트로 추출
            new_cones = [[pose.position.x, pose.position.y] for pose in msg.poses]
            finali_cones = []

            if self.obstacles:
                # 기존 장애물 리스트에 대해 KDTree를 생성 (O(m log m))
                kd_tree = KDTree(self.obstacles)
                # 각 새로운 콘에 대해 가장 가까운 장애물과의 거리를 조회 (O(n log m))
                for cone in new_cones:
                    dist, idx = kd_tree.query(cone)
                    if dist > 1.5:
                        finali_cones.append(cone)
            else:
                finali_cones = new_cones

        # 장애물 리스트 업데이트: 새로 추가할 콘이 있다면 기존 리스트에 누적
        if finali_cones:
            self.obstacles.extend(finali_cones)

        self.get_logger().info(f"New obstacles added: {len(finali_cones)}")
        # 장애물 정보를 PoseArray 형태로 퍼블리시 (RViz 확인용)

    def odom_callback(self, msg: Odometry):
        # odom 메시지에서 현재 위치 추출하여 start 업데이트
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.start = (x, y)



    def odom_callback(self, msg: Odometry):
        # odom 메시지에서 로봇의 현재 위치를 추출하여 시작 위치로 설정
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.start = (x, y)
        # self.get_logger().info(f"Current start position updated from odom: {self.start}")

    def plan_path(self):
        # 시작 위치가 업데이트되지 않았으면 경로 계획을 진행하지 않음
        if self.start is None:
            self.get_logger().warn("시작 위치가 아직 업데이트되지 않았습니다.")
            return

        self.get_logger().info(f"Planning path from {self.start} to {self.goal_point}...")
        current = self.start
        path_points = [current]
        max_iter = 10000   # 무한 루프 방지를 위한 최대 반복 횟수
        i = 0

        # 목표에 도달할 때까지 또는 최대 반복 횟수 내에서 힘 벡터 계산 후 보폭 이동
        while self.distance(current, self.goal_point) > self.threshold and i < max_iter:
            force = self.compute_total_force(current)
            norm = math.hypot(force[0], force[1])
            if norm == 0:
                self.get_logger().warn("힘 벡터가 0입니다. 지역 최소에 빠졌을 수 있습니다.")
                break
            # 힘 벡터의 단위 방향 계산 후 한 스텝만큼 이동
            direction = (force[0] / norm, force[1] / norm)
            next_point = (current[0] + self.step_size * direction[0],
                          current[1] + self.step_size * direction[1])
            path_points.append(next_point)
            current = next_point
            i += 1

        self.publish_path(path_points)
        if self.distance(current, self.goal_point) <= self.threshold:
            # self.get_logger().info(f"경로 생성 완료. 총 {len(path_points)} 포인트")
            self.publish_path(path_points)
            self.last_path_points = path_points
        else:
            self.get_logger().warn("최대 반복 횟수에 도달했습니다. 목표 도달에 실패했습니다.")


    def compute_total_force(self, pos):
        # 인력: 목표로 향하는 힘 (선형)
        f_att = (self.k_att * (self.goal_point[0] - pos[0]),
                 self.k_att * (self.goal_point[1] - pos[1]))
        # 반발력: 각 장애물로부터 작용하는 힘의 합
        f_rep = (0.0, 0.0)
        for obs in self.obstacles:
            dx = pos[0] - obs[0]
            dy = pos[1] - obs[1]
            dist = math.hypot(dx, dy)
            if dist < 1e-6:
                continue
            if dist <= self.repulsive_range:
                rep_mag = self.k_rep * (1.0/dist - 1.0/self.repulsive_range) / (dist**2)
                f_rep = (f_rep[0] + rep_mag * (dx/dist),
                         f_rep[1] + rep_mag * (dy/dist))
        # 총 힘은 인력과 반발력의 합
        f_total = (f_att[0] + f_rep[0], f_att[1] + f_rep[1])
        return f_total

    def distance(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def publish_path(self, points):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for pt in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = 0.0
            # 기본 orientation (회전 없음)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.publisher.publish(path_msg)
        self.get_logger().info("생성된 경로를 'planned_path' 토픽으로 퍼블리시했습니다.")

def main(args=None):
    rclpy.init(args=args)
    planner = PotentialFieldPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
