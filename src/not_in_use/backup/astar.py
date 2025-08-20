#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2
from heapq import heappush, heappop

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        # 파라미터 설정
        self.declare_parameters(namespace='', parameters=[
            ('obstacle_threshold', 50),
            ('heuristic_weight', 1.2),
            ('max_iterations', 50000),
        ])

        # TF 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 서브스크라이버 & 퍼블리셔
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/tunnel_costmap', self.costmap_cb, 10)
        self.pc_sub = self.create_subscription(
            PointCloud2, '/cropbox_filtered', self.pc_cb, 10)
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)

        # 변수 초기화
        self.costmap = None
        self.goal = None
        self.origin = None
        self.resolution = 0.1
        self.width = 600
        self.height = 100

    def pc_cb(self, msg):
        try:
            # 포인트 클라우드에서 x,y,z 추출 (float 강제 변환)
            points = pc2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            )

            # 모든 좌표를 float32로 명시적 변환
            transformed = [
                (float(x), float(y)) 
                for x, y, _ in points  # z값 무시
            ]

            # 목표점 계산
            if len(transformed) < 10:
                self.get_logger().warn("포인트 개수 부족")
                return

            xy_array = np.array(transformed, dtype=np.float32)
            distances = np.linalg.norm(xy_array, axis=1)
            farthest_idx = np.argpartition(distances, -10)[-10:]
            self.goal = np.mean(xy_array[farthest_idx], axis=0)

            self.run_astar()

        except Exception as e:
            self.get_logger().error(f"처리 실패: {str(e)}")

    def costmap_cb(self, msg):
        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.width = msg.info.width
        self.height = msg.info.height
        self.get_logger().info(f"코스트맵 원점: {self.origin}")

    def world_to_grid(self, x, y):
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return (np.clip(gx, 0, self.width-1), np.clip(gy, 0, self.height-1))

    def grid_to_world(self, gx, gy):
        return (
            gx * self.resolution + self.origin[0],
            gy * self.resolution + self.origin[1]
        )

    def heuristic(self, a, b):
        return np.hypot(a[0]-b[0], a[1]-b[1]) * self.get_parameter('heuristic_weight').value

    def run_astar(self):
        if self.costmap is None or self.goal is None: 
            return

        obstacle_thresh = self.get_parameter('obstacle_threshold').value
        start = self.world_to_grid(0.0, 0.0)
        goal_grid = self.world_to_grid(self.goal[0], self.goal[1])

        # 디버깅 정보
        self.get_logger().info(f"시작점(grid): {start} / 목표점(grid): {goal_grid}")

        if not (0 <= goal_grid[0] < self.width and 0 <= goal_grid[1] < self.height):
            self.get_logger().error("목표점이 맵 밖에 있습니다")
            return

        open_heap = []
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal_grid)}
        heappush(open_heap, (f_score[start], start))

        for _ in range(self.get_parameter('max_iterations').value):
            if not open_heap: 
                break

            current = heappop(open_heap)[1]
            if current == goal_grid:
                self.reconstruct_path(came_from, current)
                return

            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0: 
                        continue
                    neighbor = (current[0]+dx, current[1]+dy)

                    if not (0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height):
                        continue
                    if self.costmap[neighbor[1], neighbor[0]] >= obstacle_thresh:
                        continue

                    tentative_g = g_score[current] + np.hypot(dx, dy)
                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_grid)
                        heappush(open_heap, (f_score[neighbor], neighbor))

        self.get_logger().warn("경로를 찾을 수 없음!")

    def reconstruct_path(self, came_from, current):
        """경로를 Path 메시지로 변환"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'base_link'  # 프레임 ID 설정

        # 경로 역순으로 채우기 (목표점 → 시작점)
        temp_path = []
        while current in came_from:
            x, y = self.grid_to_world(*current)
            pose = PoseStamped()
            pose.header = path.header  # 동일한 헤더 사용
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0  # 기본 방향
            temp_path.append(pose)
            current = came_from[current]

        # 시작점 → 목표점 순서로 뒤집기
        path.poses = temp_path[::-1]

        self.path_pub.publish(path)
        self.get_logger().info(f"경로 발행: {len(path.poses)} 포인트")
        return path

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
