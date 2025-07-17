#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class DWAPathPlanner(Node):
    def __init__(self):
        super().__init__('dwa_path_planner')
        
        # 파라미터 설정
        self.declare_parameters(namespace='', parameters=[
            ('max_vel', 0.6), ('min_vel', -0.2),
            ('max_yawrate', 1.5), ('sim_time', 1.5),
            ('v_samples', 20), ('yaw_samples', 40),
            ('obstacle_thresh', 50), ('goal_weight', 5.0),
            ('path_weight', 2.0), ('epsilon', 1e-8)
        ])
        
        # 서브스크라이버 & 퍼블리셔
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/tunnel_costmap', self.costmap_cb, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/cropbox_filtered', self.pc_cb, 10)
        self.path_pub = self.create_publisher(Path, '/dwa_path', 10)
        
        # 상태 변수 초기화
        self.goal = None
        self.costmap = None
        self.origin = None
        self.resolution = 0.1
        self.width = 600
        self.height = 100
        
        # 제어 주기 설정 (10Hz)
        self.create_timer(0.1, self.dwa_planning)

    def pc_cb(self, msg):
        try:
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array([[float(p[0]), float(p[1])] for p in points])
            if len(points) < 10: return
            
            # 목표점 계산 (가장 먼 10개 점 평균)
            distances = np.linalg.norm(points, axis=1)
            self.goal = np.mean(points[np.argsort(distances)[-10:]], axis=0)
            
        except Exception as e:
            self.get_logger().error(f"목표점 오류: {str(e)}")

    def costmap_cb(self, msg):
        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height

    def world_to_grid(self, x, y):
        """월드 → 그리드 좌표 변환"""
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return (np.clip(gx, 0, self.width-1), np.clip(gy, 0, self.height-1))

    def predict_trajectory(self, v, w):
        """궤적 예측"""
        dt = 0.1
        steps = int(self.get_parameter('sim_time').value / dt)
        traj = []
        x, y, theta = 0.0, 0.0, 0.0  # base_link 기준
        
        for _ in range(steps):
            theta += w * dt
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            traj.append((x, y))
        return traj

    def collision_check(self, traj):
        """충돌 검사"""
        for (x, y) in traj:
            gx, gy = self.world_to_grid(x, y)
            if self.costmap[gy, gx] >= self.get_parameter('obstacle_thresh').value:
                return True
        return False

    def calculate_score(self, traj):
        """궤적 점수 계산"""
        if len(traj) == 0:
            return -float('inf')
            
        # 목표 방향 점수
        end_pos = traj[-1]
        goal_vec = np.array(self.goal)
        current_vec = np.array(end_pos)
        
        eps = self.get_parameter('epsilon').value
        goal_norm = goal_vec / (np.linalg.norm(goal_vec) + eps)
        current_norm = current_vec / (np.linalg.norm(current_vec) + eps)
        direction_score = np.dot(goal_norm, current_norm)
        
        # 경로 효율 점수
        path_score = np.linalg.norm(goal_vec - current_vec)
        
        return self.get_parameter('goal_weight').value * direction_score \
               - self.get_parameter('path_weight').value * path_score

    def dwa_planning(self):
        if self.goal is None or self.costmap is None:
            return

        best_score = -float('inf')
        best_traj = []
        
        # 속도 샘플링 범위
        v_options = np.linspace(
            self.get_parameter('min_vel').value,
            self.get_parameter('max_vel').value,
            self.get_parameter('v_samples').value
        )
        
        w_options = np.linspace(
            -self.get_parameter('max_yawrate').value,
            self.get_parameter('max_yawrate').value,
            self.get_parameter('yaw_samples').value
        )

        # 모든 조합 평가
        for v in v_options:
            for w in w_options:
                traj = self.predict_trajectory(v, w)
                if self.collision_check(traj):
                    continue
                
                score = self.calculate_score(traj)
                if score > best_score:
                    best_score = score
                    best_traj = traj

        # 최적 경로 발행
        self.publish_path(best_traj)

    def publish_path(self, traj):
        """경로 메시지 생성"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'base_link'
        
        for (x, y) in traj:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = Point(x=x, y=y)
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"경로 발행: {len(traj)} 포인트")

def main(args=None):
    rclpy.init(args=args)
    node = DWAPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
