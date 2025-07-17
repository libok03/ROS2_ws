import rclpy
import numpy as np
import numpy as np
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from scipy.spatial.distance import cdist
from erp42_msgs.msg import ControlMessage
from erp42_msgs.msg import SerialFeedBack
from collections import deque

class ObstacleKalmanFilter:
    """장애물 추적용 칼만 필터 (2D 위치/속도)"""
    def __init__(self, initial_pose, initial_time):
        self.dt = 0.1  # 기본 시간 간격
        self.state = np.array([
            [initial_pose.position.x],
            [initial_pose.position.y],
            [0.0],
            [0.0]
        ])
        self.P = np.eye(4) * 10.0 # 초기 오차 공분산
        self.F = np.eye(4) # 상태 전이 행렬
        self.F[0, 2] = self.dt  
        self.F[1, 3] = self.dt
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ]) #측정 행렬
        self.Q = np.eye(4) * 0.05 #프로세스 노이즈 공분산
        self.R = np.eye(2) * 100.0 # 측정 노이즈 공분산
        self.last_time = initial_time

    def predict(self, dt):
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement):
        z = np.array([[measurement.position.x], [measurement.position.y]])
        y = z - self.H @ self.state
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def get_position(self):
        return self.state[0,0], self.state[1,0]

    def get_velocity(self):
        return self.state[2,0], self.state[3,0]


class State:
    """로봇의 현재 상태를 저장하는 클래스"""
    def __init__(self):
        self.x = 0.0    # x 좌표 (미터)
        self.y = 0.0    # y 좌표 (미터)
        self.yaw = 0.0  # 방향 (라디안)
        self.v = 0.0    # 속도 (m/s)

class PathPlannerNode(Node):
    """동적/정적 장애물 추적 및 경로 계획을 수행하는 ROS2 노드"""
    
    def __init__(self):
        super().__init__('path_planner_node')
        self._next_id = 1  # ID 생성 카운터 추가
        # 추적 중인 장애물 정보 저장 딕셔너리
        # {장애물 ID: {'pose_history': 포즈 히스토리, 'stamp_history': 타임스탬프, 'missed_count': 미감지 횟수}}
        self.tracked_obstacles = {}
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_association_dist', 1.0),  # 최대 연동 거리 (m)
                ('velocity_threshold', 0.5),    # 동적 장애물 속도 임계값 (m/s)
                ('min_detection_frames', 3),    # 최소 감지 프레임 수
                ('max_missed_frames', 3)        # 최대 미감지 프레임 수
            ]
        )
        
        # 장애물 위치 정보 서브스크라이버 (맵 좌표계 기준)
        self.conesub = self.create_subscription(
            PoseArray,
            "cone_poses_map",
            self.obstacle_callback,
            10
        )
        
        self.obstacle_pub = self.create_publisher(
            PoseArray,
            "dynamic_obstacles",
            10
        )
        
    # 나중에 위치를 이용해서 경로 생성할때 살리기!
    #     # 로봇 현재 위치 정보 서브스크라이버
    #     self.create_subscription(
    #         Odometry,
    #         "/localization/kinematic_state",
    #         self.odom_callback,
    #         10
    #     )
        
    # def odom_callback(self, msg):
    #     """로봇의 오도메트리 정보 콜백 함수"""
    #     # 월드 좌표계 변환을 위한 로봇 위치 저장
    #     self.robot_pose = msg.pose.pose
        
    def obstacle_callback(self, msg):
        """장애물 위치 정보 콜백 함수"""
        current_time = self.get_clock().now().to_msg()
        
        # 1. 데이터 연동 (최근접 이웃 매칭 + 프레임 지속성 관리)
        self.update_tracks(msg.poses, current_time)
        
        # 2. 유효 장애물 필터링 (최소 감지 프레임 수 조건 충족 여부)
        valid_obstacles = self.filter_valid_obstacles()
        
        # 3. 동적/정적 분류 및 결과 발행
        self.publish_results(valid_obstacles)


    def update_tracks(self, current_poses, current_time):
        matched_ids = set()
        unmatched_poses = current_poses.copy()  # 미매칭 포즈 추적

        # 1단계: 기존 트랙 매칭 시도
        for oid in list(self.tracked_obstacles.keys()):
            best_match = None
            min_dist = self.get_parameter('max_association_dist').value

            for idx, pose in enumerate(unmatched_poses):
                dist = self.calculate_distance(pose, self.tracked_obstacles[oid]['pose_history'][-1])
                if dist < min_dist:
                    best_match = idx
                    min_dist = dist

            if best_match is not None:
                self.update_existing_track(oid, unmatched_poses.pop(best_match), current_time)
                matched_ids.add(oid)

        # 2단계: 남은 포즈에 대해 새 트랙 생성
        for pose in unmatched_poses:
            self.create_new_track(pose, current_time)

        # 3단계: 미매칭 트랙 처리
        for oid in list(self.tracked_obstacles.keys()):
            if oid not in matched_ids:
                self.tracked_obstacles[oid]['missed_count'] += 1
                if self.tracked_obstacles[oid]['missed_count'] > self.get_parameter('max_missed_frames').value:
                    del self.tracked_obstacles[oid]

    def filter_valid_obstacles(self):
        """최소 감지 프레임 수 이상인 장애물만 필터링"""
        return {
            oid: data for oid, data in self.tracked_obstacles.items() 
            if len(data['pose_history']) >= self.get_parameter('min_detection_frames').value
        }

    def publish_results(self, valid_obstacles):
        """속도 기반 장애물 필터링 및 발행"""
        fast_obstacles = {}
        for oid, data in valid_obstacles.items():
            kf = data['kf']
            velocity = self.calculate_velocity(kf)
            if velocity >= self.get_parameter("velocity_threshold").value:
                # 칼만 필터의 위치를 ROS Pose로 변환
                x, y = kf.get_position()
                pose = data['pose_history'][-1]  # 기존 포즈 복사
                pose.position.x = x
                pose.position.y = y
                fast_obstacles[oid] = {
                    'pose': pose,
                    'velocity': velocity
                }


    def calculate_distance(self, pose1, pose2):
        """두 포즈 사이의 유클리드 거리 계산"""
        return np.sqrt(
            (pose1.position.x - pose2.position.x)**2 +
            (pose1.position.y - pose2.position.y)**2 +
            (pose1.position.z - pose2.position.z)**2
        )

    def calculate_velocity(self, kf):
        vx, vy = kf.get_velocity()
        return math.hypot(vx, vy)



    def update_existing_track(self, oid, pose, stamp):
        track = self.tracked_obstacles[oid]
        # 시간 간격 계산
        prev_stamp = track['stamp_history'][-1]
        dt = (stamp.sec - prev_stamp.sec) + (stamp.nanosec - prev_stamp.nanosec) * 1e-9
        if dt <= 0: dt = 1e-2  # 최소값 보장

        # 칼만 필터 예측 및 업데이트
        track['kf'].predict(dt)
        track['kf'].update(pose)
        track['pose_history'].append(pose)
        track['stamp_history'].append(stamp)
        track['missed_count'] = 0


    def create_new_track(self, pose, stamp):
        new_id = self._next_id
        self._next_id += 1
        self.tracked_obstacles[new_id] = {
            'pose_history': deque([pose], maxlen=15),
            'stamp_history': deque([stamp], maxlen=15),
            'missed_count': 0,
            'kf': ObstacleKalmanFilter(pose, stamp)  # 칼만 필터 객체 추가
        }


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
