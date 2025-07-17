import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import math
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import time as t
from erp42_msgs.msg import ControlMessage
from scipy.interpolate import CubicSpline

# from triangle import DelaunayTriangle
from scipy.spatial import Delaunay, distance_matrix
from scipy.spatial.distance import cdist


class State:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0


class Obstacle(Node):
    def __init__(self):
        super().__init__("avoid_path_publisher")

        # 구독
        self.create_subscription(String, "/current_mission", self.mission_callback, 10)
        self.create_subscription(Path, "/global_path", self.global_path_callback, 10)
        self.create_subscription(MarkerArray, "/markers", self.obstacle_callback, 10)
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10
        )
        # 퍼블리셔
        self.local_path_pub = self.create_publisher(Path, "/local_path", 10)
        self.cmd_pub = self.create_publisher(ControlMessage, "cmd_msg", 10)

        self.state = ""
        self.global_x = []
        self.global_y = []
        self.global_yaw = []

        self.local_x = []
        self.local_y = []
        self.local_yaw = []

        self.start = []
        self.goal = []
        self.obstacles = []
        self.threshold = 0.5

        self.odom = State()

        self.timer = None  # 타이머 초기화 (아직 실행 안 함)

    def global_path_callback(self, msg):
        """global_path를 받아서 저장"""
        if len(self.global_x) == 0 or self.global_x[0] != msg.poses[0].pose.position.x:
            self.global_x = []
            self.global_y = []
            self.global_yaw = []

            for pose in msg.poses:
                self.global_x.append(pose.pose.position.x)
                self.global_y.append(pose.pose.position.y)

                # orientation을 yaw로 변환
                qx, qy, qz, qw = (
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                )
                yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
                self.global_yaw.append(yaw)

            self.start = np.array(
                [self.global_x[0], self.global_y[0], self.global_yaw[0]]
            )
            self.goal = np.array([self.global_x[-1], self.global_y[-1]])

    def mission_callback(self, msg):
        self.state = msg.data
        if self.state == "U-Turn":
            self.run_uturn_path()
            self.start_timer()
        else:
            self.stop_timer()

    def odom_callback(self, msg):
        # Odometry 메시지에서 위치와 회전 정보를 추출
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # 로봇의 현재 위치
        self.odom.x = position.x
        self.odom.y = position.y

        # Quaternion을 yaw로 변환
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.odom.yaw = euler_from_quaternion(quaternion)[2]  # yaw (회전각)
        """미션 상태 업데이트"""

    def obstacle_callback(self, msg):
        # 프레임 맵으로 변환 해야함
        if msg.markers[0].header.frame_id != "map":
            print(msg.markers[0].header.frame_id)
            # MarkerArray 데이터 처리
            for marker in msg.markers:
                for point in marker.points:
                    map_x, map_y = self.velodyne_to_map(point.x, point.y)
                    self.add_obstacle(map_x, map_y)
        else:

            for marker in msg.markers:
                point = marker.pose.position
                self.add_obstacle(point.x, point.y)

    def velodyne_to_map(self, x, y):
        """Velodyne 기준 좌표를 map 기준 좌표로 변환"""
        robot_x, robot_y, robot_yaw = self.robot
        x += 1
        # 벨로다인 좌표를 로봇 기준에서 맵 기준으로 변환
        map_x = robot_x + x * math.cos(robot_yaw) - y * math.sin(robot_yaw)
        map_y = robot_y + x * math.sin(robot_yaw) + y * math.cos(robot_yaw)

        return map_x, map_y

    def add_obstacle(self, x, y, threshold=1.0):
        # 기존 장애물들과 비교하여 가까운 점들은 필터링
        new_obstacle = np.array([x, y])  # np.array로 변경
        if not self.obstacles:
            self.obstacles.append(new_obstacle)  # 여전히 리스트로 저장
        else:
            for obs in self.obstacles:
                if (
                    np.linalg.norm(new_obstacle - obs) < threshold
                ):  # 비교 시 np.array로 변환
                    return  # 가까운 점이 있는 경우 추가하지 않음
            self.obstacles.append(new_obstacle)  # 여전히 리스트로 저장

        # 장애물 리스트를 self.start에서 가까운 순으로 정렬
        self.obstacles.sort(
            key=lambda obs: np.linalg.norm(np.array(self.start[:2]) - obs)
        )

        # print(self.obstacles)

    def run_uturn_path(self):
        if len(self.start) != 0 and len(self.obstacles) >= 3:
            print("🔹 Start:", self.start)
            print("🔹 Obstacles:", len(self.obstacles))

            tri = Delaunay(self.obstacles)
            midpoints = set()

            for simplex in tri.simplices:
                edges = [
                    (simplex[0], simplex[1]),
                    (simplex[1], simplex[2]),
                    (simplex[2], simplex[0]),
                ]
                edge_lengths = [
                    np.linalg.norm(self.obstacles[e[0]] - self.obstacles[e[1]])
                    for e in edges
                ]
                longest_edges = sorted(zip(edge_lengths, edges))[-2:]

                for _, edge in longest_edges:
                    mid = tuple(
                        (
                            (self.obstacles[edge[0]] + self.obstacles[edge[1]]) / 2
                        ).tolist()
                    )
                    midpoints.add(mid)

            midpoints = np.array(list(midpoints))
            print("🔹 Midpoints created:", len(midpoints))

            start_x, start_y, start_yaw = self.start
            direction_vector = np.array([np.cos(start_yaw), np.sin(start_yaw)])
            yaw_threshold = 0.2

            filtered_midpoints = []
            for point in midpoints:
                point_x, point_y = point
                vector_to_point = np.array([point_x - start_x, point_y - start_y])

                vector_norm = np.linalg.norm(vector_to_point)
                similarity = (
                    0
                    if vector_norm == 0
                    else np.dot(vector_to_point, direction_vector)
                    / (vector_norm * np.linalg.norm(direction_vector))
                )

                if similarity > np.cos(yaw_threshold):
                    filtered_midpoints.append(point)

            print("🔹 Filtered Midpoints:", len(filtered_midpoints))

            if not filtered_midpoints:
                print("❌ No valid midpoints found.")
                return

            distances = [
                np.linalg.norm(np.array(p) - np.array([start_x, start_y]))
                for p in filtered_midpoints
            ]
            nearest_point = sorted(
                zip(distances, filtered_midpoints), key=lambda x: x[0]
            )[0][1]

            path = [tuple(nearest_point)]
            unvisited = {tuple(point) for point in midpoints}
            unvisited.discard(tuple(nearest_point))

            print("🔹 Nearest Point:", nearest_point)
            print("🔹 Unvisited Points:", len(unvisited))

            while unvisited:
                last_point = path[-1]
                nearest = min(
                    unvisited,
                    key=lambda i: np.linalg.norm(np.array(i) - np.array(last_point)),
                )
                path.append(nearest)
                unvisited.discard(nearest)

            print("🔹 Final Path Length:", len(path))

            if len(path) < 3:
                print("❌ Path is too short.")
                return

            print("🔹 Path Before Obstacle Removal:", len(path))
            path = self.remove_nearby_obstacles(
                np.array(path), np.array(self.obstacles), min_distance=3  # 거리 조정
            )
            print("🔹 Path After Obstacle Removal:", len(path))

            if len(path) > 3:
                print("🔹 Path Before Interpolation:", len(path))
                path = self.interpolate_path(path, step_size=0.5)
                print("🔹 Path After Interpolation:", len(path))

                self.local_x = [point[0] for point in path]
                self.local_y = [point[1] for point in path]
                self.local_yaw = [
                    math.atan2(path[i][1] - path[i - 1][1], path[i][0] - path[i - 1][0])
                    for i in range(1, len(path))
                ]
                self.local_yaw.insert(0, self.local_yaw[0])

        self.publish_path()


    def remove_nearby_obstacles(self, path, obstacles, min_distance=5):
        # 장애물과 경로 점들 간의 거리 계산
        filtered_path = []  # 새로 필터링된 경로 저장
        for point in path:
            distances = np.linalg.norm(
                obstacles - point, axis=1
            )  # 장애물과의 거리 계산
            if np.all(
                distances > min_distance
            ):  # 모든 장애물과의 거리가 5보다 큰 경우에만 추가
                filtered_path.append(point)
        return np.array(filtered_path)

    def interpolate_path(self, path, step_size=0.5):
        """
        경로를 0.1m 간격으로 점을 추가하여 경로를 보간합니다.
        """
        new_path = [path[0]]  # 시작점 추가

        for i in range(1, len(path)):
            start_point = path[i - 1]
            end_point = path[i]

            # 두 점 사이의 거리 계산
            dist = np.linalg.norm(np.array(end_point) - np.array(start_point))
            num_steps = int(np.ceil(dist / step_size))

            # 0.1m 간격으로 점을 추가
            for j in range(1, num_steps):
                t = j / num_steps
                new_point = (
                    start_point[0] + t * (end_point[0] - start_point[0]),
                    start_point[1] + t * (end_point[1] - start_point[1]),
                )
                new_path.append(new_point)

            new_path.append(end_point)  # 마지막 점 추가

        return np.array(new_path)

    def moving_average_path(self, trajectory, window_size=20):
        """경로를 부드럽게 만드는 함수"""
        smoothed_trajectory = []
        for i in range(len(trajectory)):
            start_idx = max(0, i - window_size // 2)
            end_idx = min(len(trajectory), i + window_size // 2 + 1)
            window = trajectory[start_idx:end_idx]
            smoothed_trajectory.append(np.mean(window, axis=0))
        return np.array(smoothed_trajectory)

    def start_timer(self):
        """타이머 시작 (이미 실행 중이면 무시)"""
        if self.timer is None:
            self.timer = self.create_timer(0.001, self.timercallback)

    def stop_timer(self):
        """타이머 정지"""
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

    def timercallback(self):
        """local_path를 퍼블리시"""
        if self.state != "obstacle":
            return  # 현재 미션이 driving이 아니면 퍼블리시 안 함

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        if len(self.local_x) > 0:
            x_list, y_list, yaw_list = self.local_x, self.local_y, self.local_yaw
        elif len(self.global_x) > 0:
            x_list, y_list, yaw_list = self.global_x, self.global_y, self.global_yaw
        else:
            return  # 퍼블리시할 데이터가 없으면 종료

        for x, y, yaw in zip(x_list, y_list, yaw_list):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

        self.local_path_pub.publish(path_msg)

    def publish_path(self):
        if self.state != "U-Turn":
            return  # 현재 미션이 driving이 아니면 퍼블리시 안 함

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        if len(self.local_x) > 0:
            x_list, y_list, yaw_list = self.local_x, self.local_y, self.local_yaw
        elif len(self.global_x) > 0:
            x_list, y_list, yaw_list = self.global_x, self.global_y, self.global_yaw
        else:
            return  # 퍼블리시할 데이터가 없으면 종료

        for x, y, yaw in zip(x_list, y_list, yaw_list):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

        self.local_path_pub.publish(path_msg)

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """쿼터니언을 yaw로 변환"""
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        """yaw를 쿼터니언으로 변환"""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)


def main(args=None):
    rclpy.init(args=args)
    node = Obstacle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()