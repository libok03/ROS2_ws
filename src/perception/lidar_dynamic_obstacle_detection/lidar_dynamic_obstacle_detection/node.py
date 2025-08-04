# 모든장애물 (차량속도정보 적용X)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from sklearn.cluster import DBSCAN
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, String
import tf2_ros
from tf2_ros import TransformException
from collections import deque
import math
import time


class AccurateDynamicObstacleDetector(Node):
    def __init__(self):
        super().__init__("accurate_dynamic_obstacle_detector")

        # 파라미터 설정
        self.declare_parameters(
            namespace="",
            parameters=[
                ("cluster_eps", 0.5),
                ("cluster_min_samples", 5),
                ("min_dynamic_speed", 2.0),  # 동적 장애물 최소 속도 (m/s)
                ("max_static_speed", 1.9),  # 정적 장애물 최대 속도 (m/s)
                ("z_min", -0.5),
                ("z_max", 2.0),
                ("tracking_window", 5),
                ("required_consistent_frames", 1),
                ("transform_timeout", 0.1),
                ("base_frame", "base_link"),
                ("lidar_frame", "velodyne"),
                ("debug_mode", True),
            ],
        )

        # 서브스크라이버
        self.sub_cloud = self.create_subscription(
            PointCloud2, "/cropbox_filtered", self.pointcloud_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odometry_callback, 10
        )

        # 퍼블리셔
        self.marker_pub = self.create_publisher(
            MarkerArray, "/accurate_dynamic_obstacles", 10
        )
        self.debug_pub = self.create_publisher(String, "/obstacle_debug", 10)

        # TF 리스너
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.time.Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 상태 변수
        self.vehicle_odom = None
        self.obstacle_tracks = {}
        self.obstacle_id_counter = 0
        self.last_time = time.time()
        self.last_position = np.zeros(3)

    def odometry_callback(self, msg):
        """차량 속도 및 위치 업데이트"""
        self.vehicle_odom = msg
        self.last_position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )

    def extract_points(self, cloud_msg):
        """포인트 클라우드 안전하게 추출"""
        try:
            points = []
            for p in point_cloud2.read_points(
                cloud_msg, field_names=("x", "y", "z"), skip_nans=True
            ):
                points.append([p[0], p[1], p[2]])
            return np.array(points, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"Point extraction failed: {str(e)}")
            return np.empty((0, 3), dtype=np.float32)

    def pointcloud_callback(self, msg):
        try:
            if self.vehicle_odom is None:
                return

            current_time = time.time()
            dt = current_time - self.last_time
            if dt <= 0.001:  # 최소 시간 간격 보장
                return
            self.last_time = current_time

            # TF 변환 (LiDAR → base_link)
            try:
                lidar_to_base = self.tf_buffer.lookup_transform(
                    self.get_parameter("base_frame").value,
                    self.get_parameter("lidar_frame").value,
                    rclpy.time.Time(),
                    timeout=rclpy.time.Duration(
                        seconds=self.get_parameter("transform_timeout").value
                    ),
                )
            except TransformException as e:
                self.get_logger().warn(f"TF error: {str(e)}", throttle_duration_sec=1)
                return

            # 포인트 클라우드 처리
            points = self.extract_points(msg)
            if len(points) == 0:
                return

            # 높이 필터링
            z_filter = (points[:, 2] > self.get_parameter("z_min").value) & (
                points[:, 2] < self.get_parameter("z_max").value
            )
            filtered_points = points[z_filter]

            if len(filtered_points) < 5:  # 최소 포인트 수
                return

            # base_link 좌표계 변환
            rotation = R.from_quat(
                [
                    lidar_to_base.transform.rotation.x,
                    lidar_to_base.transform.rotation.y,
                    lidar_to_base.transform.rotation.z,
                    lidar_to_base.transform.rotation.w,
                ]
            )
            base_link_points = rotation.apply(filtered_points) + np.array(
                [
                    lidar_to_base.transform.translation.x,
                    lidar_to_base.transform.translation.y,
                    lidar_to_base.transform.translation.z,
                ],
                dtype=np.float32,
            )

            # 클러스터링
            clustering = DBSCAN(
                eps=self.get_parameter("cluster_eps").value,
                min_samples=self.get_parameter("cluster_min_samples").value,
            ).fit(base_link_points)

            # 클러스터 추출
            current_clusters = {}
            for label in np.unique(clustering.labels_):
                if label == -1:
                    continue
                cluster_mask = clustering.labels_ == label
                cluster_points = base_link_points[cluster_mask]
                if len(cluster_points) >= 5:  # 클러스터 최소 포인트 수
                    current_clusters[label] = {
                        "center": np.median(cluster_points, axis=0),
                        "points": cluster_points,
                        "size": np.ptp(cluster_points, axis=0),
                        "time": current_time,
                    }

            # 장애물 추적
            self.update_tracks(current_clusters, current_time, dt)
            self.visualize_obstacles(msg.header)

        except Exception as e:
            self.get_logger().error(f"Main loop error: {str(e)}")

    def update_tracks(self, current_clusters, current_time, dt):
        """차량 운동 보상이 적용된 장애물 추적"""
        if self.vehicle_odom is None:
            return

        # 차량 속도 추출 (base_link 기준)
        vehicle_velocity = np.array(
            [
                self.vehicle_odom.twist.twist.linear.x,
                self.vehicle_odom.twist.twist.linear.y,
                self.vehicle_odom.twist.twist.linear.z,
            ],
            dtype=np.float32,
        )

        angular_velocity = self.vehicle_odom.twist.twist.angular.z
        debug_msg = String()

        # 트랙 매칭 및 업데이트
        for label, cluster in current_clusters.items():
            best_match_id = None
            min_distance = float("inf")

            # 가장 가까운 기존 트랙 찾기
            for track_id, track in self.obstacle_tracks.items():
                dist = np.linalg.norm(cluster["center"] - track["current_position"])
                if dist < min_distance and dist < 1.0:  # 1m 이내만 매칭
                    min_distance = dist
                    best_match_id = track_id

            if best_match_id is not None:
                track = self.obstacle_tracks[best_match_id]
                displacement = cluster["center"] - track["current_position"]

                # 차량 운동 보상 (선속도 + 각속도)
                rotation_compensation = np.array(
                    [
                        -angular_velocity * displacement[1],
                        angular_velocity * displacement[0],
                        0,
                    ],
                    dtype=np.float32,
                )

                # 실제 장애물 속도 계산 (차량 운동 제거)
                obstacle_velocity = (displacement / dt) + (
                    vehicle_velocity + rotation_compensation
                )
                obstacle_speed = np.linalg.norm(obstacle_velocity[:2])  # 2D 속도만 고려

                # 속도 기록 업데이트
                track["speed_history"].append(obstacle_speed)
                if (
                    len(track["speed_history"])
                    > self.get_parameter("tracking_window").value
                ):
                    track["speed_history"].popleft()

                avg_speed = (
                    np.mean(track["speed_history"]) if track["speed_history"] else 0.0
                )

                # 동적/정적 분류
                if avg_speed > self.get_parameter("min_dynamic_speed").value:
                    track["dynamic_frames"] += 1
                else:
                    track["dynamic_frames"] = max(0, track["dynamic_frames"] - 1)

                # 트랙 상태 업데이트
                track.update(
                    {
                        "current_position": cluster["center"],
                        "last_update_time": current_time,
                        "size": cluster["size"],
                        "is_dynamic": track["dynamic_frames"]
                        >= self.get_parameter("required_consistent_frames").value,
                        "current_velocity": obstacle_velocity,
                        "avg_speed": avg_speed,
                    }
                )

                if self.get_parameter("debug_mode").value:
                    debug_msg.data += (
                        f"Track {best_match_id}: "
                        f"Speed={avg_speed:.2f}m/s "
                        f"(Vehicle={np.linalg.norm(vehicle_velocity):.2f}m/s)\n"
                    )

            else:
                # 새로운 트랙 생성
                self.obstacle_tracks[self.obstacle_id_counter] = {
                    "current_position": cluster["center"],
                    "last_update_time": current_time,
                    "size": cluster["size"],
                    "speed_history": deque(
                        [0.0], maxlen=self.get_parameter("tracking_window").value
                    ),
                    "dynamic_frames": 0,
                    "is_dynamic": False,
                    "current_velocity": np.zeros(3, dtype=np.float32),
                    "avg_speed": 0.0,
                }
                self.obstacle_id_counter += 1

        # 오래된 트랙 제거 (1초 이상 미감지)
        self.obstacle_tracks = {
            k: v
            for k, v in self.obstacle_tracks.items()
            if current_time - v["last_update_time"] <= 1.0
        }

        if debug_msg.data:
            self.debug_pub.publish(debug_msg)

    def visualize_obstacles(self, header):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for track_id, track in self.obstacle_tracks.items():
            # 정적 장애물 중 저속 객체는 제외
            if not track["is_dynamic"] and track["avg_speed"] < 0.1:
                continue

            # 장애물 마커 (큐브)
            marker = Marker()
            marker.header = header
            marker.header.frame_id = self.get_parameter("base_frame").value
            marker.ns = "obstacles"
            marker.id = track_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = Point(
                x=float(track["current_position"][0]),
                y=float(track["current_position"][1]),
                z=float(track["current_position"][2]),
            )
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(max(track["size"][0], 0.3))  # 최소 크기
            marker.scale.y = float(max(track["size"][1], 0.3))
            marker.scale.z = float(max(track["size"][2], 0.3))
            marker.color = ColorRGBA(
                r=1.0 if track["is_dynamic"] else 0.5,  # 동적:빨강, 정적:회색
                g=0.0,
                b=0.0,
                a=0.7,
            )
            marker.lifetime.nanosec = int(0.5 * 1e9)
            marker_array.markers.append(marker)

            # 속도 벡터 (0.3m/s 이상만 표시)
            if track["avg_speed"] >= 0.3:
                vel_marker = Marker()
                vel_marker.header = header
                vel_marker.ns = "velocity"
                vel_marker.id = track_id + 10000
                vel_marker.type = Marker.ARROW
                vel_marker.action = Marker.ADD

                start = Point(
                    x=float(track["current_position"][0]),
                    y=float(track["current_position"][1]),
                    z=float(track["current_position"][2]),
                )

                # 속도 벡터 정규화 및 스케일 조정
                vel_dir = track["current_velocity"] / (
                    np.linalg.norm(track["current_velocity"]) + 1e-6
                )
                end = Point(
                    x=start.x + vel_dir[0] * 0.5,  # 고정 길이 (0.5m)
                    y=start.y + vel_dir[1] * 0.5,
                    z=start.z + vel_dir[2] * 0.2,
                )  # Z축은 짧게

                vel_marker.points = [start, end]
                vel_marker.scale.x = 0.1  # shaft diameter
                vel_marker.scale.y = 0.2  # head diameter
                vel_marker.color = ColorRGBA(
                    r=0.0 if track["is_dynamic"] else 0.5,
                    g=1.0 if track["is_dynamic"] else 0.5,
                    b=0.0,
                    a=0.9,
                )
                vel_marker.lifetime.nanosec = int(0.5 * 1e9)
                marker_array.markers.append(vel_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = AccurateDynamicObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
