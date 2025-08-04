#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2

import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException


class AccurateDynamicObstacleDetector(Node):
    def __init__(self):
        super().__init__("accurate_dynamic_obstacle_detector")

        # ────────────────────────────────────────────────────────────────────────
        # Parameters
        # ────────────────────────────────────────────────────────────────────────
        self.declare_parameters(
            namespace="",
            parameters=[
                ("cluster_eps", 0.5),
                ("cluster_min_samples", 5),
                ("min_dynamic_speed", 2.0),   # 동적 장애물 최소 속도 (m/s)
                ("max_static_speed", 1.9),    # 정적 장애물 최대 속도 (m/s) - 미사용 변수 (예비)
                ("z_min", -0.5),
                ("z_max", 2.0),
                ("tracking_window", 5),
                ("required_consistent_frames", 1),
                ("transform_timeout", 0.1),
                ("base_frame", "base_link"),
                ("lidar_frame", "velodyne"),
                ("debug_mode", True),
                ("target_frame", "odom"),     # ★ 기본 출력 프레임: odom
                ("tf_epsilon_ms", 30.0),
            ],
        )

        # ────────────────────────────────────────────────────────────────────────
        # Subscribers
        # ────────────────────────────────────────────────────────────────────────
        self.sub_cloud = self.create_subscription(
            PointCloud2, "/cropbox_filtered", self.pointcloud_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odometry_callback, 10
        )

        # ────────────────────────────────────────────────────────────────────────
        # Publishers
        # ────────────────────────────────────────────────────────────────────────
        self.marker_pub = self.create_publisher(MarkerArray, "/accurate_dynamic_obstacles", 10)
        self.debug_pub = self.create_publisher(String, "/obstacle_debug", 10)

        # ────────────────────────────────────────────────────────────────────────
        # TF Buffer/Listener
        # ────────────────────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ────────────────────────────────────────────────────────────────────────
        # States
        # ────────────────────────────────────────────────────────────────────────
        self.vehicle_odom = None
        self.obstacle_tracks = {}
        self.obstacle_id_counter = 0
        self.last_time = time.time()
        self.last_position = np.zeros(3, dtype=np.float32)

    # ────────────────────────────────────────────────────────────────────────────
    # TF helper with epsilon & fallback
    # ────────────────────────────────────────────────────────────────────────────
    def _lookup_tf_with_epsilon_and_fallback(
        self,
        target_frame: str,
        source_frame: str,
        req_stamp,
        timeout_sec: float = 0.1,
        eps_ms: int = 20,
    ):
        """
        1) 요청 시각(req_stamp)으로 lookup
        2) '미래 외삽'이면 req_stamp - ε(ms)로 재시도
        3) 그래도 안되면 최신 TF(Time())로 폴백
        반환: (TransformStamped, tf_used_stamp)
        """
        eps_ns = int(eps_ms * 1_000_000)

        def _req_minus_eps(_req_stamp):
            req_ns = Time.from_msg(_req_stamp).nanoseconds
            adj_ns = max(req_ns - eps_ns, 0)
            return Time(nanoseconds=adj_ns).to_msg()

        try:
            T = self.tf_buffer.lookup_transform(
                target_frame, source_frame, req_stamp, timeout=Duration(seconds=timeout_sec)
            )
            return T, T.header.stamp
        except TransformException as e1:
            if "extrapolation into the future" in str(e1).lower():
                try:
                    T2 = self.tf_buffer.lookup_transform(
                        target_frame, source_frame, _req_minus_eps(req_stamp), timeout=Duration(seconds=timeout_sec)
                    )
                    return T2, T2.header.stamp
                except TransformException:
                    T3 = self.tf_buffer.lookup_transform(
                        target_frame, source_frame, Time(), timeout=Duration(seconds=timeout_sec)
                    )
                    return T3, T3.header.stamp
            # 다른 오류는 그대로 전파
            raise

    # ────────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────────────────
    def odometry_callback(self, msg: Odometry):
        """차량 속도 및 위치 업데이트"""
        self.vehicle_odom = msg
        self.last_position = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=np.float32,
        )

    def extract_points(self, cloud_msg: PointCloud2) -> np.ndarray:
        """포인트 클라우드 안전하게 추출"""
        try:
            pts = []
            for p in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                pts.append([p[0], p[1], p[2]])
            return np.array(pts, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"Point extraction failed: {str(e)}")
            return np.empty((0, 3), dtype=np.float32)

    def pointcloud_callback(self, msg: PointCloud2):
        try:
            if self.vehicle_odom is None:
                return

            current_time = time.time()
            dt = current_time - self.last_time
            if dt <= 0.001:  # 최소 시간 간격 보장
                return
            self.last_time = current_time

            # LiDAR -> base_link TF
            try:
                lidar_to_base, _ = self._lookup_tf_with_epsilon_and_fallback(
                    self.get_parameter("base_frame").value,
                    self.get_parameter("lidar_frame").value,
                    msg.header.stamp,
                    timeout_sec=float(self.get_parameter("transform_timeout").value),
                    eps_ms=int(self.get_parameter("tf_epsilon_ms").value),
                )
            except TransformException as e:
                self.get_logger().warning(f"TF error (base<-lidar): {str(e)}")
                return

            # 포인트 클라우드 처리
            points = self.extract_points(msg)
            if points.size == 0:
                return

            # 높이 필터링
            z_min = float(self.get_parameter("z_min").value)
            z_max = float(self.get_parameter("z_max").value)
            z_mask = (points[:, 2] > z_min) & (points[:, 2] < z_max)
            filtered_points = points[z_mask]
            if len(filtered_points) < 5:
                return

            # base_link 좌표계로 변환 (R·p + t)
            rot = R.from_quat(
                [
                    lidar_to_base.transform.rotation.x,
                    lidar_to_base.transform.rotation.y,
                    lidar_to_base.transform.rotation.z,
                    lidar_to_base.transform.rotation.w,
                ]
            )
            trans = np.array(
                [
                    lidar_to_base.transform.translation.x,
                    lidar_to_base.transform.translation.y,
                    lidar_to_base.transform.translation.z,
                ],
                dtype=np.float32,
            )
            base_link_points = rot.apply(filtered_points) + trans

            # 클러스터링
            clustering = DBSCAN(
                eps=float(self.get_parameter("cluster_eps").value),
                min_samples=int(self.get_parameter("cluster_min_samples").value),
            ).fit(base_link_points)

            # 클러스터 추출
            current_clusters = {}
            for label in np.unique(clustering.labels_):
                if label == -1:
                    continue
                mask = clustering.labels_ == label
                cluster_pts = base_link_points[mask]
                if len(cluster_pts) >= 5:
                    current_clusters[label] = {
                        "center": np.median(cluster_pts, axis=0),
                        "points": cluster_pts,
                        "size": np.ptp(cluster_pts, axis=0),
                        "time": current_time,
                    }

            # 트랙 업데이트 & 시각화
            self.update_tracks(current_clusters, current_time, dt)
            self.visualize_obstacles(msg.header)

        except Exception as e:
            self.get_logger().error(f"Main loop error: {str(e)}")

    # ────────────────────────────────────────────────────────────────────────────
    # Tracking
    # ────────────────────────────────────────────────────────────────────────────
    def update_tracks(self, current_clusters, current_time, dt):
        """차량 운동 보상이 포함된 장애물 추적"""
        if self.vehicle_odom is None:
            return

        # 차량 속도 (base_link 기준)
        vehicle_velocity = np.array(
            [
                self.vehicle_odom.twist.twist.linear.x,
                self.vehicle_odom.twist.twist.linear.y,
                self.vehicle_odom.twist.twist.linear.z,
            ],
            dtype=np.float32,
        )
        angular_velocity = float(self.vehicle_odom.twist.twist.angular.z)

        debug_msg = String()
        track_window = int(self.get_parameter("tracking_window").value)
        min_dyn_speed = float(self.get_parameter("min_dynamic_speed").value)
        required_frames = int(self.get_parameter("required_consistent_frames").value)
        debug_mode = bool(self.get_parameter("debug_mode").value)

        # 매칭 & 업데이트
        for label, cluster in current_clusters.items():
            best_id = None
            min_dist = float("inf")

            # 1m 이내 최근 트랙과 매칭
            for track_id, track in self.obstacle_tracks.items():
                dist = np.linalg.norm(cluster["center"] - track["current_position"])
                if dist < min_dist and dist < 1.0:
                    min_dist = dist
                    best_id = track_id

            if best_id is not None:
                track = self.obstacle_tracks[best_id]
                displacement = cluster["center"] - track["current_position"]

                # 차량 각속도 보상 (평면 근사)
                rotation_comp = np.array(
                    [-angular_velocity * displacement[1], angular_velocity * displacement[0], 0.0],
                    dtype=np.float32,
                )

                # 장애물 세계속도 추정 ~= 상대변위/dt + 차량속도 + 회전보상
                obstacle_velocity = (displacement / max(dt, 1e-6)) + (vehicle_velocity + rotation_comp)
                obstacle_speed = float(np.linalg.norm(obstacle_velocity[:2]))

                # 속도 히스토리 업데이트
                track["speed_history"].append(obstacle_speed)
                if len(track["speed_history"]) > track_window:
                    track["speed_history"].popleft()

                avg_speed = float(np.mean(track["speed_history"])) if track["speed_history"] else 0.0

                # 동적/정적 분류
                if avg_speed > min_dyn_speed:
                    track["dynamic_frames"] += 1
                else:
                    track["dynamic_frames"] = max(0, track["dynamic_frames"] - 1)

                track.update(
                    {
                        "current_position": cluster["center"],
                        "last_update_time": current_time,
                        "size": cluster["size"],
                        "is_dynamic": track["dynamic_frames"] >= required_frames,
                        "current_velocity": obstacle_velocity,
                        "avg_speed": avg_speed,
                    }
                )

                if debug_mode:
                    debug_msg.data += (
                        f"Track {best_id}: "
                        f"Speed={avg_speed:.2f}m/s "
                        f"(Vehicle={np.linalg.norm(vehicle_velocity):.2f}m/s)\n"
                    )
            else:
                # 신규 트랙
                self.obstacle_tracks[self.obstacle_id_counter] = {
                    "current_position": cluster["center"],
                    "last_update_time": current_time,
                    "size": cluster["size"],
                    "speed_history": deque([0.0], maxlen=track_window),
                    "dynamic_frames": 0,
                    "is_dynamic": False,
                    "current_velocity": np.zeros(3, dtype=np.float32),
                    "avg_speed": 0.0,
                }
                self.obstacle_id_counter += 1

        # 오래된 트랙 제거 (1.0s 이상 미갱신)
        self.obstacle_tracks = {
            k: v for k, v in self.obstacle_tracks.items() if (current_time - v["last_update_time"]) <= 1.0
        }

        if debug_msg.data:
            self.debug_pub.publish(debug_msg)

    # ────────────────────────────────────────────────────────────────────────────
    # Visualization (publish in target_frame; default 'odom')
    # ────────────────────────────────────────────────────────────────────────────
    def visualize_obstacles(self, header):
        marker_array = MarkerArray()
        base_frame = self.get_parameter("base_frame").value
        target_frame = self.get_parameter("target_frame").value  # e.g., "odom"

        try:
            # target_frame <- base_link at header.stamp (epsilon/fallback)
            T_target_base, tf_stamp = self._lookup_tf_with_epsilon_and_fallback(
                target_frame, base_frame, header.stamp
            )
        except TransformException as e:
            self.get_logger().warning(f"TF {target_frame} <- {base_frame} failed: {e}")
            return

        # 먼저 DELETEALL
        delete_marker = Marker()
        delete_marker.header.frame_id = target_frame
        delete_marker.header.stamp = tf_stamp
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for track_id, track in self.obstacle_tracks.items():
            # 완전 정적/저속은 표시 생략 (필요 시 조건 조정)
            if not track["is_dynamic"] and track["avg_speed"] < 0.1:
                continue

            # 중심점 변환 (PoseStamped → do_transform_pose)
            ps = PoseStamped()
            ps.header.frame_id = base_frame
            ps.header.stamp = tf_stamp
            ps.pose.position.x = float(track["current_position"][0])
            ps.pose.position.y = float(track["current_position"][1])
            ps.pose.position.z = float(track["current_position"][2])
            ps.pose.orientation.w = 1.0

            pose_out = tf2_geometry_msgs.do_transform_pose(ps.pose, T_target_base)
            
            mk = Marker()
            mk.header.frame_id = target_frame
            mk.header.stamp = tf_stamp
            mk.ns = "obstacles"
            mk.id = int(track_id)
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose = pose_out
            mk.scale.x = float(max(track["size"][0], 0.3))
            mk.scale.y = float(max(track["size"][1], 0.3))
            mk.scale.z = float(max(track["size"][2], 0.3))
            mk.color.r = 1.0 if track["is_dynamic"] else 0.5
            mk.color.g = 0.0
            mk.color.b = 0.0
            mk.color.a = 0.7
            mk.lifetime.nanosec = int(0.5e9)
            marker_array.markers.append(mk)

            # 속도 화살표
            if track["avg_speed"] >= 0.3:
                v = track["current_velocity"]
                n = float(np.linalg.norm(v)) + 1e-6
                v_dir = v / n
                start_b = track["current_position"]
                end_b = start_b + np.array([v_dir[0] * 0.5, v_dir[1] * 0.5, v_dir[2] * 0.2], dtype=np.float32)

                ps_start = PointStamped()
                ps_start.header.frame_id = base_frame
                ps_start.header.stamp = tf_stamp
                ps_start.point = Point(
                    x=float(start_b[0]),
                    y=float(start_b[1]),
                    z=float(start_b[2]),
                )

                ps_end = PointStamped()
                ps_end.header.frame_id = base_frame
                ps_end.header.stamp = tf_stamp
                ps_end.point = Point(
                    x=float(end_b[0]),
                    y=float(end_b[1]),
                    z=float(end_b[2]),
                )

                try:
                    start_m = tf2_geometry_msgs.do_transform_point(ps_start, T_target_base).point
                    end_m = tf2_geometry_msgs.do_transform_point(ps_end, T_target_base).point
                except Exception as e:
                    self.get_logger().warning(f"point transform failed: {e}")
                    continue

                vel = Marker()
                vel.header.frame_id = target_frame
                vel.header.stamp = tf_stamp
                vel.ns = "velocity"
                vel.id = int(track_id) + 10000
                vel.type = Marker.ARROW
                vel.action = Marker.ADD
                vel.points = [start_m, end_m]
                vel.scale.x = 0.1
                vel.scale.y = 0.2
                vel.color.r = 0.0 if track["is_dynamic"] else 0.5
                vel.color.g = 1.0 if track["is_dynamic"] else 0.5
                vel.color.b = 0.0
                vel.color.a = 0.9
                vel.lifetime.nanosec = int(0.5e9)
                marker_array.markers.append(vel)

        self.marker_pub.publish(marker_array)


# ────────────────────────────────────────────────────────────────────────────────
# main
# ────────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = AccurateDynamicObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
