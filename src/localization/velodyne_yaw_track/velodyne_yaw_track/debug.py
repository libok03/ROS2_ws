#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
실시간 Yaw 비교 노드
  • LiDAR   : /lidar_heading                (geometry_msgs/QuaternionStamped)
  • 기준값  : /localization/kinematic_state (nav_msgs/Odometry)
  • 출력    : /yaw_error                    (std_msgs/Float32, LiDAR - Localization [rad])
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
import message_filters
import numpy as np
import math
from tf_transformations import euler_from_quaternion


def yaw_from_quat(q):
    """geometry_msgs/Quaternion → yaw(rad)"""
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def ang_norm(a):
    """[-π, π] 로 정규화"""
    return (a + np.pi) % (2 * np.pi) - np.pi


class YawComparer(Node):
    def __init__(self):
        super().__init__('yaw_comparer')

        # ── 파라미터
        self.declare_parameter('lidar_topic', '/lidar_heading')
        self.declare_parameter('kin_topic',   '/localization/kinematic_state')
        self.declare_parameter('err_topic',   '/yaw_error')
        self.declare_parameter('queue_size',  30)
        self.declare_parameter('sync_slop',   0.1)   # [s]

        p = self.get_parameter
        lidar_topic = p('lidar_topic').value
        kin_topic   = p('kin_topic').value
        err_topic   = p('err_topic').value
        q_size      = p('queue_size').value
        slop        = p('sync_slop').value

        # ── pub
        self.pub_err = self.create_publisher(Float32, err_topic, 10)

        # ── subs + 시간 동기
        sub_lidar = message_filters.Subscriber(
            self, QuaternionStamped, lidar_topic,
            qos_profile=rclpy.qos.qos_profile_sensor_data)
        sub_kin = message_filters.Subscriber(
            self, Odometry, kin_topic,
            qos_profile=rclpy.qos.qos_profile_sensor_data)

        sync = message_filters.ApproximateTimeSynchronizer(
            [sub_lidar, sub_kin], queue_size=q_size, slop=slop)
        sync.registerCallback(self.cb_sync)

        self.get_logger().info(f"Yaw 비교 시작 ▶ {lidar_topic} ↔ {kin_topic}")

    # ───────────────────────────────────────
    def cb_sync(self, msg_lidar: QuaternionStamped, msg_kin: Odometry):
        yaw_lidar = yaw_from_quat(msg_lidar.quaternion)
        yaw_kin   = yaw_from_quat(msg_kin.pose.pose.orientation)

        err = math.degrees(ang_norm(yaw_lidar - yaw_kin))

        self.pub_err.publish(Float32(data=err))
        # 디버그용 로그 (필요 시 레벨 DEBUG 로 조정)
        # self.get_logger().info(f"err={np.degrees(err):+.2f}°")


def main():
    rclpy.init()
    node = YawComparer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
