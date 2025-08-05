#!/usr/bin/env python3
# lidar_heading_estimator.py  (EKF X)

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs_py import point_cloud2 as pc2
from sklearn.linear_model import RANSACRegressor, LinearRegression
import collections
from tf_transformations import quaternion_from_euler


# ─────────────────────────────────────────────────────────────
def yaw_from_slope(a):
    """y = a·x + b  →  yaw(rad)"""
    return np.arctan(a)


def angle_normalize(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def circular_mean(arr):
    """array-like(rad) → 원형 평균(rad)"""
    return np.angle(np.mean(np.exp(1j * np.array(arr))))


def fit_line_ransac(xy, thresh=0.15):
    """
    RANSAC 직선 피팅
      xy : (N,2)
    반환 : (slope, intercept, inlier_ratio) or None
    """
    if xy.shape[0] < 30:
        return None
    ransac = RANSACRegressor(LinearRegression(),
                             residual_threshold=thresh,
                             min_samples=0.5)
    ransac.fit(xy[:, 0].reshape(-1, 1), xy[:, 1])
    inlier_ratio = ransac.inlier_mask_.mean()
    a = ransac.estimator_.coef_[0]
    b = ransac.estimator_.intercept_
    return a, b, inlier_ratio


# ─────────────────────────────────────────────────────────────
class LidarHeading(Node):
    def __init__(self):
        super().__init__('lidar_heading')

        # ── ROS 파라미터
        self.declare_parameter('topic_in',        '/velodyne_points')
        self.declare_parameter('topic_out',       '/lidar_heading')
        self.declare_parameter('tunnel_yaw_map',  -2.58849)  # [rad]
        self.declare_parameter('z_low',           0.0)
        self.declare_parameter('z_high',           1.0)
        self.declare_parameter('range_min',        1.0)
        self.declare_parameter('range_max',       10.0)
        self.declare_parameter('alpha_lpf',        0.2)      # 0(강)~1(무)

        p = self.get_parameter
        topic_in   = p('topic_in').value
        topic_out  = p('topic_out').value
        self.tun_yaw = p('tunnel_yaw_map').value
        self.z_low  = p('z_low').value
        self.z_high = p('z_high').value
        self.rmin   = p('range_min').value
        self.rmax   = p('range_max').value
        self.alpha  = p('alpha_lpf').value

        # ── ROS 통신
        self.sub = self.create_subscription(PointCloud2, topic_in, self.cb, 10)
        self.pub = self.create_publisher(QuaternionStamped, topic_out, 10)

        # ── 상태
        self.window = collections.deque(maxlen=8)  # 슬라이딩 워터멜론
        self.yaw_lpf = None

    # ─────────────────────────────────────────────
    def cb(self, msg: PointCloud2):
        # 1. 포인트 클라우드 필터링
        pts = np.array([[p[0], p[1], p[2]]
                        for p in pc2.read_points(msg,
                                                 field_names=('x', 'y', 'z'),
                                                 skip_nans=True)
                        if self.z_low < p[2] < self.z_high])
        if pts.shape[0] < 60:                     # 샘플 부족
            return

        d = np.linalg.norm(pts[:, :2], axis=1)
        mask = (d > self.rmin) & (d < self.rmax)
        xy = pts[mask][:, :2]
        if xy.shape[0] < 60:
            return

        # 2. 좌/우 벽 RANSAC
        left  = xy[xy[:, 1] >  0.2]
        right = xy[xy[:, 1] < -0.2]

        l_fit = fit_line_ransac(left)
        r_fit = fit_line_ransac(right)
        if not l_fit or not r_fit or min(l_fit[2], r_fit[2]) < 0.5:
            return  # 품질 나쁨 → skip

        slope_c = (l_fit[0] + r_fit[0]) / 2.0
        heading_raw = yaw_from_slope(slope_c)

        # 3. 슬라이딩 원형 평균
        self.window.append(heading_raw)
        head_avg = circular_mean(self.window)

        # 4. 1-차 저역필터
        if self.yaw_lpf is None:
            self.yaw_lpf = head_avg
        else:
            self.yaw_lpf = self.alpha * head_avg + (1 - self.alpha) * self.yaw_lpf

        yaw_map = self.tun_yaw + angle_normalize(self.yaw_lpf)

        # 5. Publish
        q = quaternion_from_euler(0.0, 0.0, yaw_map)
        q_msg = QuaternionStamped()
        q_msg.header.stamp = msg.header.stamp
        q_msg.header.frame_id = 'map'
        q_msg.quaternion.x, q_msg.quaternion.y, \
        q_msg.quaternion.z, q_msg.quaternion.w = q
        self.pub.publish(q_msg)


# ─────────────────────────────────────────────
def main():
    rclpy.init()
    node = LidarHeading()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
