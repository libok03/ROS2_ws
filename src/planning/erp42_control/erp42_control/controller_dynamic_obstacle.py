#!/usr/bin/env python3
import math
import numpy as np
import time
import os

import rclpy
from rclpy.qos import qos_profile_system_default

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String
from tf_transformations import *  # if needed elsewhere

from stanley import Stanley
from erp42_msgs.msg import SerialFeedBack, ControlMessage


class SpeedSupporter:
    def __init__(self, node):
        self.he_gain = node.declare_parameter("/speed_supporter/he_gain_stopline", 50.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain_stopline", 30.0).value
        self.he_thr  = node.declare_parameter("/speed_supporter/he_thr_stopline", 0.001).value
        self.ce_thr  = node.declare_parameter("/speed_supporter/ce_thr_stopline", 0.002).value

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        return np.clip(value + err, min_value, max_value)


class PID:
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain_stopline", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain_stopline", 0.85).value

        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0

        now = node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + now[1] / 1e9
        self.last = self.current

    def PIDControl(self, speed_kmh, desired_value_kmh):
        now = self.node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + now[1] / 1e9
        dt = max(self.current - self.last, 1e-3)
        self.last = self.current

        err = desired_value_kmh - speed_kmh
        self.p_err = err
        self.i_err += self.p_err * dt * (0.0 if speed_kmh == 0 else 1.0)

        self.speed = speed_kmh + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, 7, 10))  # km/h -> 나중에 *10 해서 ERP 속도로 보냄


class Stopline:
    """
    - 입력 odometry: GetOdometry 객체 (x[m], y[m], yaw[rad], v[m/s])
    - /accurate_dynamic_obstacles (MarkerArray, odom frame) 구독
    - 자차(x,y)와 마커 pose 간 최소거리 ≤ 5 m → 즉시 정지, 5초 후 자동 해제
    - 기존 Stanley + PID 속도 제어 유지
    """

    def __init__(self, node):
        self.node = node

        self.target_idx = 0
        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)

        self.estop = 0
        self.target_speed = 10
        self.count = 0

        # 장애물 기반 E-Stop 상태/파라미터
        self.obstacle_xy = []                 # [(x,y), ...] in odom
        self.obstacle_stop_active = False
        self.obstacle_stop_until  = 0.0
        self.obs_stop_dist = node.declare_parameter("/stopline/obstacle_stop_distance_m", 5.0).value
        self.obs_stop_duration = node.declare_parameter("/stopline/obstacle_stop_duration_s", 5.0).value

        # MarkerArray 구독 (odom 프레임 가정)
        self.sub_obs = node.create_subscription(
            MarkerArray, "/accurate_dynamic_obstacles", self._obs_cb, 10
        )

    # ---------- 유틸 ----------
    def _now_sec(self) -> float:
        s, ns = self.node.get_clock().now().seconds_nanoseconds()
        return s + ns * 1e-9

    def _ego_xy_from_odometry(self, odom_obj) -> tuple[float, float]:
        # GetOdometry: .x, .y 제공
        return float(odom_obj.x), float(odom_obj.y)

    def _ego_speed_kmh(self, odom_obj) -> float:
        # GetOdometry: .v (m/s) 제공
        return float(odom_obj.v) * 3.6

    def _obs_cb(self, msg: MarkerArray):
        # odom 프레임 가정. ADD 마커 + pose 기반 마커만 사용
        xy = []
        for mk in msg.markers:
            if mk.action != Marker.ADD:
                continue
            if mk.type in (
                Marker.CUBE,
                Marker.SPHERE,
                Marker.CYLINDER,
                Marker.MESH_RESOURCE,
                Marker.TEXT_VIEW_FACING,
            ):
                xy.append((mk.pose.position.x, mk.pose.position.y))
        self.obstacle_xy = xy

    # ---------- 메인 제어 ----------
    def control_stop_line(self, odometry, path):
        """
        odometry: GetOdometry (x,y,yaw,v)
        path: 사용자 정의 Path 래퍼 (cx, cy, cyaw 배열; Stanley가 이 인터페이스를 기대)
        """
        stopline_finished = False
        msg = ControlMessage()

        # ----- 장애물 기반 E-Stop -----
        now = self._now_sec()
        ex, ey = self._ego_xy_from_odometry(odometry)

        # 타이머 지속 중이면 계속 정지
        if self.obstacle_stop_active:
            if now < self.obstacle_stop_until:
                msg.steer = 0
                msg.speed = 0
                msg.gear  = 2
                msg.estop = 1
                return msg, False
            else:
                self.obstacle_stop_active = False  # 타이머 만료

        # 새로 근접 감지?
        if self.obstacle_xy:
            dmin = min(math.hypot(ex - ox, ey - oy) for (ox, oy) in self.obstacle_xy)
            if dmin <= float(self.obs_stop_dist):
                self.obstacle_stop_active = True
                self.obstacle_stop_until  = now + float(self.obs_stop_duration)
                msg.steer = 0
                msg.speed = 0
                msg.gear  = 2
                msg.estop = 1
                return msg, False
        # ----- 장애물 E-Stop 끝 -----

        # ===== 기존 Stopline 제어 =====
        steer, self.target_idx, hdr, ctr = self.st.stanley_control(
            odometry, path.cx, path.cy, path.cyaw, h_gain=0.5, c_gain=0.24
        )
        # 디버깅 출력
        print(self.target_idx, len(path.cx))

        if self.target_idx >= len(path.cx) - 5:
            # goal 0.5 m 이내
            print(self.target_idx, len(path.cx), self.count)
            if self.count <= 30:
                self.estop = 1
                self.count += 1
            else:
                self.count = 0
                self.estop = 0
                stopline_finished = True
            speed_kmh = 0

        elif self.target_idx >= len(path.cx) - 50:
            # goal 5 m 이내부터 감속
            self.target_speed = (len(path.cx) - self.target_idx) / len(path.cx) * 15
            self.target_speed = int(np.clip(self.target_speed, 6, 8))
            adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=6, max_value=8)
            speed_kmh = self.pid.PIDControl(self._ego_speed_kmh(odometry), adapted_speed)

        else:
            self.target_speed = 10
            adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, min_value=8, max_value=10)
            speed_kmh = self.pid.PIDControl(self._ego_speed_kmh(odometry), adapted_speed)

        msg.steer = int(math.degrees((-1) * steer))
        msg.speed = int(speed_kmh) * 10   # ERP42 스케일
        msg.gear  = 2
        msg.estop = self.estop

        return msg, stopline_finished
