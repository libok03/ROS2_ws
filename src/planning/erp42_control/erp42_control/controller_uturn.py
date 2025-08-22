# ros2
import rclpy
from rclpy.qos import qos_profile_system_default

# msg
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from erp42_msgs.msg import ControlMessage

# tf
from tf_transformations import *

# stanley
from stanley import Stanley

# DB
try:
    from DB import DB
except Exception as e:
    print(e)

# utils
import time
from enum import Enum
import numpy as np
import math as m


class SpeedSupporter:
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter(
            "/speed_supporter/he_gain_uturn", 50.0
        ).value
        self.ce_gain = node.declare_parameter(
            "/speed_supporter/ce_gain_uturn", 30.0
        ).value

        self.he_thr = node.declare_parameter(
            "/speed_supporter/he_thr_uturn", 0.001
        ).value
        self.ce_thr = node.declare_parameter(
            "/speed_supporter/ce_thr_uturn", 0.002
        ).value

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res


class PID:
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter(
            "/stanley_controller/p_gain_uturn", 2.07
        ).value
        self.i_gain = node.declare_parameter(
            "/stanley_controller/i_gain_uturn", 0.85
        ).value

        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0

        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (
            node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )

    def PIDControl(self, speed, desired_value):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (
            self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        )
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt
        self.p_err = err
        self.i_err += self.p_err * dt * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, 6, 8))


class Uturn_state(Enum):
    In = 0
    Turn = 1
    Out = 2


class Uturn:
    def __init__(self, node):

        self.node = node

        # IN_path
        self.in_path_db = DB("U-turn_IN.db")  # kcity
        self.in_path = self.in_path_db.read_db_n("Path", "x", "y", "yaw")

        # Turn_path (템플릿/작업본 분리)  # FIX: 더블-쉬프트 방지
        self.turn_db = DB("U-turn.db")
        self.turn_path_template = self.turn_db.read_db_n("Path", "x", "y", "yaw")  # 원본
        self.turn_path = list(self.turn_path_template)  # 현재 적용본

        # Out_path
        self.out_path_db = DB("U-turn_OUT.db")
        self.out_path = self.out_path_db.read_db_n("Path", "x", "y", "yaw")

        # instance
        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)

        self.cones = []  # List to store cone positions
        self.odometry = None  # Placeholder for odometry data

        # uturn_state
        self.state = Uturn_state.In

        # subscriber
        self.node.create_subscription(
            PoseArray, "/cone_pose_map", self.cone_callback, 10
        )
        # publisher (visualization)
        self.pub_path = node.create_publisher(
            Path, "uturn_path", qos_profile_system_default
        )

    # FIX: 콘 버퍼 완전 초기화 후 새 메시지로 채움(리스트 누적 방지)
    def cone_callback(self, msg):
        self.cones = []  # 새 메시지로 초기화
        for pose in msg.poses:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z
            self.cones.append(point)

    # FIX: 항상 템플릿에서 평행이동 경로를 생성
    def _translated_turn_path(self, dx, dy):
        return [(p[0] + dx, p[1] + dy, p[2]) for p in self.turn_path_template]

    def control_uturn(self, odometry):
        self.odometry = odometry
        msg = ControlMessage()

        if self.state == Uturn_state.In:
            path_x = [p[0] for p in self.in_path]
            path_y = [p[1] for p in self.in_path]
            path_yaw = [p[2] for p in self.in_path]
            self.publish_path_msg(path_x, path_y, path_yaw)

            steer, target_idx, hdr, ctr = self.st.stanley_control(
                odometry,
                path_x,
                path_y,
                path_yaw,
                h_gain=0.5,
                c_gain=0.24,
            )

            target_speed = 12.0
            adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=10, max_value=14)
            speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed)
            msg.speed = int(speed) * 10
            msg.steer = int(m.degrees((-1) * steer))
            msg.gear = 2

            flag, dx, dy = self.in_uturn()
            if flag:
                # FIX: 템플릿에서 새로 평행이동 복제
                self.turn_path = self._translated_turn_path(dx, dy)
                self.state = Uturn_state.Turn

            # 만약 적절한 턴 지점을 못찾으면 강제 턴
            if target_idx >= len(path_x) - 3:
                # FIX: 강제 진입도 템플릿 기준으로 계산
                path0_x, path0_y = self.turn_path_template[0][0], self.turn_path_template[0][1]
                dx = self.odometry.x - path0_x
                dy = self.odometry.y - path0_y
                self.turn_path = self._translated_turn_path(dx, dy)
                self.state = Uturn_state.Turn

        elif self.state == Uturn_state.Turn:
            # 잘 못된 턴 지점이 들어오면 In 초기화
            collides = self.correct_turn()  # True면 콘과 2m 이내(=충돌)

            path_x = [p[0] for p in self.turn_path]
            path_y = [p[1] for p in self.turn_path]
            path_yaw = [p[2] for p in self.turn_path]
            self.publish_path_msg(path_x, path_y, path_yaw)

            steer, target_idx, hdr, ctr = self.st.stanley_control(
                odometry,
                path_x,
                path_y,
                path_yaw,
                h_gain=0.5,
                c_gain=0.24,
            )
            msg.speed = int(5) * 10
            msg.steer = int(m.degrees((-1) * steer))
            msg.gear = 2

            # FIX: 충돌이면 In으로 롤백, 안전하면 Out 전환만 판단
            if collides:
                self.state = Uturn_state.In
            elif target_idx >= len(path_x) - 3:
                self.state = Uturn_state.Out

        elif self.state == Uturn_state.Out:
            path_x = [p[0] for p in self.out_path]
            path_y = [p[1] for p in self.out_path]
            path_yaw = [p[2] for p in self.out_path]
            self.publish_path_msg(path_x, path_y, path_yaw)

            steer, target_idx, hdr, ctr = self.st.stanley_control(
                odometry,
                path_x,
                path_y,
                path_yaw,
                h_gain=0.5,
                c_gain=0.24,
            )
            
            target_speed = 10.0
            adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=8, max_value=12)
            speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed) # speed 조정 (PI control) 
            msg.speed = int(speed) * 10
            msg.steer = int(m.degrees((-1) * steer))
            msg.gear = 2

            if target_idx >= len(path_x) - 3:
                return msg, True
        return msg, False

    def in_uturn(self):
        # FIX: 템플릿 기준으로 평행이동 벡터 계산
        if len(self.turn_path_template) == 0:
            return False, 0, 0

        # cone 정보가 없으면 False 반환(원래 주석은 비활성화되어 있어 그대로 둠)

        # odometry와 turn_path 템플릿의 첫 점을 일치시키기 위한 평행이동 벡터 계산
        path0_x, path0_y = self.turn_path_template[0][0], self.turn_path_template[0][1]
        dx = self.odometry.x - path0_x
        dy = self.odometry.y - path0_y

        # 평행이동된 템플릿 경로와 cone의 거리 비교
        translated = self._translated_turn_path(dx, dy)
        for px, py, _ in translated:
            for cone in self.cones:
                dist = np.hypot(px - cone.x, py - cone.y)
                if dist < 2.0:
                    return False, 0, 0
        return True, dx, dy

    # True면 현재 TURN 경로가 콘과 충돌(2m 이내)
    def correct_turn(self):
        if len(self.cones) == 0 or len(self.turn_path) == 0:
            return False

        # 현재 self.turn_path는 이미 평행이동된 상태
        for px, py, _ in self.turn_path:
            for cone in self.cones:
                dist = np.hypot(px - cone.x, py - cone.y)
                if dist < 2.0:
                    return True
        return False

    def publish_path_msg(self, path_x, path_y, path_yaw):
        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for x, y, yaw in zip(path_x, path_y, path_yaw):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            yaw_f = float(yaw)
            qx, qy, qz, qw = 0.0, 0.0, np.sin(yaw_f / 2), np.cos(yaw_f / 2)
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            pose.pose.orientation.w = float(qw)
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)
