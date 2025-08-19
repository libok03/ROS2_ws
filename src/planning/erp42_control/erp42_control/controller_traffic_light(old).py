import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from erp42_msgs.msg import ControlMessage, SerialFeedBack  # SerialFeedBack 추가
from yolo_msg.msg import TrafficSign
from tf_transformations import euler_from_quaternion
import math
import numpy as np
from stanley import Stanley
from visualization_msgs.msg import Marker
from collections import deque, Counter
import time
from yolo_msg.msg import BoundingBox, BoundingBoxArray


class SpeedSupporter:
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter(
            "/speed_supporter/he_gain_traffic", 40.0
        ).value
        self.ce_gain = node.declare_parameter(
            "/speed_supporter/ce_gain_traffic", 20.0
        ).value

        self.he_thr = node.declare_parameter(
            "/speed_supporter/he_thr_traffic", 0.05
        ).value
        self.ce_thr = node.declare_parameter(
            "/speed_supporter/ce_thr_traffic", 0.001
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
            "/stanley_controller/p_gain_traffic", 2.07
        ).value
        self.i_gain = node.declare_parameter(
            "/stanley_controller/i_gain_traffic", 0.85
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

    def PIDControl(self, speed, desired_value, min, max):

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
        return int(np.clip(self.speed, min, max))


class Trafficlight:
    def __init__(self, node):
        self.node = node

        self.st = Stanley()
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)

        self.target_idx = 0
        self.pre_stanley_idx = 0

        # ── 신호 판단 관련 상태 ─────────────────────────────────────────────
        self.signal_window = deque(maxlen=int(
            node.declare_parameter("/traffic/vote_window", 10).value
        ))
        self.vote_k = int(node.declare_parameter("/traffic/vote_k", 6).value)
        self.vote_hold_s = float(node.declare_parameter("/traffic/vote_hold_s", 0.3).value)
        self.signal_timeout_s = float(node.declare_parameter("/traffic/signal_timeout_s", 0.5).value)

        self.stable_signal = None        # 현재 확정 신호: 'red'/'yellow'/'green'/None
        self.prev_stable_signal = None   # 직전 확정 신호
        self.last_stable_change_ts = 0.0 # 확정 신호가 바뀐 시각
        self.last_signal_time = 0.0      # 마지막 YOLO 메시지 수신 시각

        self.mission_finish = False
        self.start_code = True

        self.target_speed = 8
        self.speed = 0
        self.estop = 0

        self.control_pub = self.node.create_publisher(ControlMessage, "cmd_msg", 10)

        self.traffic_light_sub = self.node.create_subscription(
            BoundingBoxArray, "/yolo/light", self.traffic_light_callback, 10
        )

        self.count = 0

        # 정지선 근접 판정(인덱스 기반): 맵 해상도에 따라 조정
        self.brake_distance_idx = int(node.declare_parameter("/traffic/brake_distance_idx", 30).value)
        self.stop_near_idx = int(node.declare_parameter("/traffic/stop_near_idx", 5).value)
        self.yellow_stop_idx = int(node.declare_parameter("/traffic/yellow_stop_idx", 15).value)

    # ── 유틸: 클래스 이름 정규화 ───────────────────────────────────────────
    def _normalize_label(self, name: str):
        if not name:
            return None
        s = name.lower()
        if "red" in s or "stop" in s:
            return "red"
        if "green" in s or "go" in s:
            return "green"
        if "yellow" in s or "amber" in s:
            return "yellow"
        return None  # 모르는 라벨은 집계 제외

    # ── 유틸: 다수결로 확정 신호 갱신 ─────────────────────────────────────
    def _update_stable_signal(self):
        # 드롭아웃: 최근 수신이 오래되면 확정 해제
        now = time.time()
        if self.last_signal_time > 0 and (now - self.last_signal_time) > self.signal_timeout_s:
            # 신호 끊김
            if self.stable_signal is not None:
                self.prev_stable_signal = self.stable_signal
                self.stable_signal = None
                self.last_stable_change_ts = now
            return

        # 윈도우에서 유효 라벨만 집계
        valid = [x for x in self.signal_window if x in ("red", "yellow", "green")]
        if not valid:
            return

        counts = Counter(valid)
        # 현재 확정 신호에서 바꾸려면 hold 시간 지나야 함
        can_switch = (now - self.last_stable_change_ts) >= self.vote_hold_s

        # K개 이상인 후보들 중 count가 가장 큰 라벨 선택
        candidates = [(lab, cnt) for lab, cnt in counts.items() if cnt >= self.vote_k]
        if not candidates:
            return

        candidates.sort(key=lambda x: x[1], reverse=True)
        top_label, top_count = candidates[0]

        if self.stable_signal is None:
            # 처음 확정
            self.prev_stable_signal = None
            self.stable_signal = top_label
            self.last_stable_change_ts = now
        else:
            if top_label != self.stable_signal and can_switch:
                self.prev_stable_signal = self.stable_signal
                self.stable_signal = top_label
                self.last_stable_change_ts = now

    # ── 콜백: YOLO 신호 수신 → 윈도우에 반영 ───────────────────────────────
    def traffic_light_callback(self, msg: BoundingBoxArray):
        self.last_signal_time = time.time()
        if not self.start_code:
            return

        # 가장 아래(화면 기준 가까운) 객체를 대표로 사용
        data_list = [
            {"class_name": box.class_name, "confidence": box.confidence, "center_y": box.center_y}
            for box in msg.boxes
        ]
        data_list.sort(key=lambda x: x["center_y"], reverse=False)

        if data_list:
            lab = self._normalize_label(data_list[0]["class_name"])
            if lab is not None:
                self.signal_window.append(lab)
        # 비어있으면 윈도우에 아무 것도 넣지 않음(잡음 방지)

        self._update_stable_signal()

    # ── 제어 루프 ──────────────────────────────────────────────────────────
    def control_traffic_light(self, odometry, path):
        if self.start_code is False:
            self.start_code = True

        steer, self.target_idx, hdr, ctr = self.st.stanley_control(
            odometry,
            path.cx, path.cy, path.cyaw,
            h_gain=0.5, c_gain=0.24,
        )

        current_distance_to_stop_line = len(path.cx) - self.target_idx
        sig = self.stable_signal  # 'red'/'yellow'/'green'/None

        msg = ControlMessage()

        # ── 신호별 로직 ───────────────────────────────────────────────────
        if sig == "red":
            if current_distance_to_stop_line <= self.brake_distance_idx:
                self.speed = 0
                self.estop = 1
                self.node.get_logger().info("빨간불: 정지")
            else:
                # 아직 멀면 안전 저속으로 접근
                self.target_speed = int(np.clip(
                    (current_distance_to_stop_line / max(1, len(path.cx))) * 15, 6, 8
                ))
                adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, 6, 8)
                self.speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, 6, 8)
                self.estop = 0
                self.node.get_logger().info("빨간불 원거리: 감속 접근")

            if current_distance_to_stop_line <= self.stop_near_idx:
                self.mission_finish = True
                self.node.get_logger().info("빨간불: 정지선 부근 미션 완료")

        elif sig == "yellow":
            # 멀면 저속 통과, 가까우면 정지
            if current_distance_to_stop_line <= self.yellow_stop_idx:
                self.speed = 0
                self.estop = 1
                self.node.get_logger().info("노란불 근접: 정지 선택")
            else:
                self.target_speed = 8  # 저속 유지
                adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, 8, 10)
                self.speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, 8, 10)
                self.estop = 0
                self.node.get_logger().info("노란불 원거리: 저속 통과")

            if current_distance_to_stop_line <= self.stop_near_idx:
                self.mission_finish = True
                self.node.get_logger().info("노란불: 정지선 부근 미션 완료")

        elif sig == "green" or sig is None:
            # 초록 또는 신호 미확정(None) 시 기본 주행(보수적이면 None을 저속으로도 가능)
            self.target_speed = 10
            adapted_speed = self.ss.adaptSpeed(self.target_speed, hdr, ctr, 8, 10)
            self.speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed, 8, 10)
            self.estop = 0
            if sig == "green":
                self.node.get_logger().info("초록불: 주행 유지")
            else:
                self.node.get_logger().info("신호없음: 기본 주행 유지")

            if current_distance_to_stop_line <= self.stop_near_idx:
                self.mission_finish = True
                self.node.get_logger().info("초록/무신호: 정지선 통과 미션 완료")

        # ── 메시지 채우기 ─────────────────────────────────────────────────
        msg.steer = int(math.degrees(-1 * steer))
        msg.speed = int(self.speed) * 10
        msg.gear = 2
        msg.estop = self.estop

        # ── 미션 종료 처리 ────────────────────────────────────────────────
        if current_distance_to_stop_line <= 1:
            self.mission_finish = True
            self.node.get_logger().info("강제 미션 완료")

        if self.mission_finish:
            self.node.get_logger().info("미션 완료: 다음 신호등 구간으로 이동")
            self.start_code = False
            self.mission_finish = False
            return msg, True

        return msg, False

    def reset_traffic_light(self):
        self.signal_window.clear()
        self.stable_signal = None
        self.prev_stable_signal = None
        self.last_stable_change_ts = 0.0
        self.last_signal_time = 0.0
        self.mission_finish = False
        self.start_code = True