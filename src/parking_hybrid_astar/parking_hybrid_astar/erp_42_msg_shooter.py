#!/usr/bin/env python3
"""
이 노드는 /odom 토픽으로부터 차량 위치(odometry) 정보를,
그리고 /planned_path 토픽으로부터 경로 정보를 수신합니다.
수신된 odom과 path 데이터를 기반으로 Stanley 제어 알고리즘과 SpeedSupporter 보조 기능을 실행하여,
ERP42 제어 메시지(ControlMessage)를 /erp42_command 토픽에 발행(publish)합니다.
"""

from .stanley import Stanley
import rclpy
from rclpy.node import Node
import math
import sys
import os
# 메시지 타입 임포트
from nav_msgs.msg import Path, Odometry               # 경로와 차량 상태 메시지
from erp42_msgs.msg import ControlMessage             # 차량 제어 명령 메시지 (speed, steer, gear, brake 등)
from geometry_msgs.msg import PoseStamped             # 경로에 포함된 개별 포즈 메시지
from math import degrees
import numpy as np
# Stanley 제어 알고리즘 클래스 (해당 모듈이 올바르게 배포되어 있다고 가정)


# 차량 상태를 저장하는 간단한 데이터 클래스
class VehicleState:
    pass

class Erp42CommandSender(Node):
    def __init__(self):
        super().__init__('erp42_command_sender')

        # 파라미터는 미리 set 되어 있어야 합니다.
        # 예: ros2 param set /erp42_command_sender /speed_supporter/he_gain 30.0 등

        # 구독: 차량 상태(odometry)를 /odom 토픽에서 받음
        self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odom_callback,
            10
        )
        # 구독: 경로 정보를 /planned_path 토픽에서 받음
        self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        # 발행: ERP42 제어 메시지를 /erp42_command 토픽으로 발행
        self.cmd_pub = self.create_publisher(ControlMessage, '/erp42_command', 10)
        
        # Stanley 제어 모듈 생성 (내부 파라미터는 파라미터 서버나 YAML 파일에 정의되어 있다고 가정)
        self.stanley = Stanley()
        # SpeedSupporter 인스턴스 생성 (현재 노드를 인자로 넘겨 파라미터를 읽음)
        self.speed_supporter = 3
        
        # 경로 진행시 사용할 마지막 타겟 인덱스 (초기값 0)
        self.last_target_idx = 0

        # /odom 메시지로부터 최신 차량 상태 저장 (초기에는 None)
        self.odom_state = None

        self.get_logger().info("ERP42 Command Sender 노드(Ros2)가 시작되었습니다.")

    def odom_callback(self, odom_msg: Odometry):
        """
        /odom 토픽 메시지가 수신될 때 호출되는 콜백 함수.
        odom 메시지로부터 현재 차량의 위치(x, y), yaw(방향)와 속도(v)를 추출하여 저장합니다.
        """
        state = VehicleState()
        # 위치 정보
        state.x = odom_msg.pose.pose.position.x
        state.y = odom_msg.pose.pose.position.y

        # 쿼터니언을 오일러 각으로 변환하여 yaw 값을 계산
        q = odom_msg.pose.pose.orientation
        yaw = self.euler_from_quaternion([q.x, q.y, q.z, q.w])
        state.yaw = yaw

        # 속도 정보: linear.x 를 사용 (m/s)
        state.v = odom_msg.twist.twist.linear.x

        # 최신 state 업데이트
        self.odom_state = state



    def path_callback(self, path_msg: Path):
        """
        /planned_path 토픽 메시지가 수신될 때 호출되는 콜백 함수.
        전달된 path 메시지에서 각 포즈의 x, y, yaw 값을 추출한 뒤,
        odom 데이터와 결합하여 제어 명령을 생성하고 발행합니다.
        """
        # 경로 좌표와 yaw 값을 저장할 리스트 생성
        local_x = []
        local_y = []
        local_yaw = []

        for pose_stamped in path_msg.poses:
            local_x.append(pose_stamped.pose.position.x)
            local_y.append(pose_stamped.pose.position.y)
            # 쿼터니언을 오일러 각으로 변환 (yaw값 추출)
            q = pose_stamped.pose.orientation
            yaw = self.euler_from_quaternion(q)
            local_yaw.append(yaw)

        # 경로 데이터가 없으면 기본 전진 명령 발행 (속도 1 m/s)
        if not local_x or not local_y or not local_yaw:
            self.get_logger().warn("Path 메시지의 poses 필드에 데이터가 없습니다. 기본 속도 1.0 m/s로 전진합니다.")
            # ERP42 제어 메시지 구성
            cmd_msg = ControlMessage()
            cmd_msg.mora = 0  # 기본값
            cmd_msg.estop = 0  # 비상 정지 (0=해제, 1=정지)
            cmd_msg.gear = 2  # 전진(1), 후진(2), 중립(0)
            cmd_msg.speed =  int(1.0) * 10  # 속도 (0.1 km/h 단위 → 10km/h)
            cmd_msg.steer = 0 # 조향 각 (-2000~2000)
            cmd_msg.brake = 0 # 브레이크 (0~200)
            cmd_msg.alive = 1  # 패킷 생존 값            # 브레이크 없음
            self.cmd_pub.publish(cmd_msg)



        # odom 데이터가 아직 들어오지 않은 경우
        if self.odom_state is None:
            self.get_logger().warn("아직 odom 데이터가 수신되지 않았습니다.")
            return

        # odom에서 받아온 차량 상태를 이용
        state = self.odom_state

        # Stanley 제어 알고리즘 호출: state와 경로 정보를 입력으로 조향각 및 오차값 계산
        delta, target_idx, hdr, ctr, normed_hdr, cross_track_err = self.stanley.stanley_control(
            state, local_x, local_y, local_yaw, self.last_target_idx, reverse=True
        )
        # 업데이트된 타겟 인덱스 저장 (다음 계산에 사용)
        self.last_target_idx = target_idx

        # 타겟 속도 설정 (경로 상황에 따라 수정 가능, 예시: 3 m/s)
        target_speed = 3.0
        adapted_speed = 3.0  # SpeedSupporter 모듈을 사용하는 부분은 필요에 따라 수정

        # 간단한 brake 계산 예제: 현재 속도와 보정 속도 차이에 따라 산출
        if state.v * 3.6 >= adapted_speed:
            brake = (abs(state.v * 3.6 - adapted_speed) / 20.0) * 200
        else:
            brake = 0

        # ERP42 제어 메시지 구성
        cmd_msg = ControlMessage()
        cmd_msg.mora = 0  # 기본값
        cmd_msg.estop = 0  # 비상 정지 (0=해제, 1=정지)
        cmd_msg.gear = 2  # 전진(1), 후진(2), 중립(0)
        cmd_msg.speed = int(adapted_speed) * 10  # 속도 (0.1 km/h 단위 → 10km/h)
        cmd_msg.steer = int(degrees(-delta))  # 조향 각 (-2000~2000)
        cmd_msg.brake = int(brake)  # 브레이크 (0~200)
        cmd_msg.alive = 1  # 패킷 생존 값

        self.get_logger().info(f"발행 ERP42 커맨드: speed={cmd_msg.speed}, steer={cmd_msg.steer}, gear={cmd_msg.gear}, brake={cmd_msg.brake}")
        self.cmd_pub.publish(cmd_msg)


    def euler_from_quaternion(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
def main(args=None):
    rclpy.init(args=args)
    node = Erp42CommandSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("노드가 종료됩니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
