#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, PoseStamped
from erp42_msgs.msg import SerialFeedBack, ControlMessage
# DWAPlanner는 dwa_planner.py 에 정의되어 있다고 가정
from dwa_planner import DWAPlanner, DWAConfig



class DWAPlannerNode(Node):
    def __init__(self):
        super().__init__('dwa_planner_node')

        # 장애물 받아오기
        self.sub_obs = self.create_subscription(
            PoseArray, '/cone_poses', self.obstacle_callback, 10)
        # ERP42 현재 속도·steer 받아오기
        self.sub_fb = self.create_subscription(
            SerialFeedBack, '/erp42_feedback', self.feedback_callback, 10)
        # 제어 명령 발행
        self.pub_cmd = self.create_publisher(
            ControlMessage, '/cmd_msg', 10)
        self.pub_path = self.create_publisher(Path, 'dwa_trajectory', 10)


        self.planner = DWAPlanner(DWAConfig())
        self.obstacles = []    # [[ox, oy], ...]
        self.current_v = 0.0   # m/s
        # yaw는 기준 방향(앞)을 0으로 고정
        self.current_yaw = 0.0

        # 10Hz로 DWA 실행
        self.timer = self.create_timer(0.1, self.timer_callback)

    def obstacle_callback(self, msg: PoseArray):
        thresh = 0.1  # [m]
        self.obstacles = []
        for pose in msg.poses:
            dx = pose.position.x
            dy = pose.position.y
            if math.hypot(dx, dy) > thresh:
                self.obstacles.append([dx, dy])
        self.get_logger().debug(f'obstacles filtered: {len(self.obstacles)} points')


    def feedback_callback(self, msg: SerialFeedBack):
        # speed 필드는 uint16 (cm/s 등)로 넘어올 수 있으니 단위 변환이 필요하면 조정
        # 여기서는 msg.speed -> m/s 라고 가정
        self.current_v = msg.speed

    def timer_callback(self):
        # 현재 (x,y) = (0,0), yaw=0 으로 가정
        best_traj = self.planner.plan(
            self.obstacles,
            x=0.0, y=0.0,
            yaw=self.current_yaw,
            v=self.current_v
        )

        if len(best_traj) < 2:
            self.get_logger().warn("no DWA_route found")
            return
        # best_traj[0] 은 현재 state, best_traj[1] 의 v와 yaw 차이로 u 추정
        # dt = self.planner.cfg.dt 만큼의 예측이니까
        next_state = best_traj[1]
        target_v = next_state[3]
        # yaw_rate = Δyaw / dt
        yaw_rate = (next_state[2] - best_traj[0][2]) / self.planner.cfg.dt

        # CmdMsg 에 채워넣기
        cmd = ControlMessage()
        cmd.mora = 0            # 예: 고정값
        cmd.estop = 0
        cmd.gear = 2            # 2: 전진
        cmd.speed = int(target_v*3.6) * 10
        raw_steer = (yaw_rate / self.planner.cfg.max_yaw_rate) * 28.0
        cmd.steer  = int(max(-28, min(28, raw_steer)))
        cmd.brake = 0
        cmd.alive = 1

        self.pub_cmd.publish(cmd)
        self.get_logger().debug(
            f'pub cmd → speed: {cmd.speed}, steer: {cmd.steer}'
        )
        
        # 1) Path 메시지 생성
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'base_link'  # RViz에서 base_link 기준으로 표시

        # 2) 각 궤적 점을 PoseStamped로 변환
        for x, y, yaw, v in best_traj:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            # yaw → quaternion
            ps.pose.orientation.z = math.sin(yaw/2.0)
            ps.pose.orientation.w = math.cos(yaw/2.0)
            path_msg.poses.append(ps)

        # 3) 퍼블리시
        self.pub_path.publish(path_msg)

        self.get_logger().debug(f'Published DWA path with {len(best_traj)} points')


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
