#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from erp42_msgs.msg import SerialFeedBack, ControlMessage

from stanley import Stanley
from DB import DB
import numpy as np
import math as m


from enum import Enum
import threading


# from controller_obstacle import Obstacle
# from controller_pickup import Pickup
# from controller_delivery import Delivery
# from controller_parking import Pakring
# # from controller_traffic_light import Trafficlight
# from controller_stop_line import Stopline

from Modifier_param import set_param



def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return qx, qy, qz, qw


class SMC:
    def __init__(self, node):
        """
        SMC 제어 클래스 (입출력 단위: km/h)

        ROS2 노드를 받아 파라미터를 선언 및 초기 설정합니다.
        """
        self.node = node

        # 슬라이딩 모드 제어 이득
        self.lambda_gain = 3.0
        self.k_gain = 5.0
        self.tol =  1.0

        # 내부 상태 변수
        self.prev_error = 0.0

        # ROS2 시간 초기화
        now = node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + now[1] / 1e9
        self.last = self.current

    def saturation(self, s):
        """포화 함수: [-1, 1]로 제한"""
        return max(min(s / self.tol, 1.0), -1.0)

    def SMCControl(self, current_speed_kph, desired_speed_kph, min, max):
        """
        SMC 제어 입력 계산 함수 (단위: km/h)

        입력:
        - current_speed_kph: 현재 속도 (km/h)
        - desired_speed_kph: 목표 속도 (km/h)
        - min_kph: 제어 출력 최소 범위 (km/h)
        - max_kph: 제어 출력 최대 범위 (km/h)

        반환:
        - new_speed_kph: SMC 제어 후 출력 속도 (km/h)
        """
        # ROS 시간 계산
        dt = 0.1
        self.last = self.current
        if dt <= 0.0:
            dt = 1e-5

        # 단위 변환: km/h → m/s
        current_speed_mps = current_speed_kph / 3.6
        desired_speed_mps = desired_speed_kph / 3.6

        # 오차 + 슬라이딩면 계산
        error = desired_speed_mps - current_speed_mps
        error_dot = (error - self.prev_error) / dt
        self.prev_error = error
        s = error_dot + self.lambda_gain * error

        # 슬라이딩 모드 제어 신호 계산
        u = self.lambda_gain * error_dot + self.k_gain * self.saturation(s)

        # 업데이트된 속도 계산
        new_speed_mps = current_speed_mps + u * dt
        
        # m/s → km/h로 다시 변환 후, 출력 제한 적용
        new_speed_kph = new_speed_mps * 3.6
        new_speed_kph = np.clip(new_speed_kph, min, max)
        print(f"curr={current_speed_kph:.2f}  target={desired_speed_kph:.2f}  error={error:.2f}  error_dot={error_dot:.2f}  s={s:.2f}  sat(s)={self.saturation(s):.2f}  u={u:.2f}  next={new_speed_kph:.2f}")


        return int(new_speed_kph)

    
class SpeedSupporter():
    def __init__(self, node):
        # self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 30.0).value
        # self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 20.0).value

        # self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.01).value
        # self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.02).value

        self.he_gain = node.declare_parameter("/speed_supporter/he_gain", 50.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain", 30.0).value

        self.he_thr = node.declare_parameter("/speed_supporter/he_thr",0.001).value
        self.ce_thr = node.declare_parameter("/speed_supporter/ce_thr",0.002).value

        
    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self,value,hdr,ctr,min_value,max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        res = np.clip(value + err, min_value, max_value)
        return res




class State(Enum):    
#kcity 본선 대회용 (final - 1012)
    A1A2 = "driving_a"  #13
    A2A3 = "driving_b"
    A3A4 = "driving_c"
    A4A5 = "driving_d"
    A5A6 = "driving_e"
    A6A7 = "driving_f"
    A7A8 = "driving_g"
    # A2A3 = "pickup_b"  #9
    # A3A4 = "curve_c"  #8
    # A4A5 = "curve_d"  #8
    # A5A6 = "obstacle_e"  #6
    # A6A7 = "curve_f"  #8
    # A7A8 = "stop_line_a"  #8
    # A8A9 = "stop_line_b"  #8
    # A9A10 = "curve_h"  #8
    # A10A11 = "driving_i"  #8
    # A11A12 = "curve_j"  #8
    # A12A13 = "driving_k"  #8
    # A13A14 = "driving_l"  #15
    # A14A15 = "obstacle_m"  #6
    # A15A16 = "curve_n"  #8
    # A16A17 = "driving_o"  #8
    # A17A18 = "driving_p"  #10
    # A18A19 = "delivery_q"  #7 #delivery
    # A19A20 = "driving_r"  #8
    # A20A21 = "driving_s"  #8
    # A21A22 = "driving_t"  #10
    # A22A23 = "driving_u"  #8
    # A23A24 = "curve_v"  #10
    # A24A25 = "driving_w"  #15
    # A25A26 = "curve_x"  #11
    # A26A27 = "stop_line_c"  #8
    # A27A28 = "curve_y"  #8
    # A28A29 = "driving_z"  #13
    # A29A30 = "driving_A"  #8
    # A30A31 = "driving_B"  #16
    # A31A32 = "driving_C"  #8
    # A32A33 = "driving_D"  #15
    # A33A34 = "parking_E"  #6
    # A34A35 = "driving_E"  #15

    # A1A2 = "driving_a"
    # A2A3 = "pickup_b"
    # A3A4 = "curve_c"
    # A4A5 = "obstacle_d"
    # A5A6 = "driving_e"
    # A6A7 = "stop_line_f"
    # A7A8 = "curve_g"
    # A8A9 = "parking_h"
    # A9A10 = "driving_j"
    # A10A11 = "delivery_i"
    # A11A12 = "curve_k"
    # A12A13 = "driving_l"

## Parking test 용
    # A1A2 = "parking_a"
    # A2A3 = "driving_b"

    # A1A2 = "driving_a"  #13
    # A2A3 = "obstacle_b"  #9
    # A3A4 = "driving_c"  #8
    # A4A5 = "driving_d"  #8
    # A5A6 = "driving_e"  #6
    # A6A7 = "curve_f"  #8
    # A7A8 = "driving_s"  #8
    # A8A9 = "curve_c"  #8
    # A9A10 = "driving_h"  #8
    # A10A11 = "driving_i"  #8
   

class GetPath():
    def __init__(self, db, init_state):
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.cv = []
        self.db = db

        self.file_open_with_id(init_state.name)


    def file_open_with_id(self, id):
        self.cx, self.cy, self.cyaw, self.cv = self.db.query_from_id(id)



class GetOdometry():
    def __init__(self, node, odom_topic):
        self.x = 0.  # m
        self.y = 0.  # m
        self.yaw = 0.  # rad
        self.v = 0.  # m/s

        self.node = node

        self.node.create_subscription(Odometry, odom_topic, self.callback, qos_profile=qos_profile_system_default)
        self.node.create_subscription(SerialFeedBack, "erp42_feedback", self.callback_erp, qos_profile=qos_profile_system_default)

    def callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _,_,self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        
    def callback_erp(self,msg):
        self.v = msg.speed

class StateMachine():
    def __init__(self, node, odometry, path, state):
        self.pub = node.create_publisher(ControlMessage, "cmd_msg", qos_profile=qos_profile_system_default)
        self.path_pub = node.create_publisher(Path, "global_path", qos_profile=qos_profile_system_default)
        self.node = node
        self.state = state
        self.path = path
        self.odometry = odometry

        self.st = Stanley()
        self.smc = SMC(node)
        self.ss = SpeedSupporter(node)

        self.target_idx = 0
        self.current_idx = 0
        self.idx_len = 0
        self.mission_finish = False


        # self.obstacle = Obstacle(self.node)
        # self.pickup = Pickup(self.node)
        # self.delivery = Delivery(self.node)
        # self.parking = Pakring(self.node)
        # # self.traffic_light = Trafficlight(self.node)
        # self.stop_line = Stopline(self.node)

        self.trial = 0




    def update_state_and_path(self):
        if self.state.value[:-2] == "driving" or self.state.value[:-2] == "curve":
            if self.target_idx >= len(self.path.cx) - 10: # driving에서 state 전환 조건
                states = list(State)
                current_index = states.index(self.state)
                try:
                    self.state = states[current_index + 1]  # state update (driving -> mission)
                    self.path.file_open_with_id(self.state.name) # path update
                    self.publish_path()  # path publish
                    self.mission_finish = False
                except IndexError:
                    print("index out of range")
        else:
            if self.mission_finish: # mission에서 state 전환 조건
                states = list(State)
                current_index = states.index(self.state)
                try:
                    self.state = states[current_index + 1]  # state update (mission -> driving)
                    self.path.file_open_with_id(self.state.name) # path update
                    self.publish_path() # path publish
                except IndexError:
                    print("index out of range")



    def update_cmd_msg(self):
        # print(self.state.value)
        msg = ControlMessage()
        self.current_idx = self.target_idx

        if self.state.value[:-2] == "driving":
            if self.odometry.x != 0.: #10.03 수정
                steer, self.target_idx, hdr, ctr = self.st.stanley_control(self.odometry, self.path.cx, self.path.cy, self.path.cyaw, h_gain=1.0, c_gain=1.0)
                target_speed = self.set_target_speed()
                adapted_speed = self.ss.adaptSpeed(target_speed,hdr,ctr,min_value= 5,max_value=15)
                speed = self.smc.SMCControl(self.odometry.v * 3.6, adapted_speed, min=0, max=15) # speed 조정 (PI control) 
                brake = self.cacluate_brake(adapted_speed) # brake 조정

                # msg.speed = int(adapted_speed) * 10
                msg.speed = int(speed) * 10
                msg.steer = int(m.degrees((-1) * steer))
                msg.gear = 2
                msg.brake = int(brake)
            else:
                pass

        elif self.state.value[:-2] == "curve":
            if self.odometry.x != 0.: #10.03 수정
                steer, self.target_idx, hdr, ctr = self.st.stanley_control(self.odometry, self.path.cx, self.path.cy, self.path.cyaw, h_gain=1.0, c_gain=1.0)
                target_speed = self.set_target_speed()
                adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=4, max_value=10) # 에러(hdr, ctr) 기반 목표 속력 조정
                speed = self.smc.SMCControl(self.odometry.v * 3.6, adapted_speed,  min=4, max=10) # speed 조정 (PI control) 
                brake = self.cacluate_brake(adapted_speed) # brake 조정

                # msg.speed = int(adapted_speed) * 10
                msg.speed = int(speed) * 10
                msg.steer = int(m.degrees((-1) * steer))
                msg.gear = 2
                msg.brake = int(brake)
            else:
                pass
        
        elif self.state.value[:-2] == "parking":
            if self.trial < 1: # 초기 시도 횟수 1
                try:
                    set_param("bs_cropbox_filter","detection_area","[-2.,4.,-4.,0.]")

                except:
                    print("set param Fail")
                else:
                    print("set param Success")
                    self.trial +=1 

            msg, self.mission_finish = self.parking.control_parking(self.odometry)

        elif self.state.value[:-2] == "obstacle":
            msg, self.mission_finish = self.obstacle.control_obstacle(self.odometry, self.path)

        elif self.state.value[:-2] == "pickup":
            msg, self.mission_finish = self.pickup.control_pickup(self.odometry, self.path)
        
        elif self.state.value[:-2] == "delivery":
            msg, self.mission_finish = self.delivery.control_delivery(self.odometry, "B1")  # abs_var 설정, B1은 예시로 사용됨

        # elif self.state.value[:-2] == "traffic_light":
        #     msg, self.mission_finish = self.traffic_light.control_traffic_light(self.odometry, self.path)

        elif self.state.value[:-2] == "stop_line":
            msg, self.mission_finish = self.stop_line.control_stop_line(self.odometry, self.path)

        else:
            print("error: ", self.state.value)
            

        return msg



    def set_target_speed(self):
        target_speed = self.path.cv[self.target_idx]
        return target_speed


    def cacluate_brake(self, adapted_speed): # brake 값 정하는 알고리즘 좀 더 정교하게 생각
        if self.odometry.v * 3.6 >= adapted_speed:
            brake = (abs(self.odometry.v * 3.6 - adapted_speed) / 20.0) * 200
            brake = np.clip(brake, 0, 100)
        else:
            brake = 0
        return brake


    def publish_cmd(self):
        self.update_state_and_path()
        msg = self.update_cmd_msg()
        self.pub.publish(msg)
        print(f"목표 인덱스: {self.target_idx}, 남은 인덱스:{len(self.path.cx)-self.target_idx}, STATE:{self.state.value[:-2]}")
        

    def publish_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"  # Assuming frame_id is 'map', adjust as necessary

        for x, y, yaw in zip(self.path.cx, self.path.cy, self.path.cyaw):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0  # Assuming the path is on the ground, adjust if needed
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)



def main():
    rclpy.init(args=None)
    node = rclpy.create_node("state_machine_node")

    # Declare Params
    node.declare_parameter("file_name", "YS_final.db") #kcity
    # node.declare_parameter("file_name", "global_path.db") #dolge
    # node.declare_parameter("file_name", "good.db") #bunsudae
    node.declare_parameter("odom_topic", "/localization/kinematic_state")


    # Get Params
    file_name = node.get_parameter("file_name").get_parameter_value().string_value
    odom_topic = node.get_parameter("odom_topic").get_parameter_value().string_value



    #Declare Instance
    db = DB(file_name)
    state = State.A1A2
    path = GetPath(db, state)
    odometry = GetOdometry(node, odom_topic)
    state_machine = StateMachine(node, odometry, path, state)
    state_machine.publish_path() # A1A2(초기 path) pubF


    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    rate = node.create_rate(20)

    while rclpy.ok():
        try:
            state_machine.publish_cmd()
        except Exception as ex:
            print(ex)
        rate.sleep()


if __name__ == "__main__":
    main()