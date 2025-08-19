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


from controller_obstacle import Obstacle
from controller_pickup_mj import Pickup
from controller_delivery_mj import Delivery
from controller_parking import Pakring
from controller_traffic_light import Trafficlight
from controller_stop_line import Stopline

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




class PID():
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain", 0.85).value
       
        self.p_err = 0.0
        self.i_err = 0.0
        self.speed = 0.0
        
        self.current = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        self.last = node.get_clock().now().seconds_nanoseconds()[0] + (node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
    def PIDControl(self, speed, desired_value, min, max):

        self.current = self.node.get_clock().now().seconds_nanoseconds()[0] + (self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
        dt = self.current - self.last
        self.last = self.current

        err = desired_value - speed
        # self.d_err = (err - self.p_err) / dt 
        self.p_err = err
        # print(speed)
        self.i_err += self.p_err * dt  * (0.0 if speed == 0 else 1.0)

        self.speed = speed + (self.p_gain * self.p_err) + (self.i_gain * self.i_err)
        return int(np.clip(self.speed, min, max))
    
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

    A1A2 = "driving_a"
    A2A3 = "pickup_b"
    A3A4 = "curve_c"
    A4A5 = "traffic_light_d"
    # A4A5 = "driving_d"
    A5A6 = "driving_e"
    A6A7 = "traffic_light_f"
    # A6A7 = "driving_f"
    A7A8 = "driving_g"
    A8A9 = "obstacle_h"
    A9A10 = "curve_i"
    A10A11 = "traffic_light_j"
    # A10A11 = "driving_j"
    A11A12 = "driving_k"
    A12A13 = "stop_line_l"
    A13A14 = "stop_line_m"
    A14A15 = "driving_o"
    A15A16 = "curve_p"
    A16A17 = "driving_q"
    A17A18 = "traffic_light_r"
    # A17A18 = "driving_r"
    A18A19 = "driving_s"
    A19A20 = "curve_t"
    A20A21 = "delivery_u"
    A21A22 = "traffic_light_v"
    # A21A22 = "driving_v"
    A22A23 = "driving_w"
    A23A24 = "traffic_light_x"
    # A23A24 = "driving_x"
    A24A25 = "driving_y"
    A25A26 = "curve_z"
    A26A27 = "obstacle_A"
    A27A28 = "curve_B"
    A28A29 = "driving_C"
    A29A30 = "parking_D"
    A30A31 = "driving_E"
    
    
    

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
        self.pid = PID(node)
        self.ss = SpeedSupporter(node)

        self.target_idx = 0
        self.current_idx = 0
        self.idx_len = 0
        self.mission_finish = False
        self.abs_var = None


        self.obstacle = Obstacle(self.node)
        self.pickup = Pickup(self.node)
        self.delivery = Delivery(self.node)
        self.parking = Pakring(self.node)
        self.traffic_light = Trafficlight(self.node)
        self.stop_line = Stopline(self.node)

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
                adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=10, max_value=20) # 에러(hdr, ctr) 기반 목표 속력 조정
                speed = self.pid.PIDControl(self.odometry.v * 3.6, adapted_speed, min=10, max=20) # speed 조정 (PI control) 
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
                adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=10, max_value=15) # 에러(hdr, ctr) 기반 목표 속력 조정
                speed = self.pid.PIDControl(self.odometry.v * 3.6, adapted_speed,  min=10, max=15) # speed 조정 (PI control) 
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
            msg, self.abs_var, self.mission_finish = self.pickup.control_pickup(self.odometry, self.path)

        elif self.state.value[:-2] == "delivery":
            msg, self.mission_finish = self.delivery.control_delivery(self.odometry, self.abs_var, self.path)  # abs_var 설정, A1 = 1, A2 = 2, A3 = 3, B1 = 4, B2 = 5, B3 = 6 으로 픽업에서 들어오는값의 +3으로 계산되어 나옴

        elif self.state.value[:-2] == "traffic_light":
            msg, self.mission_finish = self.traffic_light.control_traffic_light(self.odometry, self.path)

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
        print(self.abs_var)
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
    node.declare_parameter("file_name", "bsbs.db") #kcity
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