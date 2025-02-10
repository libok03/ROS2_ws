import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseArray
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import numpy as np
import math as m
from scipy.spatial import Delaunay
from scipy.spatial.distance import cdist
from scipy.interpolate import UnivariateSpline, CubicSpline
from scipy.spatial.transform import Rotation
sys.path.append('/home/ps/ros2_ws/src/global_path/global_path')
import time
#이거 경로를 현재 위치로부터 생성하기!
"""
과제 순서
1. 모든 콘 리스트에 x,y형태로 저장하기 v
2. 현재 차량 기준 가장 가까운, 먼 콘 구하기 v
3. 앞에서 구한 콘 기준으로 ABC 콘 중심 구하기 v
4. 콘 중심에서 r을 반지름으로 가지는 원 안에있는 콘 개수 구하기 v
5. min 구하기 v
"""

class ParkingPublisher(Node):
    def __init__(self):
        super().__init__("parking_publisher")

        #option
        self.r = 0.5 # 꼬깔 거리 
        self.d = 3.0 #원의 반지름
        self.cone_num = 15

        # Subscriptions
        self.cone_sub = self.create_subscription(PoseArray, "filtered_pose_array",self.cone_callback,10)
        self.odom_sub = self.create_subscription(Odometry, "/localization/kinematic_state",self.odom_callback,10)

        # Publishers
        self.path_pub = self.create_publisher(Path,'del_path',10)
        self.marker_pub = self.create_publisher(MarkerArray, "del_array",10)

        #Frame id
        self.frame_id = "map"

        #Initialize lists to store points
        self.cone_points = np.empty((0,2))

        # set flag
        self.flag_1 = False
        self.flag_2 = False
        self.front_forwarding = False

        # Timer
        self.timer = self.create_timer(0.25,self.path_create)
        self.get_logger().info("Initialization complete")

    def cone_callback(self,msg):
        # msg.poses
        for i in msg.poses:
            x_pose= i.position.x
            y_pose= i.position.y
            cnt_cone_point = np.array([x_pose,y_pose])
            if len(self.cone_points) <= self.cone_num:
                if not np.any(np.linalg.norm(self.cone_points - cnt_cone_point, axis=1) <= self.r):
                    self.cone_points = np.vstack([self.cone_points, cnt_cone_point])
        self.get_logger().info(f"cone callback complete filltered cone num:  {len(self.cone_points)}")

    def odom_callback(self, msg):
        # 차량의 orientation에서 yaw 값을 추출
        orientation = msg.pose.pose.orientation
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = Rotation.from_quat(quat).as_euler('xyz', degrees=False)
        yaw = euler[2]  # yaw는 z축 회전
        if not self.flag_1:
            self.first_point = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
            self.flag_1 = True
            self.get_logger().info("car map odom is now set")
            self.cur_point = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        else:
            self.cur_point = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def path_create(self):

        if not self.front_forwarding and self.flag_1:
            x_curr, y_curr, yaw = self.first_point  # yaw 포함

            # 전진 경로 계산 (20m 전진)
            forward_x = x_curr + 20 * np.cos(yaw)
            forward_y = y_curr + 20 * np.sin(yaw)

            waypoints = [[self.first_point[0], self.first_point[1]], [forward_x, forward_y]]
            self.make_path(waypoints)
            self.get_logger().info("scanning......")
            if cdist([[forward_x,forward_y]],[[self.cur_point[0],self.cur_point[1]]]) <= 1.0:
                self.front_forwarding = True
                self.get_logger().info("scan complete!!!")

        elif self.front_forwarding and not self.flag_2:
            a_mid_point = [32.8, 3.4]
            b_mid_point = [28.6, 3.9]
            c_mid_point = [24.3, 4.4]
            mid_points = [a_mid_point, b_mid_point, c_mid_point]
            self.idx = [
            self.distingish_cone_num(a_mid_point, self.cone_points),
            self.distingish_cone_num(b_mid_point, self.cone_points),
            self.distingish_cone_num(c_mid_point, self.cone_points)]
            print(self.idx)
            path = np.argmin(self.idx)
            waypoints=[self.cur_point,
                [mid_points[path][0]-2, mid_points[path][1]-3],
                mid_points[path],
                [mid_points[path][0]+1, mid_points[path][1]]]
            self.make_path(waypoints)
            self.get_logger().info("normal parking path is now created")
            self.flag_2 = True
        # if len(self.cone_points) >= self.cone_num and self.flag_1 and (not self.flag_2): # 여기 콘 개수 정해져 있으니 그걸로 바꾸면 좋을듯
        #     a_mid_point = [32.8, 3.4]
        #     b_mid_point = [28.6, 3.9]
        #     c_mid_point = [24.3, 4.4]
        #     mid_points = [a_mid_point, b_mid_point, c_mid_point]
        #     idx = [
        #     self.distingish_cone_num(a_mid_point, self.cone_points),
        #     self.distingish_cone_num(b_mid_point, self.cone_points),
        #     self.distingish_cone_num(c_mid_point, self.cone_points)]
        #     path = np.argmin(idx)
        #     waypoints=[self.cur_point,
        #         [mid_points[path][0]+2, mid_points[path][1]-3],#x로 +2, y로 -3
        #         mid_points[path],
        #         [mid_points[path][0]-1, mid_points[path][1]]] #x로 -1.5
        #     self.make_path(waypoints)
        #     self.get_logger().info("normal parking path is now created")
        #     self.flag_2 = True

        # elif self.flag_1 and (not self.flag_2):
        #     # 전진 후진 경로 계산
        #     x_curr, y_curr, yaw = self.first_point  # yaw 포함

        #     # 전진 경로 계산 (20m 전진)
        #     forward_x = x_curr + 20 * np.cos(yaw)
        #     forward_y = y_curr + 20 * np.sin(yaw)

        #     # 후진 경로 계산 (원래 위치로 되돌아오기)
        #     backward_x = x_curr - 20 * np.cos(yaw)
        #     backward_y = y_curr - 20 * np.sin(yaw)

        #     # 전진 후진 경로 생성
        #     waypoints = [[self.first_point[0], self.first_point[1]], [forward_x, forward_y], [backward_x, backward_y]]
        #     self.make_path(waypoints)
        #     self.get_logger().info("filtered cone is not reached ideal number")

        elif not self.flag_1:
            self.get_logger().info("your car odom is not yet subscribed")

        else:
            self.get_logger().info("Path is already generated")
            self.get_logger().info(f"root num:{self.idx}")




    # def find_closefar_points(self, starting_point, points):
    #     if len(points) >= 2:
    #         distances = cdist([starting_point], points)
    #         closest_idx = np.argmin(distances)
    #         farthest_idx = np.argmax(distances)
    #         return points[closest_idx], points[farthest_idx]

    def distingish_cone_num(self, mid_point, points):
        dists = cdist([mid_point], points).flatten()
        return np.sum(dists <= self.d)


    def make_path(self, waypoints):
        """웨이포인트를 기반으로 보간 경로 생성 및 퍼블리싱"""
        # X, Y 좌표 분리
        waypoints = np.array(waypoints)
        x = waypoints[:, 0]
        y = waypoints[:, 1]

        # Cubic Spline 보간법 적용
        num_samples = 100  # 경로의 세밀함 조정
        spline_x = CubicSpline(np.linspace(0, 1, len(x)), x)
        spline_y = CubicSpline(np.linspace(0, 1, len(y)), y)
        t = np.linspace(0, 1, num_samples)

        # 보간된 경로 생성
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        marker_array = MarkerArray()
        for i in range(num_samples):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = spline_x(t[i]).item()  # .item()을 사용해 float로 변환
            pose.pose.position.y = spline_y(t[i]).item()  # .item()을 사용해 float로 변환
            pose.pose.position.z = 0.0

            # 차량 방향 설정 (마지막 점의 방향 계산)
            if i < num_samples - 1:
                dx = spline_x(t[i + 1]) - spline_x(t[i])
                dy = spline_y(t[i + 1]) - spline_y(t[i])
            else:
                dx = spline_x(t[i]) - spline_x(t[i - 1])
                dy = spline_y(t[i]) - spline_y(t[i - 1])

            yaw = np.arctan2(dy, dx)
            orientation = Rotation.from_euler('z', yaw).as_quat()
            pose.pose.orientation.x = orientation[0]
            pose.pose.orientation.y = orientation[1]
            pose.pose.orientation.z = orientation[2]
            pose.pose.orientation.w = orientation[3]

            # # 차량 방향 설정 (마지막 점의 방향 직접 지정)
            # if i < num_samples - 1:
            #     dx = spline_x(t[i + 1]) - spline_x(t[i])
            #     dy = spline_y(t[i + 1]) - spline_y(t[i])
            #     yaw = np.arctan2(dy, dx)  # 일반적인 경우
            # else:
            #     # 마지막 점: 원하는 yaw 값 설정 (예: 90도 = π/2 라디안)
            #     yaw = np.deg2rad(90)  # 90도를 라디안으로 변환

            # # yaw를 쿼터니언으로 변환
            # orientation = Rotation.from_euler('z', yaw).as_quat()
            # pose.pose.orientation.x = orientation[0]
            # pose.pose.orientation.y = orientation[1]
            # pose.pose.orientation.z = orientation[2]
            # pose.pose.orientation.w = orientation[3]


            path.poses.append(pose)

            # Marker 생성
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "path_markers"
            marker.id = 1  # Marker ID는 고유해야 함
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.pose.position.x = pose.pose.position.x
            marker.pose.position.y = pose.pose.position.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2  # Marker 크기
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0  # 투명도
            marker.color.r = 0.0  # 빨간색
            marker.color.g = 1.0  # 초록색
            marker.color.b = 0.0  # 파란색

            marker_array.markers.append(marker)

        # 생성된 경로 퍼블리싱
        self.path_pub.publish(path)
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Path published with {num_samples} points (interpolated).")

def main(args=None):
    rclpy.init(args=args)
    node = ParkingPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
