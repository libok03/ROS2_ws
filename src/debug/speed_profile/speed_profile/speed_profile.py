import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from erp42_msgs.msg import ControlMessage  # 실제 메시지 타입으로 변경 필요
from visualization_msgs.msg import Marker, MarkerArray
import os
os.environ['DISPLAY'] = ''
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import matplotlib
matplotlib.use('Agg', force=True)
import matplotlib.pyplot as plt
from datetime import datetime
import signal
import math
from collections import deque

class VelocityMonitor(Node):
    def __init__(self, min_speed=0.5, max_speed=1.5, buffer_size=1000):
        super().__init__('velocity_monitor')

        self.declare_parameter('min_speed', min_speed)
        self.declare_parameter('max_speed', max_speed)
        self.declare_parameter('buffer_size', buffer_size)
        self.declare_parameter('update_rate', 10.0)
        
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        self.odom_sub = self.create_subscription(
            Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.cmd_sub = self.create_subscription(
            ControlMessage, '/cmd_msg', self.cmd_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/velocity_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/velocity_markers', 10)

        self.odom_vel = deque(maxlen=self.buffer_size)
        self.cmd_vel = deque(maxlen=self.buffer_size)
        self.odom_time = deque(maxlen=self.buffer_size)
        self.cmd_time = deque(maxlen=self.buffer_size)
        
        self.odom_count = 0
        self.cmd_count = 0
        self.violation_count = 0
        self.last_pose = None
        self.last_velocity = 0.0

        self.running = True
        self.start_time = self.get_clock().now()
        
        signal.signal(signal.SIGINT, self.handle_exit)
        signal.signal(signal.SIGTERM, self.handle_exit)

        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)

        self.get_logger().info(f"속도 모니터링 시작 - 범위: {self.min_speed:.2f} ~ {self.max_speed:.2f} m/s")

    def odom_callback(self, msg):
        linear_vel = msg.twist.twist.linear
        v = math.sqrt(linear_vel.x**2 + linear_vel.y**2)
        self.last_velocity = v
        current_time = self.get_clock().now()
        time_diff = (current_time - self.start_time).nanoseconds / 1e9
        self.odom_vel.append(v)
        self.odom_time.append(time_diff)
        self.odom_count += 1
        self.last_pose = msg.pose.pose
        if v < self.min_speed or v > self.max_speed:
            self.violation_count += 1
            self.get_logger().warn(f"속도 위반 감지: {v:.2f} m/s (범위: {self.min_speed:.2f}-{self.max_speed:.2f})")
        self.publish_marker(v)

    def cmd_callback(self, msg):
        try:
            if hasattr(msg, 'velocity'):
                v = abs(msg.velocity)
            elif hasattr(msg, 'speed'):
                v = abs(msg.speed)
            else:
                self.get_logger().error("Command 메시지에서 속도 필드를 찾을 수 없습니다.")
                return
            current_time = self.get_clock().now()
            time_diff = (current_time - self.start_time).nanoseconds / 1e9
            self.cmd_vel.append(v)
            self.cmd_time.append(time_diff)
            self.cmd_count += 1
        except Exception as e:
            self.get_logger().error(f"Command 콜백 에러: {str(e)}")

    def publish_marker(self, velocity):
        marker_array = MarkerArray()
        
        # 차량 이름 텍스트 마커
        name_marker = Marker()
        name_marker.header.frame_id = "map"
        name_marker.header.stamp = self.get_clock().now().to_msg()
        if self.last_pose is not None:
            name_marker.pose.position = self.last_pose.position
            name_marker.pose.orientation = self.last_pose.orientation
            name_marker.pose.position.z += 2.5
        else:
            name_marker.pose.position.x = 0.0
            name_marker.pose.position.y = 0.0
            name_marker.pose.position.z = 1.0
            name_marker.pose.orientation.w = 1.0

        # 속도 텍스트 마커 (차량 이름 위에 속도 표시)
        text_marker = Marker()
        text_marker.header = name_marker.header
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.id = 1
        text_marker.ns = "velocity_text"
        text_marker.text = f"{velocity:.2f} m/s"
        text_marker.pose = name_marker.pose
        text_marker.pose.position.z += 1.0  # 이름 위에 속도 표시
        text_marker.scale.z = 0.7
        # 속도에 따른 색상
        if velocity < self.min_speed:
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 1.0
        elif velocity > self.max_speed:
            text_marker.color.r = 1.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
        else:
            text_marker.color.r = 0.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.0
        text_marker.color.a = 1.0

        marker_array.markers.append(name_marker)
        marker_array.markers.append(text_marker)
        self.marker_pub.publish(name_marker)
        self.marker_pub.publish(text_marker)
        self.marker_array_pub.publish(marker_array)

    def timer_callback(self):
        if self.odom_count > 0:
            current_odom_vel = self.odom_vel[-1] if self.odom_vel else 0.0
            self.get_logger().info(
                f"통계 - Odom: {self.odom_count}개, Cmd: {self.cmd_count}개, "
                f"현재속도: {current_odom_vel:.2f} m/s, 위반: {self.violation_count}회"
            )

    def handle_exit(self, signum, frame):
        self.get_logger().info("종료 시그널 감지, 그래프 저장 후 종료합니다.")
        self.running = False

    def get_statistics(self):
        if not self.odom_vel:
            return None
        odom_velocities = list(self.odom_vel)
        return {
            'avg_velocity': sum(odom_velocities) / len(odom_velocities),
            'max_velocity': max(odom_velocities),
            'min_velocity': min(odom_velocities),
            'violation_rate': self.violation_count / len(odom_velocities) * 100
        }

def main(args=None):
    rclpy.init(args=args)
    node = VelocityMonitor(min_speed=0.5, max_speed=1.5)
    plot_counter = 0
    
    def create_plot():
        plt.switch_backend('Agg')
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle('real-time speed profile')
        ax1.clear()
        ax1.axhspan(node.min_speed, node.max_speed, color='green', alpha=0.2, label='safe range')
        ax1.axhline(node.min_speed, color='red', linestyle='--', linewidth=1, label='min speed')
        ax1.axhline(node.max_speed, color='red', linestyle='--', linewidth=1, label='max speed')
        if len(node.odom_time) > 0 and len(node.odom_vel) > 0:
            ax1.plot(list(node.odom_time), list(node.odom_vel), label='Odometry', color='blue', linewidth=2)
        if len(node.cmd_time) > 0 and len(node.cmd_vel) > 0:
            ax1.plot(list(node.cmd_time), list(node.cmd_vel), label='Command', color='green', linewidth=2)
        ax1.set_xlabel('시간 (초)')
        ax1.set_ylabel('속도 (m/s)')
        ax1.set_title('속도 추이')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax2.clear()
        if len(node.odom_vel) > 10:
            ax2.hist(list(node.odom_vel), bins=20, alpha=0.7, color='blue', edgecolor='black')
            ax2.axvline(node.min_speed, color='red', linestyle='--', linewidth=2, label='최소 속도')
            ax2.axvline(node.max_speed, color='red', linestyle='--', linewidth=2, label='최대 속도')
            ax2.set_xlabel('속도 (m/s)')
            ax2.set_ylabel('빈도')
            ax2.set_title('속도 분포')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        stats = node.get_statistics()
        if stats:
            info_text = f"평균: {stats['avg_velocity']:.2f} m/s\n"
            info_text += f"최대: {stats['max_velocity']:.2f} m/s\n"
            info_text += f"최소: {stats['min_velocity']:.2f} m/s\n"
            info_text += f"위반율: {stats['violation_rate']:.1f}%"
            ax1.text(0.02, 0.98, info_text, transform=ax1.transAxes, 
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        plt.tight_layout()
        temp_filename = "/tmp/velocity_monitor_live.png"
        fig.savefig(temp_filename, dpi=150, bbox_inches='tight')
        plt.close(fig)
        return temp_filename

    try:
        node.get_logger().info("속도 모니터링 시작. 실시간 그래프는 /tmp/velocity_monitor_live.png에서 확인")
        node.get_logger().info("RViz에서 Marker 토픽 '/velocity_marker' 또는 '/velocity_markers' 추가하세요")
        while node.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            plot_counter += 1
            if plot_counter >= 50:
                try:
                    create_plot()
                    plot_counter = 0
                except Exception as e:
                    node.get_logger().error(f"그래프 생성 에러: {str(e)}")
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 종료됨")
    finally:
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"velocity_monitor_{timestamp}.png"
            plt.switch_backend('Agg')
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
            fig.suptitle('final speed profile')
            if len(node.odom_time) > 0 and len(node.odom_vel) > 0:
                ax1.plot(list(node.odom_time), list(node.odom_vel), label='Odometry', color='blue', linewidth=2)
            if len(node.cmd_time) > 0 and len(node.cmd_vel) > 0:
                ax1.plot(list(node.cmd_time), list(node.cmd_vel), label='Command', color='green', linewidth=2)
            ax1.axhspan(node.min_speed, node.max_speed, color='green', alpha=0.2, label='safe range')
            ax1.axhline(node.min_speed, color='red', linestyle='--', linewidth=1, label='min speed')
            ax1.axhline(node.max_speed, color='red', linestyle='--', linewidth=1, label='max speed')
            ax1.set_xlabel('time (s)')
            ax1.set_ylabel('speed (m/s)')
            ax1.set_title('speed trend')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            if len(node.odom_vel) > 10:
                ax2.hist(list(node.odom_vel), bins=30, alpha=0.7, color='blue', edgecolor='black')
                ax2.axvline(node.min_speed, color='red', linestyle='--', linewidth=2, label='min speed')
                ax2.axvline(node.max_speed, color='red', linestyle='--', linewidth=2, label='max speed')
                ax2.set_xlabel('speed (m/s)')
                ax2.set_ylabel('frequency')
                ax2.set_title('final speed distribution')
                ax2.legend()
                ax2.grid(True, alpha=0.3)
            plt.tight_layout()
            fig.savefig(filename, dpi=300, bbox_inches='tight')
            plt.close(fig)
            node.get_logger().info(f"최종 그래프 저장 완료: {filename}")
        except Exception as e:
            node.get_logger().error(f"최종 그래프 저장 에러: {str(e)}")
        stats = node.get_statistics()
        if stats:
            node.get_logger().info(f"최종 통계 - 평균속도: {stats['avg_velocity']:.2f} m/s, "
                                 f"위반율: {stats['violation_rate']:.1f}%")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
