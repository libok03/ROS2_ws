import rclpy
from rclpy.node import Node
from erp42_msgs.msg import ControlMessage


class Erp42CmdPublisher(Node):
    def __init__(self):
        super().__init__("erp42_cmd_publisher")
        self.publisher_ = self.create_publisher(ControlMessage, "erp42_cmd", 10)
        self.timer = self.create_timer(0.1, self.publish_cmd)  # 10Hz 주기

    def publish_cmd(self):
        msg = ControlMessage()
        msg.mora = 0  # 기본값
        msg.estop = 0  # 비상 정지 (0=해제, 1=정지)
        msg.gear = 2  # 전진(1), 후진(2), 중립(0)
        msg.speed = 100  # 속도 (0.1 km/h 단위 → 10km/h)
        msg.steer = -28  # 조향 각 (-2000~2000)
        msg.brake = 0  # 브레이크 (0~200)
        msg.alive = 1  # 패킷 생존 값

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Publishing ERP42 Command: Speed={msg.speed}, Steer={msg.steer}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Erp42CmdPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
