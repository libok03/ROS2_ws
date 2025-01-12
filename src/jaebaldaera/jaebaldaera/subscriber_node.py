import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__("subscriber_node")
        qos_profile = QoSProfile(depth=10)
        self.subscriber_ = self.create_subscription(
            String,
            "topic",
            self.listener_callback,
            qos_profile)
        self.get_count=0
        self.set_count=0

    def listener_callback(self,msg):
        if msg.data == "get":
            self.get_count +=1
        elif msg.data == "set":
            self.set_count +=1
        self.get_logger().info(f"""
{msg.data}라는 소리를 들었습니다!
get은 {self.get_count} 번
set은 {self.set_count} 번 들림요""")

def main(args=None):
	rclpy.init(args=args)
	node = SubscriberNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info("응 멈출게")

	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
