import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String




class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node') # pub라는 이름의 노드를 생성하겠다!
        qos_profile = QoSProfile(depth=10) #퍼블리셔 data buffer개수를 10로 설정 예기치 못한경우 퍼블리시할 데이터를 10개까지 저장한다는 뜻
        self.publisher_ = self.create_publisher(String, "topic", qos_profile) #토픽메시지에 상요한 내용으로 타입, 내용, 설정 이 3개지 결정하는 내용이다.
        timer_period = 1.0#초
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count=0
        self.getsetlist=[]
        with open("/home/libok/dev_ws/src/jaebaldaera/jaebaldaera/query.dat","r") as file:
            for line in file:
                command = line.split(",")[0]
                if command in ["get","set"]:
                    self.getsetlist.append(command)


    def timer_callback(self):

        try:
            msg=String()
            word = self.getsetlist.pop(0)
            msg.data = word
            self.publisher_.publish(msg)
            self.get_logger().info(f"니가 보낸 단어:{msg.data}임")
            self.count +=1
        except IndexError:
            self.get_logger().error("End of Line 이제 다 끝이야~")

def main(args=None):
    rclpy.init(args=args)
    node=PublisherNode()
    try:
        rclpy.spin(node) #ros2 노드 부분을 한번씩 돌려라!
    except KeyboardInterrupt:
        node.get_logger().info("정지함 ㅇㅇ")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
