import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class AfterCropSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('pub_sub_test')

        self.after_crop_subscriber = self.create_subscription(
            PointCloud2,
            'velodyne_points',
            self.after_crop_callback,
            10
        )

        self.afaf_publisher = self.create_publisher(PointCloud2, 'complete', 10)

        self.get_logger().info('Node initialized. Subscribing to "after_crop" and publishing to "afaf".')

    def after_crop_callback(self, msg):
        # 메시지 처리 로직 (현재는 단순히 메시지를 전달)
        self.get_logger().info('Received message from "after_crop". Publishing to "afaf".')

        # 수신한 메시지를 그대로 발행
        self.afaf_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = AfterCropSubscriberPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
