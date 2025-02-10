import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointField
import numpy as np
import struct

def trapezoid_filter(y, x, x_min, x_max, y_min_bottom, y_max_bottom, y_min_top, y_max_top):
    if x < x_min or x > x_max:
        return False

    ratio = (x - x_min) / (x_max - x_min)
    y_min = y_min_bottom + ratio * (y_min_top - y_min_bottom)
    y_max = y_max_bottom + ratio * (y_max_top - y_max_bottom)

    return y_min <= y <= y_max

class VelodyneSubscriber(Node):
    def __init__(self):
        super().__init__('change_data_size')
        self.velodyne_raw_subscriber = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.velodyne_callback,
            10)
        self.cropbox_publisher = self.create_publisher(PointCloud2, '/after_filter', 10)

    def velodyne_callback(self, msg):
        pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pc_array = np.array(list(pc_data))
# 사다리꼴 파라미터 설정
        x_min, x_max = -10, 10
        y_min_bottom, y_max_bottom = -10, 10
        y_min_top, y_max_top = 10, 20

        mask = np.array([trapezoid_filter(point[1], point[0], x_min, x_max, y_min_bottom, y_max_bottom, y_min_top, y_max_top) for point in pc_array])


        filtered_pc = pc_array[mask]

        fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

        processed_msg = PointCloud2()
        processed_msg.header = msg.header
        processed_msg.height = msg.height
        processed_msg.width = len(filtered_pc)
        processed_msg.fields = msg.fields
        processed_msg.is_bigendian = msg.is_bigendian
        processed_msg.point_step = msg.point_step
        processed_msg.row_step = processed_msg.width * msg.point_step
        processed_msg.is_dense = msg.is_dense


        # Convert filtered points back to byte array
        buffer = bytearray()
        for point in filtered_pc:
            buffer.extend(point.tobytes())
        processed_msg.data = bytes(buffer)


        if len(processed_msg.data) == processed_msg.width * processed_msg.point_step:
            self.cropbox_publisher.publish(processed_msg)
        else:
            self.get_logger().error(f"Data size ({len(processed_msg.data)} bytes) does not match width ({processed_msg.width}) times point_step ({processed_msg.point_step}).")


def main(args=None):
    rclpy.init(args=args)
    node = VelodyneSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
