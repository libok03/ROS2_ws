from geometry_msgs.msg import PoseArray, Pose
from rclpy.node import Node
import rclpy
import numpy as np


class PoseFilter(Node):
    def __init__(self):
        super().__init__('pose_filter')

        # Subscriptions
        self.pose_array_sub = self.create_subscription(
            PoseArray, 'cone_pose_map', self.pose_array_callback, 10)

        # Publishers
        self.filtered_pose_array_pub = self.create_publisher(
            PoseArray, 'filtered_pose_array', 10)

        # 사각형의 네 점 정의 (시계방향 또는 반시계방향)
        self.rectangle_points = [
            (34.02, 1.92),  # Point 1 (x1, y1)
            (20.37, 3.61),   # Point 2 (x2, y2)
            (20.99, 7.64),   # Point 3 (x3, y3)
            (34.89, 5.87)   # Point 4 (x4, y4)
        ]

    def pose_array_callback(self, msg: PoseArray):
        """입력 PoseArray를 필터링하여 사각형 내부의 Pose만 추출"""
        filtered_poses = []

        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y

            # 점이 사각형 안에 있는지 확인
            if self.is_point_in_rectangle(x, y):
                filtered_poses.append(pose)

        # 필터링된 PoseArray 생성
        filtered_pose_array = PoseArray()
        filtered_pose_array.header = msg.header  # 원래 헤더 유지
        filtered_pose_array.poses = filtered_poses

        # 결과 퍼블리시
        self.filtered_pose_array_pub.publish(filtered_pose_array)
        self.get_logger().info(f"Filtered {len(filtered_poses)} poses from PoseArray.")

    def is_point_in_rectangle(self, x, y):
        """사각형 내부에 점이 포함되어 있는지 확인"""
        rectangle = np.array(self.rectangle_points)
        point = np.array([x, y])

        # 다각형 포함 여부 확인 (Ray-Casting Algorithm)
        n = len(rectangle)
        inside = False
        px, py = point

        for i in range(n):
            x1, y1 = rectangle[i]
            x2, y2 = rectangle[(i + 1) % n]

            if ((y1 > py) != (y2 > py)) and (px < (x2 - x1) * (py - y1) / (y2 - y1) + x1):
                inside = not inside

        return inside


def main(args=None):
    rclpy.init(args=args)
    node = PoseFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
