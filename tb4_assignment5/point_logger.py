"""Print RViz Publish Point coordinates in the map frame for goal collection."""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class PointLogger(Node):
    def __init__(self):
        super().__init__('assignment5_point_logger')
        self.create_subscription(PointStamped, 'clicked_point', self._on_point, 10)
        self.get_logger().info(
            'RViz: set Fixed Frame to map, select Publish Point and click three '
            'distinct free spaces. Copy x/y into config/goals.yaml and choose yaw.')

    def _on_point(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning(
                f'Clicked frame is {msg.header.frame_id!r}, not map. '
                'Switch RViz Fixed Frame to map.')
            return
        self.get_logger().info(
            f'Clicked map point: x: {msg.point.x:.3f}, y: {msg.point.y:.3f}, '
            'yaw: 0.0')


def main(args=None):
    rclpy.init(args=args)
    node = PointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
