"""Log simulation scan and map->odom transform freshness during navigation.

This node is diagnostic only. It never fabricates or rebroadcasts transforms.
"""

import time

from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
import rclpy


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def age_seconds(now_ns, stamp):
    return (now_ns - stamp_ns(stamp)) / 1_000_000_000.0


class TfHealth(Node):
    def __init__(self):
        super().__init__('assignment5_tf_health')
        self.declare_parameter('stale_threshold', 0.75)
        self._limit = float(self.get_parameter('stale_threshold').value)
        self._scan_stamp = None
        self._tf_stamp = None
        self._odom_stamp = None
        # Match Nav2's reliable TF listener as well as observing best-effort /tf.
        # This distinguishes publisher freshness from a stalled reader/buffer.
        self._buffer = Buffer(node=self)
        self._listener = TransformListener(self._buffer, self)
        self._started = time.monotonic()
        self._last_clock_ns = None
        self._last_clock_advance = self._started
        self._last_print = 0.0
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self._on_tf,
                                 qos_profile_sensor_data)
        self.create_timer(2.0, self._check,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            'Watching /scan, map->odom, odom->base_link, reliable TF buffer and simulation time')

    def _on_scan(self, msg):
        self._scan_stamp = msg.header.stamp

    def _on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id.lstrip('/') == 'map' and \
                    tf.child_frame_id.lstrip('/') == 'odom':
                self._tf_stamp = tf.header.stamp
            if tf.header.frame_id.lstrip('/') == 'odom' and \
                    tf.child_frame_id.lstrip('/') == 'base_link':
                self._odom_stamp = tf.header.stamp

    def _check(self):
        now_wall = time.monotonic()
        now_ns = self.get_clock().now().nanoseconds
        if now_ns == 0:
            if now_wall - self._started > 8.0:
                self._warn('No simulation /clock yet (ROS time is zero)')
            return
        if now_ns != self._last_clock_ns:
            self._last_clock_ns = now_ns
            self._last_clock_advance = now_wall
        elif now_wall - self._last_clock_advance > 5.0:
            self._warn('Simulation /clock has not advanced in > 5 wall seconds')
            return

        if self._scan_stamp is None:
            if now_wall - self._started > 10.0:
                self._warn('No /scan received; check Gazebo sensor / bridge / QoS')
            return
        scan_age = age_seconds(now_ns, self._scan_stamp)
        if self._tf_stamp is None:
            if now_wall - self._started > 10.0:
                self._warn('No map->odom TF yet; set Initial Pose and check AMCL')
            return
        tf_age = age_seconds(now_ns, self._tf_stamp)
        try:
            # No blocking timeout: the diagnostic must keep processing /tf.
            buffered = self._buffer.lookup_transform('odom', 'map', rclpy.time.Time())
            buffer_age = age_seconds(now_ns, buffered.header.stamp)
        except TransformException as exc:
            self._warn(f'TF buffer cannot resolve map->odom: {exc}')
            return
        if scan_age > self._limit:
            self._warn(f'STALE /scan: age={scan_age:.2f}s (sim time) '
                       f'-- simulator/sensor/bridge may be delayed')
        elif tf_age > self._limit:
            self._warn(f'STALE map->odom TF: age={tf_age:.2f}s, '
                       f'/scan age={scan_age:.2f}s -- check AMCL / TF')
        elif buffer_age > self._limit:
            self._warn(f'STALE reliable TF buffer: age={buffer_age:.2f}s, '
                       f'best-effort map->odom age={tf_age:.2f}s '
                       '-- check DDS transport / subscriber')
        elif self._odom_stamp is None:
            self._warn('No odom->base_link TF; check robot odometry publisher')
        elif age_seconds(now_ns, self._odom_stamp) > self._limit:
            self._warn(f'STALE odom->base_link TF: '
                       f'age={age_seconds(now_ns, self._odom_stamp):.2f}s')
        elif now_wall - self._last_print > 12.0:
            self.get_logger().info(
                f'TF health OK: scan age={scan_age:.2f}s, '
                f'map->odom age={tf_age:.2f}s, buffer age={buffer_age:.2f}s, '
                f'odom->base_link age={age_seconds(now_ns, self._odom_stamp):.2f}s '
                '(negative TF age is normal)')
            self._last_print = now_wall

    def _warn(self, message):
        now_wall = time.monotonic()
        if now_wall - self._last_print > 5.0:
            self.get_logger().warning(message)
            self._last_print = now_wall


def main(args=None):
    rclpy.init(args=args)
    node = TfHealth()
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
