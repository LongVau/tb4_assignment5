
"""Reliable, sequential Nav2 navigation for three goals.

Each NavigateToPose request has a ComputePathToPose
preflight check to detect empty or degenerate plans.
"""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import ComputePathToPose, NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from .goal_config import load_goals, yaw_quaternion
from .path_checks import assess_nav2_path, close_goal_pairs


class SequentialNavigator(Node):
    def __init__(self):
        super().__init__('assignment5_navigator')

        self.declare_parameter('goals_file', '')
        self.declare_parameter('auto_initial_pose', False)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('auto_undock', True)
        self.declare_parameter('dock_status_timeout', 20.0)
        self.declare_parameter('goal_timeout', 240.0)

        # Navigation reliability parameters
        self.declare_parameter('goal_pause', 2.0)
        self.declare_parameter('plan_timeout', 15.0)
        self.declare_parameter('plan_retries', 3)
        self.declare_parameter('nav_retries', 1)
        self.declare_parameter('retry_delay', 3.0)
        self.declare_parameter('minimum_path_length', 0.10)

        self._goals = load_goals(
            self.get_parameter('goals_file').value
        )

        self._auto_pose = bool(
            self.get_parameter('auto_initial_pose').value
        )
        self._auto_undock = bool(
            self.get_parameter('auto_undock').value
        )
        self._dock_timeout = float(
            self.get_parameter('dock_status_timeout').value
        )
        self._goal_timeout = float(
            self.get_parameter('goal_timeout').value
        )
        self._goal_pause = float(
            self.get_parameter('goal_pause').value
        )
        self._plan_timeout = float(
            self.get_parameter('plan_timeout').value
        )
        self._plan_retries = int(
            self.get_parameter('plan_retries').value
        )
        self._nav_retries = int(
            self.get_parameter('nav_retries').value
        )
        self._retry_delay = float(
            self.get_parameter('retry_delay').value
        )
        self._min_path = float(
            self.get_parameter('minimum_path_length').value
        )

        if (
            self._goal_timeout <= 0
            or self._dock_timeout < 0
            or self._goal_pause < 0
            or self._plan_timeout <= 0
            or self._plan_retries < 1
            or self._nav_retries < 0
            or self._retry_delay < 0
            or self._min_path <= 0
        ):
            raise ValueError(
                'Invalid navigation timing or retry parameters'
            )

        self._pose_received = False
        self._initialpose_requested = False
        self._dock_status = None

        self._phase = 'waiting_localization'
        self._goal_index = 0
        self._goal_handle = None
        self._goal_start_time = None

        self._plan_started_at = None
        self._plan_token = 0
        self._plan_attempt = 0
        self._nav_retry = 0
        self._ready_after = 0.0

        self._started_at = time.monotonic()
        self._last_pose_pub = 0.0
        self._last_log = 0.0
        self._last_feedback = 0.0

        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self._on_amcl_pose,
            amcl_qos,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self._on_initialpose,
            10,
        )

        self.create_subscription(
            DockStatus,
            'dock_status',
            self._on_dock_status,
            qos_profile_sensor_data,
        )

        self._initial_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10,
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        self._plan_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose'
        )

        self._undock_client = ActionClient(
            self, Undock, 'undock'
        )

        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            'Loaded 3 goals: '
            + ' -> '.join(goal.name for goal in self._goals)
        )

        for name_a, name_b, distance in close_goal_pairs(self._goals):
            self.get_logger().warning(
                f'{name_a} and {name_b} are only '
                f'{distance:.2f} m apart. '
                'Nearby goals may lead to zero-length plans.'
            )

        if not self._auto_pose:
            self.get_logger().info(
                'Waiting for 2D Pose Estimate in RViz '
                'and subsequent /amcl_pose.'
            )

    # -------------------------------------------------------
    # Localization and docking
    # -------------------------------------------------------

    def _on_amcl_pose(self, _message):
        if self._initialpose_requested:
            self._pose_received = True

    def _on_initialpose(self, message):
        if message.header.frame_id != 'map':
            self.get_logger().warning(
                'Initial pose must be in the map frame.'
            )
            return

        self._initialpose_requested = True
        self._pose_received = False

        self.get_logger().info(
            'Initial pose received; awaiting AMCL update.'
        )

    def _on_dock_status(self, message):
        self._dock_status = bool(message.is_docked)

    def _log_wait(self, message):
        now = time.monotonic()

        if now - self._last_log >= 8.0:
            self.get_logger().info(message)
            self._last_log = now

    def _publish_initial_pose(self):
        message = PoseWithCovarianceStamped()

        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()

        message.pose.pose.position.x = float(
            self.get_parameter('initial_x').value
        )
        message.pose.pose.position.y = float(
            self.get_parameter('initial_y').value
        )

        yaw = float(
            self.get_parameter('initial_yaw').value
        )

        (
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ) = yaw_quaternion(yaw)

        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = math.radians(15.0) ** 2

        self._initial_pub.publish(message)

    # -------------------------------------------------------
    # Main state machine
    # -------------------------------------------------------

    def _tick(self):
        now = time.monotonic()

        if self._phase in (
            'done', 'failed', 'undocking', 'sending', 'canceling'
        ):
            return

        if self._phase == 'navigating':
            if (
                self._goal_start_time is not None
                and now - self._goal_start_time > self._goal_timeout
            ):
                self._phase = 'canceling'

                self.get_logger().error(
                    'Navigation goal timed out; requesting cancel.'
                )

                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async().add_done_callback(
                        self._on_cancel_response
                    )
            return

        if self._phase == 'planning':
            if (
                self._plan_started_at is not None
                and now - self._plan_started_at > self._plan_timeout
            ):
                self._plan_token += 1

                self._halt(
                    f'Planner did not respond within '
                    f'{self._plan_timeout:.1f}s. '
                    'Check /compute_path_to_pose and planner_server logs.'
                )
            return

        if not self._pose_received:
            if (
                self._auto_pose
                and now - self._last_pose_pub >= 2.0
            ):
                self._publish_initial_pose()
                self._last_pose_pub = now

            self._log_wait(
                'Waiting for /amcl_pose; '
                'check map, localization and initial pose.'
            )
            return

        if not self._nav_client.server_is_ready():
            self._log_wait(
                'Waiting for Nav2 /navigate_to_pose action server...'
            )
            return

        if not self._plan_client.server_is_ready():
            self._log_wait(
                'Waiting for Nav2 /compute_path_to_pose planner server...'
            )
            return

        if self._phase == 'waiting_localization':
            if self._auto_undock:

                if self._dock_status is None:
                    if now - self._started_at < self._dock_timeout:
                        self._log_wait('Waiting for /dock_status...')
                        return

                    self._halt(
                        'No /dock_status received. '
                        'Set auto_undock:=false only if '
                        'the robot is already undocked.'
                    )
                    return

                if self._dock_status:
                    if not self._undock_client.server_is_ready():
                        self._log_wait(
                            'Waiting for /undock action server...'
                        )
                        return

                    self._phase = 'undocking'

                    self.get_logger().info(
                        'Robot docked; sending Undock action.'
                    )

                    self._undock_client.send_goal_async(
                        Undock.Goal()
                    ).add_done_callback(
                        self._on_undock_accepted
                    )
                    return

            self._phase = 'ready'

        if self._phase == 'ready':
            if now < self._ready_after:
                return

            if self._goal_index >= len(self._goals):
                self._phase = 'done'

                self.get_logger().info(
                    'SUCCESS: all three navigation goals completed.'
                )
            else:
                self._check_next_plan()

    # -------------------------------------------------------
    # Undock callbacks
    # -------------------------------------------------------

    def _on_undock_accepted(self, future):
        try:
            handle = future.result()

            if handle is None or not handle.accepted:
                self._halt(
                    'Undock request rejected; navigation not started.'
                )
                return

            handle.get_result_async().add_done_callback(
                self._on_undock_result
            )

        except Exception as exc:
            self._halt(f'Undock failed: {exc}')

    def _on_undock_result(self, future):
        try:
            outcome = future.result()

            if outcome.status != GoalStatus.STATUS_SUCCEEDED:
                self._halt(
                    f'Undock failed with status {outcome.status}.'
                )
                return

            self.get_logger().info(
                'Undocked successfully; proceeding to Goal 1.'
            )

            self._phase = 'ready'
            self._ready_after = time.monotonic() + self._goal_pause

        except Exception as exc:
            self._halt(f'Undock response error: {exc}')

    # -------------------------------------------------------
    # Goal and path planning
    # -------------------------------------------------------

    def _goal_pose(self):
        goal = self._goals[self._goal_index]

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = goal.x
        pose.pose.position.y = goal.y

        (
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = yaw_quaternion(goal.yaw)

        return pose

    def _check_next_plan(self):
        """Check the global path before sending navigation."""

        self._plan_token += 1
        token = self._plan_token
        self._plan_attempt += 1

        request = ComputePathToPose.Goal()
        request.goal = self._goal_pose()
        request.use_start = False
        request.planner_id = ''

        self._phase = 'planning'
        self._plan_started_at = time.monotonic()

        goal = self._goals[self._goal_index]

        self.get_logger().info(
            f'Preflight Goal {self._goal_index + 1}/3 '
            f'({goal.name}), '
            f'plan attempt '
            f'{self._plan_attempt}/{self._plan_retries}, '
            f'nav retry {self._nav_retry}/{self._nav_retries}'
        )

        self._plan_client.send_goal_async(
            request
        ).add_done_callback(
            lambda future: self._on_plan_accepted(future, token)
        )

    def _on_plan_accepted(self, future, token):
        if (
            self._phase != 'planning'
            or token != self._plan_token
        ):
            return

        try:
            handle = future.result()

            if handle is None or not handle.accepted:
                self._plan_failed(
                    'planner rejected ComputePathToPose'
                )
                return

            handle.get_result_async().add_done_callback(
                lambda result_future: self._on_plan_result(
                    result_future, token
                )
            )

        except Exception as exc:
            self._plan_failed(f'planner request error: {exc}')

    def _on_plan_result(self, future, token):
        if (
            self._phase != 'planning'
            or token != self._plan_token
        ):
            return

        try:
            outcome = future.result()

            if outcome.status != GoalStatus.STATUS_SUCCEEDED:
                self._plan_failed(
                    f'planner action status {outcome.status} '
                    '(path unavailable)'
                )
                return

            valid, reason, _ = assess_nav2_path(
                outcome.result.path,
                min_travel=self._min_path,
            )

            if not valid:
                self._plan_failed(reason)
                return

            self.get_logger().info(
                f'Preflight OK: {reason}. Sending Nav2 goal.'
            )

            self._send_next_goal()

        except Exception as exc:
            self._plan_failed(f'planner result error: {exc}')

    def _plan_failed(self, reason):
        self.get_logger().warning(
            f'Goal {self._goal_index + 1} '
            f'cannot get a drivable plan: {reason}'
        )

        if self._plan_attempt < self._plan_retries:
            self._phase = 'ready'
            self._ready_after = (
                time.monotonic() + self._retry_delay
            )

            self.get_logger().info(
                f'Retrying global path planning after '
                f'{self._retry_delay:.1f}s.'
            )
        else:
            goal = self._goals[self._goal_index]

            self._halt(
                f'Goal {self._goal_index + 1} '
                f'({goal.name}) has no usable path after '
                f'{self._plan_retries} checks. '
                f'Goal x={goal.x:.3f}, y={goal.y:.3f}. '
                'Check clicked-point map coordinates, '
                'free cells / costmap, AMCL alignment, '
                'and spacing from the previous goal.'
            )

    # -------------------------------------------------------
    # NavigateToPose
    # -------------------------------------------------------

    def _send_next_goal(self):
        goal = self._goals[self._goal_index]

        request = NavigateToPose.Goal()
        request.pose = self._goal_pose()

        self._phase = 'sending'

        self.get_logger().info(
            f'Sending Goal {self._goal_index + 1}/3 '
            f'({goal.name}): '
            f'x={goal.x:.3f}, y={goal.y:.3f}, '
            f'yaw={goal.yaw:.3f} rad'
        )

        self._nav_client.send_goal_async(
            request,
            feedback_callback=self._on_feedback,
        ).add_done_callback(
            self._on_goal_accepted
        )

    def _on_goal_accepted(self, future):
        if self._phase != 'sending':
            return

        try:
            handle = future.result()

            if handle is None or not handle.accepted:
                self._retry_navigation(
                    'Nav2 rejected this goal'
                )
                return

            self._goal_handle = handle
            self._goal_start_time = time.monotonic()
            self._phase = 'navigating'

            handle.get_result_async().add_done_callback(
                self._on_goal_result
            )

        except Exception as exc:
            self._retry_navigation(
                f'Navigation send error: {exc}'
            )

    def _on_feedback(self, feedback_message):
        now = time.monotonic()

        if (
            self._phase != 'navigating'
            or now - self._last_feedback < 5.0
        ):
            return

        self._last_feedback = now

        remaining = (
            feedback_message.feedback.distance_remaining
        )

        self.get_logger().info(
            f'Goal {self._goal_index + 1}/3: '
            f'remaining {remaining:.2f} m'
        )

    def _on_goal_result(self, future):
        if self._phase in ('failed', 'done'):
            return

        if self._phase == 'canceling':
            self._halt(
                'Navigation timed out and was canceled; '
                'sequence stopped.'
            )
            return

        if self._phase != 'navigating':
            return

        try:
            outcome = future.result()

            if outcome.status != GoalStatus.STATUS_SUCCEEDED:
                self._retry_navigation(
                    f'Nav2 action status {outcome.status}'
                )
                return

            self.get_logger().info(
                f'Goal {self._goal_index + 1}/3 '
                'reached successfully.'
            )

            self._goal_index += 1
            self._goal_handle = None
            self._goal_start_time = None

            self._plan_attempt = 0
            self._nav_retry = 0
            self._phase = 'ready'

            self._ready_after = (
                time.monotonic() + self._goal_pause
            )

            if self._goal_index < len(self._goals):
                self.get_logger().info(
                    f'Waiting {self._goal_pause:.1f}s '
                    'for Nav2 to settle before '
                    'preparing the next goal.'
                )

        except Exception as exc:
            self._retry_navigation(
                f'Navigation result error: {exc}'
            )

    # -------------------------------------------------------
    # Recovery handling
    # -------------------------------------------------------

    def _retry_navigation(self, reason):
        self._goal_handle = None
        self._goal_start_time = None

        self.get_logger().warning(
            f'Goal {self._goal_index + 1} '
            f'navigation failed: {reason}'
        )

        if self._nav_retry >= self._nav_retries:
            self._halt(
                f'Goal {self._goal_index + 1} failed; '
                f'exhausted {self._nav_retries} '
                'additional navigation attempts. '
                'Inspect controller_server, '
                'planner_server and costmaps.'
            )
            return

        self._nav_retry += 1
        self._plan_attempt = 0
        self._phase = 'ready'

        self._ready_after = (
            time.monotonic() + self._retry_delay
        )

        self.get_logger().warning(
            f'Replanning Goal {self._goal_index + 1} '
            f'from current pose after '
            f'{self._retry_delay:.1f}s '
            f'(retry {self._nav_retry}/{self._nav_retries}).'
        )

    def _on_cancel_response(self, future):
        try:
            future.result()
        except Exception as exc:
            self._halt(
                f'Cancel request failed: {exc}'
            )

    def _halt(self, reason):
        self._phase = 'failed'
        self.get_logger().error(reason)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SequentialNavigator()
        rclpy.spin(node)

    except (ValueError, RuntimeError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f'ERROR: {exc}', flush=True)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
