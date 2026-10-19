"""One-entry launch for maze simulation, AMCL, Nav2, and YOLO (ROS Humble).

The original assignment-4 launch passed nav2/localization/map options to
`turtlebot4_ignition.launch.py`, which does not forward those arguments to the
robot spawn launch. Here localization and navigation are explicitly included.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from tb4_assignment5.parameter_tuning import build_wsl_parameter_files


PKG = 'tb4_assignment5'


def _argument(name, default, description):
    return DeclareLaunchArgument(name, default_value=str(default), description=description)


def _valid_map(yaml_path):
    """Ensure the YAML points to an EXISTING grid image (not just a metadata file)."""
    candidate = Path(yaml_path).expanduser()
    if not candidate.is_file():
        return False
    try:
        with candidate.open(encoding='utf-8') as stream:
            info = yaml.safe_load(stream)
        if not isinstance(info, dict) or not isinstance(info.get('image'), str):
            return False
        image_path = Path(info['image'])
        if not image_path.is_absolute():
            image_path = candidate.parent / image_path
        return image_path.is_file()
    except (OSError, yaml.YAMLError):
        return False


def _pick_map(requested):
    if requested != 'auto':
        if not _valid_map(requested):
            raise RuntimeError(
                f'Invalid map: {requested}. Verify map.yaml and its referenced map.pgm.')
        return str(Path(requested).expanduser().resolve())

    local = Path(get_package_share_directory(PKG)) / 'maps' / 'map.yaml'
    if _valid_map(local):
        return str(local)

    try:
        previous = Path(get_package_share_directory('tb4_nav2_assignment'))
        previous = previous / 'maps' / 'map.yaml'
        if _valid_map(previous):
            return str(previous)
    except Exception:
        pass

    raise RuntimeError(
        'Original assignment-4 map.pgm not found. Copy the real '
        'tb4_nav2_assignment/maps/map.pgm into tb4_assignment5/maps/ '
        'and rebuild; or source an existing tb4_nav2_assignment install; '
        'or pass map:=/absolute/path/map.yaml. Do not replace with fake data.')


def _launch_setup(context, *args, **kwargs):
    sim_share = Path(get_package_share_directory('turtlebot4_ignition_bringup'))
    nav_share = Path(get_package_share_directory('turtlebot4_navigation'))
    start_nav_stack = LaunchConfiguration('nav_stack').perform(context).lower() == 'true'
    map_file = (_pick_map(LaunchConfiguration('map').perform(context))
                if start_nav_stack else 'map managed by existing Nav2 stack')
    wsl_optimized = LaunchConfiguration('wsl_optimized').perform(context).lower() == 'true'
    if start_nav_stack and wsl_optimized:
        loc_params, nav_params = build_wsl_parameter_files(nav_share)
    else:
        loc_params, nav_params = None, None
    goals_file = LaunchConfiguration('goals_file').perform(context)
    if not Path(goals_file).expanduser().is_file():
        raise RuntimeError(f'Goals configuration not found: {goals_file}')

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            sim_share / 'launch' / 'ignition.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('sim')),
    )
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            sim_share / 'launch' / 'turtlebot4_spawn.launch.py')),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': 'true',
            'x': LaunchConfiguration('spawn_x'),
            'y': LaunchConfiguration('spawn_y'),
            'yaw': LaunchConfiguration('spawn_yaw'),
            'slam': 'false',
            'nav2': 'false',
            'localization': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    localization_args = {'map': map_file, 'use_sim_time': 'true'}
    if loc_params:
        localization_args['params'] = loc_params
    nav_args = {'use_sim_time': 'true'}
    if nav_params:
        nav_args['params_file'] = nav_params

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            nav_share / 'launch' / 'localization.launch.py')),
        launch_arguments=localization_args.items(),
        condition=IfCondition(LaunchConfiguration('nav_stack')),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            nav_share / 'launch' / 'nav2.launch.py')),
        launch_arguments=nav_args.items(),
        condition=IfCondition(LaunchConfiguration('nav_stack')),
    )

    navigator = Node(
        package=PKG,
        executable='navigate_goals',
        name='assignment5_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'goals_file': goals_file,
            'auto_initial_pose': ParameterValue(
                LaunchConfiguration('auto_initial_pose'), value_type=bool),
            'initial_x': ParameterValue(
                LaunchConfiguration('initial_x'), value_type=float),
            'initial_y': ParameterValue(
                LaunchConfiguration('initial_y'), value_type=float),
            'initial_yaw': ParameterValue(
                LaunchConfiguration('initial_yaw'), value_type=float),
            'auto_undock': ParameterValue(
                LaunchConfiguration('auto_undock'), value_type=bool),
            'goal_timeout': ParameterValue(
                LaunchConfiguration('goal_timeout'), value_type=float),
            'goal_pause': ParameterValue(
                LaunchConfiguration('goal_pause'), value_type=float),
            'plan_timeout': ParameterValue(
                LaunchConfiguration('plan_timeout'), value_type=float),
            'plan_retries': ParameterValue(
                LaunchConfiguration('plan_retries'), value_type=int),
            'nav_retries': ParameterValue(
                LaunchConfiguration('nav_retries'), value_type=int),
            'retry_delay': ParameterValue(
                LaunchConfiguration('retry_delay'), value_type=float),
            'minimum_path_length': ParameterValue(
                LaunchConfiguration('minimum_path_length'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('navigate')),
    )
    detector = Node(
        package=PKG,
        executable='detect_objects',
        name='assignment5_yolo',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': LaunchConfiguration('camera_topic'),
            'model': LaunchConfiguration('model_file'),
            'confidence': ParameterValue(
                LaunchConfiguration('confidence'), value_type=float),
            'max_fps': ParameterValue(
                LaunchConfiguration('max_fps'), value_type=float),
            'show_window': ParameterValue(
                LaunchConfiguration('show_window'), value_type=bool),
        }],
        condition=IfCondition(LaunchConfiguration('yolo')),
    )
    tf_health = Node(
        package=PKG,
        executable='check_tf_health',
        name='assignment5_tf_health',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('tf_health')),
    )
    point_logger = Node(
        package=PKG,
        executable='record_points',
        name='assignment5_point_logger',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('points')),
    )

    return [
        # Set before starting any child (AMCL, Nav2, simulator and bridge).
        # A fresh /tf in another node does not imply controller's DDS reader
        # is receiving it. Avoid WSL shared-memory reader stalls at transport level.
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            str(Path(get_package_share_directory(PKG)) / 'config' / 'fastdds_udp.xml'),
            condition=IfCondition(LaunchConfiguration('fastdds_udp')),
        ),
        LogInfo(
            msg='Fast DDS UDP-only profile enabled for launched processes',
            condition=IfCondition(LaunchConfiguration('fastdds_udp')),
        ),
        LogInfo(msg=['Loading maze occupancy map: ', map_file]),
        LogInfo(msg=['Reading navigation goals from: ', goals_file]),
        LogInfo(msg=['WSL lower-load Nav2 settings: ', str(wsl_optimized)]),
        ignition,
        robot_spawn,
        localization,
        nav2,
        TimerAction(period=5.0, actions=[navigator, detector, point_logger, tf_health]),
    ]


def generate_launch_description():
    own_share = Path(get_package_share_directory(PKG))
    args = [
        _argument('world', 'maze', 'Gazebo world used in assignment 4'),
        _argument('model', 'standard', 'TurtleBot4 model: standard or lite'),
        _argument('sim', 'true', 'Start Gazebo and TurtleBot4 robot'),
        _argument('nav_stack', 'true', 'Start AMCL, map server, and Nav2'),
        _argument('rviz', 'true', 'Start RViz visualization'),
        _argument('fastdds_udp', 'true',
                  'Use UDP-only Fast DDS transport to avoid WSL2 shared-memory TF stalls; '
                  'false preserves external DDS profiles'),
        _argument('wsl_optimized', 'false',
                  'Use lower-load, version-matched TurtleBot4 Nav2 YAML copies'),
        _argument('tf_health', 'false',
                  'Log scan/TF freshness while navigating'),
        _argument('map', 'auto', 'auto, or absolute path to map.yaml'),
        _argument('goals_file', str(own_share / 'config' / 'goals.yaml'),
                  'YAML containing three measured map-frame goals'),
        _argument('spawn_x', '0.0', 'Gazebo spawn x'),
        _argument('spawn_y', '0.0', 'Gazebo spawn y'),
        _argument('spawn_yaw', '0.0', 'Gazebo spawn heading (rad)'),
        _argument('navigate', 'true', 'Run 3-goal ROS 2 action node'),
        _argument('auto_initial_pose', 'false',
                  'Automatically publish configured AMCL initial pose'),
        _argument('initial_x', '0.0', 'Initial map pose x (if enabled)'),
        _argument('initial_y', '0.0', 'Initial map pose y (if enabled)'),
        _argument('initial_yaw', '0.0', 'Initial map heading in radians'),
        _argument('auto_undock', 'true', 'Undock robot before Nav2 commands'),
        _argument('goal_timeout', '240.0', 'Timeout in seconds per Nav2 goal'),
        _argument('goal_pause', '2.0', 'Pause between completed Nav2 goals (seconds)'),
        _argument('plan_timeout', '15.0', 'Timeout for path preflight (seconds)'),
        _argument('plan_retries', '3', 'Maximum preflight planning attempts per goal'),
        _argument('nav_retries', '1', 'Extra navigation retries on abort/rejection'),
        _argument('retry_delay', '3.0', 'Delay before retrying path/navigation'),
        _argument('minimum_path_length', '0.10', 'Minimum usable global path travel (m)'),
        _argument('yolo', 'true', 'Run live YOLO detection'),
        _argument('model_file', 'yolov8n.pt', 'YOLO model name or absolute .pt path'),
        _argument('camera_topic', '/oakd/rgb/preview/image_raw',
                  'TurtleBot4 RGB image topic'),
        _argument('confidence', '0.35', 'YOLO confidence threshold'),
        _argument('max_fps', '4.0', 'Maximum YOLO inference rate'),
        _argument('show_window', 'false',
                  'Show OpenCV window (otherwise use annotated ROS image topic)'),
        _argument('points', 'false', 'Print RViz clicked_point map coordinates'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
