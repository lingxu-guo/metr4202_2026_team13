"""Launch the complete Team 13 stack on a real TurtleBot3."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _rviz_preflight(context):
    """Fail early when RViz was requested without a graphical display."""
    requested = LaunchConfiguration('start_rviz').perform(context).lower()
    if requested not in ('1', 'true', 'yes', 'on'):
        return []
    display = os.environ.get('WAYLAND_DISPLAY') or os.environ.get('DISPLAY')
    if not display:
        raise RuntimeError(
            'RViz was requested, but DISPLAY and WAYLAND_DISPLAY are unset. '
            'Use a graphical session or start_rviz:=false.'
        )
    return [LogInfo(msg=f'RViz preflight passed; display={display}.')]


def generate_launch_description() -> LaunchDescription:
    """Create the hardware, online SLAM, Nav2, and application stack."""
    robot_model = LaunchConfiguration('robot_model')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    lds_model = LaunchConfiguration('lds_model')
    usb_port = LaunchConfiguration('usb_port')
    start_robot_bringup = LaunchConfiguration('start_robot_bringup')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    start_rviz = LaunchConfiguration('start_rviz')
    start_team13 = LaunchConfiguration('start_team13')
    rviz_start_delay = LaunchConfiguration('rviz_start_delay')
    rviz_software = LaunchConfiguration('rviz_software_rendering')
    controller_frequency = LaunchConfiguration('controller_frequency')
    planner_frequency = LaunchConfiguration('planner_frequency')
    inflation_radius = LaunchConfiguration('inflation_radius')
    navigation_params_file = LaunchConfiguration('navigation_params_file')
    aruco_params_file = LaunchConfiguration('aruco_params_file')

    turtlebot3_nav_params = PathJoinSubstitution([
        FindPackageShare('turtlebot3_navigation2'),
        'param',
        EnvironmentVariable('ROS_DISTRO'),
        [robot_model, '.yaml'],
    ])
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'controller_frequency': controller_frequency,
            'expected_planner_frequency': planner_frequency,
            'inflation_radius': inflation_radius,
        },
        convert_types=True,
    )
    nav2_rviz_config = PathJoinSubstitution([
        FindPackageShare('metr4202_bringup'),
        'rviz',
        'team13.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model',
            default_value=EnvironmentVariable(
                'TURTLEBOT3_MODEL', default_value='waffle_pi'
            ),
            description='TurtleBot3 model: burger, waffle, or waffle_pi.',
        ),
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value=EnvironmentVariable('ROS_DOMAIN_ID', default_value='0'),
            description='ROS 2 DDS domain shared by every launched node.',
        ),
        DeclareLaunchArgument(
            'lds_model',
            default_value=EnvironmentVariable(
                'LDS_MODEL', default_value='LDS-02'
            ),
            description='TurtleBot3 lidar model: LDS-01, LDS-02, or LDS-03.',
        ),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument(
            'start_robot_bringup',
            default_value='true',
            description='Start local TurtleBot3 hardware drivers.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file', default_value=turtlebot3_nav_params
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('metr4202_bringup'),
                'config',
                'slam_toolbox.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'navigation_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('metr4202_navigation'),
                'config',
                'navigation.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'aruco_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('metr4202_aruco'),
                'config',
                'aruco.yaml',
            ]),
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='True'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_team13', default_value='true'),
        DeclareLaunchArgument(
            'rviz_start_delay',
            default_value='5.0',
            description='Seconds to wait before RViz.',
        ),
        DeclareLaunchArgument(
            'rviz_software_rendering',
            default_value='false',
            description='Use Mesa software rendering for RViz.',
        ),
        DeclareLaunchArgument(
            'controller_frequency',
            default_value='8.0',
            description='Nav2 controller rate for resource-constrained hosts.',
        ),
        DeclareLaunchArgument(
            'planner_frequency',
            default_value='5.0',
            description='Nav2 planner rate for resource-constrained hosts.',
        ),
        DeclareLaunchArgument(
            'inflation_radius',
            default_value='0.55',
            description='Nav2 local and global costmap inflation radius.',
        ),
        OpaqueFunction(function=_rviz_preflight),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', robot_model),
        SetEnvironmentVariable('LDS_MODEL', lds_model),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_bringup'),
                    'launch',
                    'robot.launch.py',
                ])
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'usb_port': usb_port,
            }.items(),
            condition=IfCondition(start_robot_bringup),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_sync_launch.py',
                ])
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': slam_params_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py',
                ])
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': configured_nav2_params,
                'autostart': autostart,
                'use_composition': 'False',
                'use_respawn': use_respawn,
                'log_level': log_level,
            }.items(),
        ),
        TimerAction(
            period=rviz_start_delay,
            actions=[Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', nav2_rviz_config],
                parameters=[{'use_sim_time': False}],
                additional_env={
                    'LIBGL_ALWAYS_SOFTWARE': rviz_software,
                    'QT_X11_NO_MITSHM': '1',
                },
                output='screen',
                condition=IfCondition(start_rviz),
            )],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('metr4202_navigation'),
                    'launch',
                    'team13_system.launch.py',
                ])
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'navigation_params_file': navigation_params_file,
                'aruco_params_file': aruco_params_file,
            }.items(),
            condition=IfCondition(start_team13),
        ),
    ])
