"""Launch the complete Team 13 stack in TurtleBot3 Gazebo simulation."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.actions import SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _gui_preflight(context):
    """Fail early with a useful message when no graphical display exists."""
    gui_requested = any(
        LaunchConfiguration(name).perform(context).lower()
        in ('1', 'true', 'yes', 'on')
        for name in ('start_gazebo_gui', 'start_rviz')
    )
    if not gui_requested:
        return []

    display = os.environ.get('WAYLAND_DISPLAY') or os.environ.get('DISPLAY')
    if not display:
        raise RuntimeError(
            'A GUI was requested, but DISPLAY and WAYLAND_DISPLAY are unset. '
            'Start from a graphical session or disable both GUIs.'
        )

    return [LogInfo(msg=f'GUI preflight passed; display={display}.')]


def generate_launch_description() -> LaunchDescription:
    """Create the simulation, online SLAM, Nav2, and application stack."""
    robot_model = LaunchConfiguration('robot_model')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    start_rviz = LaunchConfiguration('start_rviz')
    start_gazebo_gui = LaunchConfiguration('start_gazebo_gui')
    start_team13 = LaunchConfiguration('start_team13')
    gazebo_gui_delay = LaunchConfiguration('gazebo_gui_delay')
    rviz_start_delay = LaunchConfiguration('rviz_start_delay')
    navigation_start_delay = LaunchConfiguration('navigation_start_delay')
    team13_start_delay = LaunchConfiguration('team13_start_delay')
    gazebo_software = LaunchConfiguration('gazebo_gui_software_rendering')
    rviz_software = LaunchConfiguration('rviz_software_rendering')
    controller_frequency = LaunchConfiguration('controller_frequency')
    planner_frequency = LaunchConfiguration('planner_frequency')
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
        },
        convert_types=True,
    )
    nav2_rviz_config = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz',
    ])
    turtlebot3_gazebo_share = FindPackageShare('turtlebot3_gazebo')
    gazebo_ros_share = FindPackageShare('gazebo_ros')
    default_gazebo_world = PathJoinSubstitution([
        turtlebot3_gazebo_share, 'worlds', 'turtlebot3_world.world'
    ])
    gazebo_models = PathJoinSubstitution([
        turtlebot3_gazebo_share, 'models'
    ])
    turtlebot3_gazebo_launch = PathJoinSubstitution([
        turtlebot3_gazebo_share, 'launch'
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
            'world',
            default_value=default_gazebo_world,
            description='Absolute path to the Gazebo Classic .world file.',
        ),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
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
        DeclareLaunchArgument('use_respawn', default_value='False'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_gazebo_gui', default_value='true'),
        DeclareLaunchArgument('start_team13', default_value='true'),
        DeclareLaunchArgument(
            'gazebo_gui_delay',
            default_value='3.0',
            description='Seconds to wait for gzserver before gzclient.',
        ),
        DeclareLaunchArgument(
            'rviz_start_delay',
            default_value='7.0',
            description='Seconds to wait before RViz.',
        ),
        DeclareLaunchArgument(
            'navigation_start_delay',
            default_value='6.0',
            description='Seconds to wait for robot sensors before SLAM/Nav2.',
        ),
        DeclareLaunchArgument(
            'team13_start_delay',
            default_value='10.0',
            description='Seconds to wait before Team 13 mission nodes.',
        ),
        DeclareLaunchArgument(
            'gazebo_gui_software_rendering',
            default_value='false',
            description='Use Mesa software rendering for gzclient.',
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
        OpaqueFunction(function=_gui_preflight),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', robot_model),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            [
                gazebo_models,
                ':',
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    gazebo_ros_share, 'launch', 'gzserver.launch.py'
                ])
            ),
            launch_arguments={'world': world}.items(),
        ),
        TimerAction(
            period=gazebo_gui_delay,
            actions=[ExecuteProcess(
                cmd=['gzclient', '--verbose'],
                output='screen',
                additional_env={
                    'LIBGL_ALWAYS_SOFTWARE': gazebo_software,
                    'QT_X11_NO_MITSHM': '1',
                },
                condition=IfCondition(start_gazebo_gui),
            )],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    turtlebot3_gazebo_launch,
                    'robot_state_publisher.launch.py',
                ])
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    turtlebot3_gazebo_launch,
                    'spawn_turtlebot3.launch.py',
                ])
            ),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        ),
        TimerAction(
            period=navigation_start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('slam_toolbox'),
                            'launch',
                            'online_sync_launch.py',
                        ])
                    ),
                    launch_arguments={
                        'use_sim_time': 'true',
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
                        'use_sim_time': 'true',
                        'params_file': configured_nav2_params,
                        'autostart': autostart,
                        'use_composition': 'False',
                        'use_respawn': use_respawn,
                        'log_level': log_level,
                    }.items(),
                ),
            ],
        ),
        TimerAction(
            period=rviz_start_delay,
            actions=[Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', nav2_rviz_config],
                parameters=[{'use_sim_time': True}],
                additional_env={
                    'LIBGL_ALWAYS_SOFTWARE': rviz_software,
                    'QT_X11_NO_MITSHM': '1',
                },
                output='screen',
                condition=IfCondition(start_rviz),
            )],
        ),
        TimerAction(
            period=team13_start_delay,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('metr4202_navigation'),
                        'launch',
                        'team13_system.launch.py',
                    ])
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'navigation_params_file': navigation_params_file,
                    'aruco_params_file': aruco_params_file,
                }.items(),
                condition=IfCondition(start_team13),
            )],
        ),
    ])
