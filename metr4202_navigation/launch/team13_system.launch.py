"""Launch all Team 13 nodes except the external Nav2 stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the Team 13 application-node launch description."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    node_parameters = [{'use_sim_time': use_sim_time}]
    aruco_parameters = LaunchConfiguration('aruco_params_file')
    aruco_node_parameters = [
        aruco_parameters,
        {'use_sim_time': use_sim_time},
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the Gazebo simulation clock when true.',
        ),
        DeclareLaunchArgument(
            'aruco_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('metr4202_aruco'),
                'config',
                'aruco.yaml',
            ]),
            description='ArUco detector and localization parameters.',
        ),
        Node(
            package='metr4202_navigation',
            executable='exploration',
            name='exploration',
            output='screen',
            parameters=node_parameters,
        ),
        Node(
            package='metr4202_navigation',
            executable='coverage',
            name='coverage',
            output='screen',
            parameters=node_parameters,
        ),
        Node(
            package='metr4202_navigation',
            executable='recovery',
            name='recovery',
            output='screen',
            parameters=node_parameters,
        ),
        Node(
            package='metr4202_navigation',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=node_parameters,
        ),
        Node(
            package='metr4202_aruco',
            executable='aruco_detection',
            name='aruco_detection',
            output='screen',
            parameters=aruco_node_parameters,
        ),
        Node(
            package='metr4202_aruco',
            executable='aruco_localization',
            name='aruco_localization',
            output='screen',
            parameters=aruco_node_parameters,
        ),
    ])
