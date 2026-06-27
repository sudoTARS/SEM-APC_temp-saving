from launch import LaunchDescription
from launch_ros.actions import Node

import os

import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription([
        # Declare launch arguments (all have defaults)
        launch.actions.DeclareLaunchArgument('host',    default_value='localhost'),
        launch.actions.DeclareLaunchArgument('port',    default_value='2000'),
        launch.actions.DeclareLaunchArgument('timeout', default_value='10'),
        launch.actions.DeclareLaunchArgument('role_name', default_value='ego_vehicle'),
        launch.actions.DeclareLaunchArgument('vehicle_filter', default_value='vehicle.*'),
        launch.actions.DeclareLaunchArgument('spawn_point', default_value='None'),
        launch.actions.DeclareLaunchArgument('town', default_value='Town01'),
        launch.actions.DeclareLaunchArgument('passive', default_value='False'),
        launch.actions.DeclareLaunchArgument('synchronous_mode_wait_for_vehicle_control_command', default_value='False'),
        launch.actions.DeclareLaunchArgument('fixed_delta_seconds', default_value='0.05'),

        # Include the CARLA ROS bridge launch
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('carla_ros_bridge'),
                    'carla_ros_bridge.launch.py'
                )
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'autopilot': launch.substitutions.LaunchConfiguration('autopilot'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds'),
                'spawn_point': launch.substitutions.LaunchConfiguration('spawn_point'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'vehicle_filter': launch.substitutions.LaunchConfiguration('vehicle_filter'),
                'enable_example_control': 'True'
            }.items()
        ),
    ])

    # Add the example_control node
    example_control = Node(
        package='shell_simulation',
        namespace='',
        executable='example_control',
        name='example_control',
        output='screen'
    )
    ld.add_action(example_control)
    return ld

