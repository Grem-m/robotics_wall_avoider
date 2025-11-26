# wall_avoider/launch/wall_avoider_launch.py
import os

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('wall_avoider')
    wall_avoider_params = os.path.join(package_dir, 'config', 'wall_avoider_params.yaml')
    world_file = os.path.join(package_dir, 'worlds', 'epuck_world.wbt')

    # Use the URDF and ros2_control configuration from the official e-puck package
    epuck_package_dir = get_package_share_directory('webots_ros2_epuck')
    robot_description_path = os.path.join(epuck_package_dir, 'resource', 'epuck_webots.urdf')
    ros2_control_params = os.path.join(epuck_package_dir, 'resource', 'ros2_control.yml')

    # Start Webots with the e-puck world
    webots = WebotsLauncher(
        world=world_file
    )

    # Webots driver for the e-puck robot, with proper robot_description and ros2_control params
    use_sim_time = True

    # Match the official e-puck launch: remap diffdrive command/odom topics to /cmd_vel and /odom
    use_twist_stamped = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] in ['kilted', 'rolling', 'jazzy']
    if use_twist_stamped:
        mappings = [
            ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
            ('/diffdrive_controller/odom', '/odom'),
        ]
    else:
        mappings = [
            ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/diffdrive_controller/odom', '/odom'),
        ]

    my_robot_driver = WebotsController(
        robot_name='e-puck',
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True,
            },
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=True,
    )

    # Spawn ROS 2 control controllers (diffdrive + joint state broadcaster)
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    # Wall-avoider behavior node, publishing TwistStamped to /cmd_vel
    wall_avoider_node = Node(
        package='wall_avoider',
        executable='wall_avoider_node',
        output='screen',
        parameters=[wall_avoider_params],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        diffdrive_controller_spawner,
        joint_state_broadcaster_spawner,
        wall_avoider_node,
        # Shut down ROS when Webots exits
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
