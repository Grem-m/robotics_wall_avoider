# wall_avoider/launch/wall_avoider_launch.py
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    
    package_dir = get_package_share_directory('wall_avoider')
    wall_avoider_params = os.path.join(package_dir, 'config', 'wall_avoider_params.yaml')
    world_file = os.path.join(package_dir, 'worlds', 'epuck_world.wbt')
    robot_description_path = os.path.join(package_dir, 'resource', 'epuck_webots.urdf')

    # Your Webots launch command would go here
    webots_process = WebotsLauncher(
        world=world_file
    )

    my_robot_driver = WebotsController(
        robot_name='my_epuck',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    
    wall_avoider_node = Node(
        package='wall_avoider',
        executable='wall_avoider_node',
        output='screen',
        parameters=[wall_avoider_params],
        arguments=['--ros-args', '--log-level', 'info'],
    )
    
    return LaunchDescription([
        webots_process,
        my_robot_driver,
        wall_avoider_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots_process,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])