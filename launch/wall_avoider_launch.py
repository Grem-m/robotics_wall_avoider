# wall_avoider/launch/wall_avoider_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    
    package_dir = get_package_share_directory('wall_avoider')
    wall_avoider_params = os.path.join(package_dir, 'config', 'wall_avoider_params.yaml')
    world_file = os.path.join(package_dir, 'worlds', 'epuck_world.wbt')

    # Your Webots launch command would go here
    webots_process = ExecuteProcess(
        cmd=['webots', '--mode=realtime', world_file],
        output='screen',
        shell=True
    )
    
    wall_avoider_node = Node(
        package='wall_avoider',
        executable='wall_avoider_node',
        output='screen',
        parameters=[wall_avoider_params]
    )
    
    return LaunchDescription([
        webots_process,
        wall_avoider_node
    ])