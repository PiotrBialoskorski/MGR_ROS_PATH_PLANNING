import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():

    ld = LaunchDescription()

    map_server_config_path = os.path.join(
        get_package_share_directory('map_loader'),
        'launch',
        'map_server_params.yaml'
    )
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=['/home/roslab/ros2_mgr/maps/map_server_params.yaml']
    )

    lifecycle_nodes = ['map_server']
    use_sim_time = False
    autostart = True

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )
    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    return ld