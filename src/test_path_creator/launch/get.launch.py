from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'test_path_creator',
            name = 'Get_initial',
            executable = 'GetInitial',
            ),
        Node(
            package = 'test_path_creator',
            name = 'Get_goal',
            executable = 'GetGoal',
        )
    ])