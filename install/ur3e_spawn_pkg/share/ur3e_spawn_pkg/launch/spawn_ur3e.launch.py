from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ur3e_xacro_path = os.path.join(get_package_share_directory('ur3e_spawn_pkg'), 'ur3e.xacro')

    return LaunchDescription([
        Node(
            package='xacro',
            executable='xacro',
            name='ur3e_xacro',
            output='screen',
            arguments=[ur3e_xacro_path]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ur3e_xacro_path}]
        ),
        # Add more nodes for other functionalities you need, like controllers, visualization, etc.
    ])
