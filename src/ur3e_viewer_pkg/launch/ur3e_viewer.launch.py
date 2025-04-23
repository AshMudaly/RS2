from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_description_pkg = FindPackageShare("ur_description").find("ur_description")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ur_description_pkg, "/launch/view_ur.launch.py"]),
            launch_arguments={"ur_type": "ur3e"}.items(),
        )
    ])
