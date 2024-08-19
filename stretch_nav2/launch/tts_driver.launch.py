import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stretch_navigation_path = get_package_share_directory('stretch_nav2')

    tts_node_launch = Node(
            package='stretch_nav2',
            executable='tts_driver_code.py',
            name='tts_node',
            output='screen'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', 'src/stretch_ros2/stretch_nav2/stretch_nav2/tts_driver_code.py'],
            output = 'screen'
        ),
        # tts_node_launch,
    ])