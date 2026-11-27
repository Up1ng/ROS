"""Start the controller after the upstream simulator is running."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default = PathJoinSubstitution([FindPackageShare('lane_controller'), 'config', 'lane.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        Node(package='lane_controller', executable='lane_controller', output='screen',
             parameters=[LaunchConfiguration('config'), {'use_sim_time': True}]),
    ])
