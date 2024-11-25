from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'temperature_tracker'

    params_path = os.path.join(
        get_package_share_directory(package_name),
        'params',
        'temperature_tracker_parameters.yaml'
    )

    temperature_tracker_node = Node(
        package=package_name,
        executable='temperature_tracker',
        name='temperature_tracker',
        parameters=[params_path],
        output='screen',
    )

    return LaunchDescription([
        temperature_tracker_node
    ])