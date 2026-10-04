import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('point_extract')
    config = os.path.join(share, 'config', 'cluster_extract.yaml')

    return LaunchDescription([
        Node(
            package='point_extract',
            executable='cluster_extract_node',
            name='cluster_extract_node',
            output='screen',
            parameters=[config],
        ),
    ])
