import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("traj_cover")
    config = os.path.join(share, "config", "cover.yaml")
    return LaunchDescription(
        [
            Node(
                package="traj_cover",
                executable="cover_plan_node",
                name="cover_plan_node",
                output="screen",
                parameters=[config],
            )
        ]
    )
