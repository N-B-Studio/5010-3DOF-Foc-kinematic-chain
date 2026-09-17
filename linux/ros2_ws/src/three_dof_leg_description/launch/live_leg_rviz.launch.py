from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("three_dof_leg_description"))
    robot_description = (package_share / "urdf" / "three_dof_leg.urdf").read_text()
    rviz_config = package_share / "rviz" / "live_leg.rviz"

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config)],
            output="screen",
        ),
    ])
