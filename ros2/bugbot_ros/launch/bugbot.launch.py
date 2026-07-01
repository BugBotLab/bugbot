"""Launch the BugBot learned controller.

    ros2 launch bugbot_ros bugbot.launch.py policy_path:=/path/to/policy.pt

With no policy_path it runs a zero policy (robot holds still) -- safe for bring-up.
Run the direct teleop bridge separately with: ros2 run bugbot_ros twist_bridge
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    policy_path = LaunchConfiguration("policy_path")
    control_hz = LaunchConfiguration("control_hz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("policy_path", default_value="", description="TorchScript policy (empty = zero policy)"),
            DeclareLaunchArgument("control_hz", default_value="50.0", description="Control loop rate (Hz)"),
            Node(
                package="bugbot_ros",
                executable="policy_node",
                name="bugbot_policy",
                output="screen",
                parameters=[{"policy_path": policy_path, "control_hz": control_hz}],
            ),
        ]
    )
