from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for connecting to real Fairino robot with MoveIt2.
    Uses FairinoHardwareInterface for real robot control.
    """
    # Build MoveIt config with real robot URDF (use_fake_hardware=false)
    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .robot_description(
            file_path="config/fairino5_v6_robot.urdf.xacro",
            mappings={"use_fake_hardware": "false"}
        )
        .to_moveit_configs()
    )

    return generate_demo_launch(moveit_config)
