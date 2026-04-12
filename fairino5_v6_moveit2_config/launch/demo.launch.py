from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use mock hardware for simulation (true) or real robot (false)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.58.2",
            description="IP address of the robot (only used when use_fake_hardware:=false)",
        )
    )

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")

    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .robot_description(mappings={
            "use_fake_hardware": use_fake_hardware,
            "robot_ip": robot_ip,
        })
        .to_moveit_configs()
    )

    return LaunchDescription(declared_arguments + generate_demo_launch(moveit_config).entities)
