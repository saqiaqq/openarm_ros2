import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _generate_robot_description(
    context: LaunchContext,
    description_package: LaunchConfiguration,
    description_file: LaunchConfiguration,
    arm_type: LaunchConfiguration,
    use_fake_hardware: LaunchConfiguration,
    use_sim_hardware: LaunchConfiguration,
    right_can_interface: LaunchConfiguration,
    left_can_interface: LaunchConfiguration,
):
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    use_sim_hardware_str = context.perform_substitution(use_sim_hardware)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    left_can_interface_str = context.perform_substitution(left_can_interface)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf",
        "robot",
        description_file_str,
    )

    return xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "use_sim_hardware": use_sim_hardware_str,
            "ros2_control": "true",
            "left_can_interface": left_can_interface_str,
            "right_can_interface": right_can_interface_str,
        },
    ).toprettyxml(indent="  ")


def _rviz_only(context: LaunchContext, *args, **kwargs):
    # Important:
    # - Do NOT start robot_state_publisher locally (avoid /tf duplication).
    # - Only inject robot_description (and related MoveIt params) into RViz.
    robot_description = _generate_robot_description(
        context,
        LaunchConfiguration("description_package"),
        LaunchConfiguration("description_file"),
        LaunchConfiguration("arm_type"),
        LaunchConfiguration("use_fake_hardware"),
        LaunchConfiguration("use_sim_hardware"),
        LaunchConfiguration("right_can_interface"),
        LaunchConfiguration("left_can_interface"),
    )

    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config"
    ).to_moveit_configs()
    moveit_params = moveit_config.to_dict()

    moveit_cfg_share = get_package_share_directory("openarm_bimanual_moveit_config")
    rviz_cfg = os.path.join(moveit_cfg_share, "config", "moveit.rviz")

    # MotionPlanning display in RViz reads these params from the RViz node.
    # The actual planning scene/services/actions come from the remote move_group.
    rviz_params = [
        {"use_sim_time": context.perform_substitution(LaunchConfiguration("use_sim_time")) == "true"},
        moveit_params,
        # Ensure RViz loads the exact same URDF as the board launch arguments.
        {"robot_description": robot_description},
    ]

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_cfg],
            parameters=rviz_params,
        )
    ]


def generate_launch_description():
    # Defaults are kept identical to the development board launch so RViz loads
    # the same URDF. This file is intended for the laptop side only.
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("description_package", default_value="openarm_description"),
        DeclareLaunchArgument("description_file", default_value="v10.urdf.xacro"),
        DeclareLaunchArgument("arm_type", default_value="v10"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("use_sim_hardware", default_value="false"),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("left_can_interface", default_value="can1"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_rviz_only)])
