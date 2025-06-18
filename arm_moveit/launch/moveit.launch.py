from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
    )

    is_sim = LaunchConfiguration("is_sim")

    moveit_configs = MoveItConfigsBuilder(
        "roboarm", package_name="arm_moveit").robot_description(
            robot_description_semantic_param_name="robot_description_semantic",
            robot_description_kinematics_param_name="robot_description_kinematics",
            robot_description_planning_param_name="robot_description_planning"
        )

    return LaunchDescription()