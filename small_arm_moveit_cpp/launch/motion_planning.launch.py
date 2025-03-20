import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="small_arm", package_name="small_arm_moveit_config"
        )
        .robot_description(file_path="config/small_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/small_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("small_arm_moveit_cpp")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )

    cpp_node = DeclareLaunchArgument(
        "cpp_node",
        default_value="joint_goal",
        description="C++ API file name",
    )


    moveit_cpp_node = Node(
        name="moveit_cpp",
        package="small_arm_moveit_cpp",
        executable=LaunchConfiguration("cpp_node"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )




    return LaunchDescription(
        [
            cpp_node,
            moveit_cpp_node,
        ]
    )