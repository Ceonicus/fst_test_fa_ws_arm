import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    robotname = "fairino5_v6_robot"
    packagename = "fairino5_v6_moveit2_config"
    moveit_controllers_path = os.path.join(
        get_package_share_directory(packagename),
        "config",
        "moveit_controllers.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder(robotname, package_name=packagename)
        .robot_description(file_path="config/" + robotname + ".urdf.xacro")
        .trajectory_execution(file_path=moveit_controllers_path)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    planning_group = LaunchConfiguration("planning_group")
    j1 = LaunchConfiguration("j1")
    j2 = LaunchConfiguration("j2")
    j3 = LaunchConfiguration("j3")
    j4 = LaunchConfiguration("j4")
    j5 = LaunchConfiguration("j5")
    j6 = LaunchConfiguration("j6")
    max_joint_delta_rad = LaunchConfiguration("max_joint_delta_rad")
    already_at_target_tolerance_rad = LaunchConfiguration(
        "already_at_target_tolerance_rad"
    )
    planning_time_sec = LaunchConfiguration("planning_time_sec")
    velocity_scaling = LaunchConfiguration("velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")
    execute_motion = LaunchConfiguration("execute_motion")
    allow_reset_execution = LaunchConfiguration("allow_reset_execution")

    return_to_saved_joint_pose = Node(
        package="moveit_move",
        executable="return_to_saved_joint_pose",
        name="return_to_saved_joint_pose",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "planning_group": planning_group,
                "j1": ParameterValue(j1, value_type=float),
                "j2": ParameterValue(j2, value_type=float),
                "j3": ParameterValue(j3, value_type=float),
                "j4": ParameterValue(j4, value_type=float),
                "j5": ParameterValue(j5, value_type=float),
                "j6": ParameterValue(j6, value_type=float),
                "max_joint_delta_rad": ParameterValue(
                    max_joint_delta_rad, value_type=float
                ),
                "already_at_target_tolerance_rad": ParameterValue(
                    already_at_target_tolerance_rad, value_type=float
                ),
                "planning_time_sec": ParameterValue(planning_time_sec, value_type=float),
                "velocity_scaling": ParameterValue(velocity_scaling, value_type=float),
                "acceleration_scaling": ParameterValue(
                    acceleration_scaling, value_type=float
                ),
                "execute_motion": ParameterValue(execute_motion, value_type=bool),
                "allow_reset_execution": ParameterValue(
                    allow_reset_execution, value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("planning_group", default_value="fairino5_v6_group"),
            DeclareLaunchArgument("j1", default_value="-1.4155149936414828"),
            DeclareLaunchArgument("j2", default_value="-1.4576652436883306"),
            DeclareLaunchArgument("j3", default_value="-1.8781316413347113"),
            DeclareLaunchArgument("j4", default_value="0.2190782725278865"),
            DeclareLaunchArgument("j5", default_value="2.206984498534039"),
            DeclareLaunchArgument("j6", default_value="2.3956715922950256"),
            DeclareLaunchArgument("max_joint_delta_rad", default_value="0.50"),
            DeclareLaunchArgument(
                "already_at_target_tolerance_rad", default_value="0.002"
            ),
            DeclareLaunchArgument("planning_time_sec", default_value="5.0"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.002"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.002"),
            DeclareLaunchArgument("execute_motion", default_value="false"),
            DeclareLaunchArgument("allow_reset_execution", default_value="false"),
            return_to_saved_joint_pose,
        ]
    )
