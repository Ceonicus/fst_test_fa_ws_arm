import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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

    workspace_root = LaunchConfiguration("workspace_root")
    action_name = LaunchConfiguration("action_name")
    planning_group = LaunchConfiguration("planning_group")
    target_link = LaunchConfiguration("target_link")
    contact_pose_topic = LaunchConfiguration("contact_pose_topic")
    motion_config_file = LaunchConfiguration("motion_config_file")
    recorded_poses_file = LaunchConfiguration("recorded_poses_file")

    action_server = Node(
        package="moveit_move",
        executable="contact_box_cover_removal_action_server",
        name="contact_box_cover_removal_action_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "workspace_root": workspace_root,
                "action_name": action_name,
                "planning_group": planning_group,
                "target_link": target_link,
                "contact_pose_topic": contact_pose_topic,
                "motion_config_file": motion_config_file,
                "recorded_poses_file": recorded_poses_file,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("workspace_root", default_value=os.getcwd()),
            DeclareLaunchArgument("action_name", default_value="/contact_box_cover_removal"),
            DeclareLaunchArgument("planning_group", default_value="fairino5_v6_group"),
            DeclareLaunchArgument("target_link", default_value="wrist3_link"),
            DeclareLaunchArgument("contact_pose_topic", default_value="/contact_box_pose_debug"),
            DeclareLaunchArgument(
                "motion_config_file",
                default_value="config/contact_box_motion.env",
            ),
            DeclareLaunchArgument(
                "recorded_poses_file",
                default_value="config/recorded_robot_poses.json",
            ),
            action_server,
        ]
    )
