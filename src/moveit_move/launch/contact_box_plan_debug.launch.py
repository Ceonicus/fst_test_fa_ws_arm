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

    input_topic = LaunchConfiguration("input_topic")
    planning_group = LaunchConfiguration("planning_group")
    target_link = LaunchConfiguration("target_link")
    planning_mode = LaunchConfiguration("planning_mode")
    execute_motion = LaunchConfiguration("execute_motion", default="false")
    approach_offset_x = LaunchConfiguration("approach_offset_x")
    approach_offset_y = LaunchConfiguration("approach_offset_y")
    approach_offset_z = LaunchConfiguration("approach_offset_z")
    use_tool_offset = LaunchConfiguration("use_tool_offset")
    tool_offset_x = LaunchConfiguration("tool_offset_x")
    tool_offset_y = LaunchConfiguration("tool_offset_y")
    tool_offset_z = LaunchConfiguration("tool_offset_z")
    use_fixed_orientation = LaunchConfiguration("use_fixed_orientation")
    target_qx = LaunchConfiguration("target_qx")
    target_qy = LaunchConfiguration("target_qy")
    target_qz = LaunchConfiguration("target_qz")
    target_qw = LaunchConfiguration("target_qw")
    plan_once = LaunchConfiguration("plan_once")
    use_tool_standoff = LaunchConfiguration("use_tool_standoff")
    standoff_distance = LaunchConfiguration("standoff_distance")
    allow_standoff_execution = LaunchConfiguration("allow_standoff_execution")
    velocity_scaling = LaunchConfiguration("velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")
    max_wrist_target_distance = LaunchConfiguration("max_wrist_target_distance")
    cartesian_eef_step = LaunchConfiguration("cartesian_eef_step")
    cartesian_jump_threshold = LaunchConfiguration("cartesian_jump_threshold")
    cartesian_min_fraction = LaunchConfiguration("cartesian_min_fraction")
    max_joint_delta_rad = LaunchConfiguration("max_joint_delta_rad")
    lock_contact_pose = LaunchConfiguration("lock_contact_pose")
    contact_pose_sample_count = LaunchConfiguration("contact_pose_sample_count")
    contact_pose_sample_timeout_sec = LaunchConfiguration("contact_pose_sample_timeout_sec")
    contact_pose_max_spread_m = LaunchConfiguration("contact_pose_max_spread_m")
    locked_contact_pose_source = LaunchConfiguration("locked_contact_pose_source")
    locked_contact_x = LaunchConfiguration("locked_contact_x")
    locked_contact_y = LaunchConfiguration("locked_contact_y")
    locked_contact_z = LaunchConfiguration("locked_contact_z")
    exit_after_completion = LaunchConfiguration("exit_after_completion", default="true")

    contact_box_plan_debug = Node(
        package="moveit_move",
        executable="contact_box_plan_debug",
        name="contact_box_plan_debug",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "input_topic": input_topic,
                "planning_group": planning_group,
                "target_link": target_link,
                "planning_mode": planning_mode,
                "execute_motion": ParameterValue(execute_motion, value_type=bool),
                "approach_offset_x": ParameterValue(approach_offset_x, value_type=float),
                "approach_offset_y": ParameterValue(approach_offset_y, value_type=float),
                "approach_offset_z": ParameterValue(approach_offset_z, value_type=float),
                "use_tool_offset": ParameterValue(use_tool_offset, value_type=bool),
                "tool_offset_x": ParameterValue(tool_offset_x, value_type=float),
                "tool_offset_y": ParameterValue(tool_offset_y, value_type=float),
                "tool_offset_z": ParameterValue(tool_offset_z, value_type=float),
                "use_fixed_orientation": ParameterValue(use_fixed_orientation, value_type=bool),
                "target_qx": ParameterValue(target_qx, value_type=float),
                "target_qy": ParameterValue(target_qy, value_type=float),
                "target_qz": ParameterValue(target_qz, value_type=float),
                "target_qw": ParameterValue(target_qw, value_type=float),
                "plan_once": ParameterValue(plan_once, value_type=bool),
                "use_tool_standoff": ParameterValue(use_tool_standoff, value_type=bool),
                "standoff_distance": ParameterValue(standoff_distance, value_type=float),
                "allow_standoff_execution": ParameterValue(allow_standoff_execution, value_type=bool),
                "velocity_scaling": ParameterValue(velocity_scaling, value_type=float),
                "acceleration_scaling": ParameterValue(acceleration_scaling, value_type=float),
                "max_wrist_target_distance": ParameterValue(
                    max_wrist_target_distance, value_type=float
                ),
                "cartesian_eef_step": ParameterValue(cartesian_eef_step, value_type=float),
                "cartesian_jump_threshold": ParameterValue(
                    cartesian_jump_threshold, value_type=float
                ),
                "cartesian_min_fraction": ParameterValue(
                    cartesian_min_fraction, value_type=float
                ),
                "max_joint_delta_rad": ParameterValue(max_joint_delta_rad, value_type=float),
                "lock_contact_pose": ParameterValue(lock_contact_pose, value_type=bool),
                "contact_pose_sample_count": ParameterValue(
                    contact_pose_sample_count, value_type=int
                ),
                "contact_pose_sample_timeout_sec": ParameterValue(
                    contact_pose_sample_timeout_sec, value_type=float
                ),
                "contact_pose_max_spread_m": ParameterValue(
                    contact_pose_max_spread_m, value_type=float
                ),
                "locked_contact_pose_source": locked_contact_pose_source,
                "locked_contact_x": ParameterValue(locked_contact_x, value_type=float),
                "locked_contact_y": ParameterValue(locked_contact_y, value_type=float),
                "locked_contact_z": ParameterValue(locked_contact_z, value_type=float),
                "exit_after_completion": ParameterValue(
                    exit_after_completion, value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/contact_box_pose_debug"),
            DeclareLaunchArgument("planning_group", default_value="fairino5_v6_group"),
            DeclareLaunchArgument("target_link", default_value="wrist3_link"),
            DeclareLaunchArgument("planning_mode", default_value="pose_target"),
            DeclareLaunchArgument("execute_motion", default_value="false"),
            DeclareLaunchArgument("approach_offset_x", default_value="0.0"),
            DeclareLaunchArgument("approach_offset_y", default_value="0.0"),
            DeclareLaunchArgument("approach_offset_z", default_value="0.0"),
            DeclareLaunchArgument("use_tool_offset", default_value="true"),
            DeclareLaunchArgument("tool_offset_x", default_value="0.0"),
            DeclareLaunchArgument("tool_offset_y", default_value="0.0"),
            DeclareLaunchArgument("tool_offset_z", default_value="0.263"),
            DeclareLaunchArgument("use_fixed_orientation", default_value="false"),
            DeclareLaunchArgument("target_qx", default_value="0.0"),
            DeclareLaunchArgument("target_qy", default_value="0.0"),
            DeclareLaunchArgument("target_qz", default_value="0.0"),
            DeclareLaunchArgument("target_qw", default_value="1.0"),
            DeclareLaunchArgument("plan_once", default_value="true"),
            DeclareLaunchArgument("use_tool_standoff", default_value="true"),
            DeclareLaunchArgument("standoff_distance", default_value="0.05"),
            DeclareLaunchArgument("allow_standoff_execution", default_value="false"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.001"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.001"),
            DeclareLaunchArgument("max_wrist_target_distance", default_value="0.05"),
            DeclareLaunchArgument("cartesian_eef_step", default_value="0.001"),
            DeclareLaunchArgument("cartesian_jump_threshold", default_value="0.0"),
            DeclareLaunchArgument("cartesian_min_fraction", default_value="0.99"),
            DeclareLaunchArgument("max_joint_delta_rad", default_value="0.08"),
            DeclareLaunchArgument("lock_contact_pose", default_value="true"),
            DeclareLaunchArgument("contact_pose_sample_count", default_value="5"),
            DeclareLaunchArgument("contact_pose_sample_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("contact_pose_max_spread_m", default_value="0.015"),
            DeclareLaunchArgument("locked_contact_pose_source", default_value="live_average"),
            DeclareLaunchArgument("locked_contact_x", default_value="0.0"),
            DeclareLaunchArgument("locked_contact_y", default_value="0.0"),
            DeclareLaunchArgument("locked_contact_z", default_value="0.0"),
            DeclareLaunchArgument("exit_after_completion", default_value="true"),
            contact_box_plan_debug,
        ]
    )
