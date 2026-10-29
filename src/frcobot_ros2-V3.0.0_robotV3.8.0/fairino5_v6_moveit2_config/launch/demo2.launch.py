import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
import xacro
def generate_launch_description():
    # Define package directory

    rviz_config_path = os.path.join(
        get_package_share_directory("fairino_description"),
        "config",
        "urdf.rviz"
    )
    xacro_file = os.path.join(
        get_package_share_directory('fairino_description'), 'urdf', 'fairino_camera_test.urdf.xacro'
    )
    
    
    doc = xacro.process_file(xacro_file)
    robot_description_xml = doc.toxml()
    robot_description = {"robot_description": robot_description_xml}

    # Nodes
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_xml}]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # arguments=["-d", rviz_config_path]
    )

    return LaunchDescription([

        joint_state_publisher_gui,
        robot_state_publisher,
        rviz
    ])

















# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
# from launch.substitutions import Command, LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# import os

# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():
#     # Set the path to the URDF file
#     urdf_file_name = 'fairino_camera_test.urdf'
#     urdf_path = os.path.join(
#         get_package_share_directory('fairino_description'),
#         'urdf',
#         urdf_file_name)

#     # Load the robot_description parameter with the URDF contents
#     with open(urdf_path, 'r') as inf:
#         robot_description_content = inf.read()

#     return LaunchDescription([
#         # Robot State Publisher
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[{'robot_description': robot_description_content}]
#         ),

#         # RViz2
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             arguments=['-d', os.path.join(
#                 get_package_share_directory('fairino_description'), 'rviz', 'robot.rviz')]
#         ),
#     ])



















# import launch  # Ensure launch is imported
# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
# from launch_ros.substitutions import FindPackageShare
# from launch.conditions import IfCondition

# def generate_launch_description():
#     # Argument to control publishing of handeye calibration
#     publish_handeye_arg = DeclareLaunchArgument( 'publish_handeye_calibration', default_value='true',  # By default, publish when demo.launch.py is run standalone
#                                                description='Whether to launch easy_handeye publish.launch.py')
#     # Argument for the calibration name, to be passed to publish.launch.py
#     calibration_name_arg = DeclareLaunchArgument( 'calibration_name', default_value='fairino_realsense_calibration',  # Default name, ensure it matches what you save
#                                                 description='Name of the hand-eye calibration to publish.')

#     moveit_config = MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config").to_moveit_configs()

#     # Renamed for clarity and consistency with previous suggestions
#     easy_handeye_publish_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource( [PathJoinSubstitution([FindPackageShare("easy_handeye2"), "launch", "publish.launch.py"])]),  # Use LaunchConfiguration to pass the calibration_name argument
#         launch_arguments={'calibration_name': LaunchConfiguration('calibration_name')}.items(),  # Conditionally launch based on the 'publish_handeye_calibration' argument
#         condition=IfCondition(LaunchConfiguration('publish_handeye_calibration')))

#     # Get the base LaunchDescription from generate_demo_launch
#     ld = generate_demo_launch(moveit_config)
#     # Add the declared arguments and the easy_handeye publish launch action
#     ld.add_action(publish_handeye_arg)
#     ld.add_action(calibration_name_arg)
#     ld.add_action(easy_handeye_publish_launch)
#     return ld
