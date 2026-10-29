from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)


# from moveit_configs_utils 
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, OpaqueFunction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch  
# from ament_index_python.packages import get_package_share_directory

# import os

# def launch_setup(context, *args, **kwargs):
#     # Load MoveIt config
#     moveit_config = MoveItConfigsBuilder(
#         "fairino5_v6_robot",
#         package_name="fairino5_v6_moveit2_config"
#     ).to_moveit_configs()

#     # Collect actions from generate_demo_launch
#     moveit_demo_actions = generate_demo_launch(moveit_config).entities

#     # Include easy_handeye2 calibrate launch
#     easy_handeye2_path = get_package_share_directory('easy_handeye2')
#     calibrate_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(easy_handeye2_path, 'launch', 'calibrate.launch.py')
#         ),
#         launch_arguments={
#             'calibration_type': 'eye_in_hand',
#             'name': 'fairino_calib',
#             'robot_base_frame': 'base_link',
#             'robot_effector_frame': 'wrist3_link',
#             'tracking_base_frame': 'camera_link',
#             'tracking_marker_frame': 'apriltag_0'
#         }.items()
#     )

#     return moveit_demo_actions + [calibrate_launch]


# def generate_launch_description():
#     return LaunchDescription([
#         OpaqueFunction(function=launch_setup)
#     ])
