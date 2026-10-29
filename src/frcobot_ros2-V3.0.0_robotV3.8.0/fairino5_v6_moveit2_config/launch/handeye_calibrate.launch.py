import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Before you begin, ensure you have the necessary packages installed:
    # sudo apt install ros-$ROS_DISTRO-easy-handeye2
    # sudo apt install ros-$ROS_DISTRO-rqt-easy-handeye
    # sudo apt install ros-$ROS_DISTRO-apriltag-ros
    
    # Declare arguments that can be set when launching this file
    calibration_name_arg = DeclareLaunchArgument(
        'calibration_name',
        default_value='fairino_realsense_calibration', # A more descriptive default name
        description='Name of the calibration to perform and save.'
    )
    calibration_type_arg = DeclareLaunchArgument(
        'calibration_type',
        default_value='eye_in_hand', # Or 'eye_on_base'
        choices=['eye_in_hand', 'eye_on_base'],
        description='Type of hand-eye calibration.'
    )
    robot_base_frame_arg = DeclareLaunchArgument(
        'robot_base_frame',
        default_value='base_link', # IMPORTANT: Verify this for your Fairino robot
        description='Base frame of the robot (e.g., base_link, world).'
    )
    robot_effector_frame_arg = DeclareLaunchArgument(
        'robot_effector_frame',
        default_value='wrist3_link', # IMPORTANT: Verify this for your Fairino robot's end-effector
        description='End-effector frame of the robot (e.g., tool0, ee_link).'
    )
    tracking_base_frame_arg = DeclareLaunchArgument(
        'tracking_base_frame',
        # For RealSense, this is typically the frame of the color camera.
        default_value='camera_color_optical_frame',
        description='Base frame of the tracking system (e.g., camera_link, camera_color_optical_frame).'
    )
    tracking_marker_frame_arg = DeclareLaunchArgument(
        'tracking_marker_frame',
        # This frame is published by the marker detection node (e.g., apriltag_ros).
        # It should match the configuration in your marker detection setup.
        default_value='tag36h11_0', # Example for apriltag_ros detecting a tag of family 36h11 with id 0
        description='Frame of the detected marker.'
    )

    # 1. Launch the robot description and MoveIt
    # Note: Ensure your demo.launch.py accepts 'publish_handeye_calibration'.
    # This is important to prevent it from publishing an old calibration.
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("fairino5_v6_moveit2_config"), "launch", "demo.launch.py"])]
        ),
        launch_arguments={
            'publish_handeye_calibration': 'false',
        }.items()
    )

    # 2. Launch the camera driver (e.g., RealSense)
    tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])]
        ),
        launch_arguments={
                           'align_depth.enable': 'true',
                           'pointcloud.enable': 'true',
        }.items()
    )

    # 3. Launch the marker detection node (e.g., apriltag_ros)
    # This node detects the calibration pattern and publishes its transform.
    # You will need to configure its parameters (e.g., in apriltag.yaml)
    # to match your camera topics and calibration target.
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("apriltag_ros"), "launch", "tag_detector.launch.py"])]
        )
    )

    # 4. Launch the easy_handeye calibration solver
    easy_handeye_calibrate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("easy_handeye2"), "launch", "calibrate.launch.py"])]
        ),
        launch_arguments={
            # The 'name' argument namespaces the calibration services.
            'name': LaunchConfiguration('calibration_name'),
            'calibration_type': LaunchConfiguration('calibration_type'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
        }.items()
    )

    # 5. Launch the RQT GUI for calibration
    # We pass the calibration name to the GUI so it connects to the correct services.
    rqt_calibrator_gui = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_handeye_calibrator_gui',
        arguments=['--standalone', 'rqt_easy_handeye.easy_handeye_calibrator.EasyHandeyeCalibrator',
                   '--args', '--calibration_name', LaunchConfiguration('calibration_name')],
        output='screen',
    )

    return launch.LaunchDescription([
        calibration_name_arg,
        calibration_type_arg,
        robot_base_frame_arg,
        robot_effector_frame_arg,
        tracking_base_frame_arg,
        tracking_marker_frame_arg,
        robot_launch,
        tracking_launch,
        apriltag_launch,
        easy_handeye_calibrate_launch,
        rqt_calibrator_gui
    ])