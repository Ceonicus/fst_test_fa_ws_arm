from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    calibration_params = {
        "name": "fairino_realsense_wrist2",
        "calibration_type": "eye_in_hand",
        "tracking_base_frame": "camera_color_optical_frame",
        "tracking_marker_frame": "tag36h11_0",
        "robot_base_frame": "base_link",
        "robot_effector_frame": "wrist2_link",
    }

    handeye_server = Node(
        package="easy_handeye2",
        executable="handeye_server",
        name="handeye_server",
        parameters=[calibration_params],
        output="screen",
    )

    handeye_rqt_calibrator = Node(
        package="easy_handeye2",
        executable="rqt_calibrator.py",
        name="handeye_rqt_calibrator",
        parameters=[calibration_params],
        output="screen",
    )

    return LaunchDescription([
        handeye_server,
        handeye_rqt_calibrator,
    ])
