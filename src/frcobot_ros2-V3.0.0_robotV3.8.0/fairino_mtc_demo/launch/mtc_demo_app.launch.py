from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

packagename = "fairino5_v6_moveit2_config"


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fairino_resource", package_name=packagename).to_dict()

    # Static Transform Publisher for world -> base_link
    # This connects the robot's coordinate system to the world's coordinate system.
    static_tf_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    # MTC Demo node
    pick_place_demo = Node(
        package="fairino_mtc_demo",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    # Add the new static_tf_pub node to the list of nodes to be launched
    return LaunchDescription([static_tf_pub, pick_place_demo])