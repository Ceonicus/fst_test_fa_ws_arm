#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState

class MoveItGoalPublisher(Node):
    def __init__(self):
        super().__init__('moveit_goal_publisher')
        self.goal_pub = self.create_publisher(RobotState, '/rviz/moveit/update_custom_goal_state', 10)
        self.plan_pub = self.create_publisher(Empty, '/rviz/moveit/plan', 10)
        self.execute_pub = self.create_publisher(Empty, '/rviz/moveit/execute', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)

    def publish_goal(self):
        robot_state = RobotState()
        goal_pose = Pose()
        goal_pose.position.x = 0.3
        goal_pose.position.y = 0.0
        goal_pose.position.z = 0.5
        goal_pose.orientation.w = 1.0
        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = 0.0

        robot_state.attached_collision_objects = []
        robot_state.is_diff = True

        self.goal_pub.publish(robot_state)
        self.get_logger().info('Published goal pose to /rviz/moveit/update_custom_goal_state')

        plan_msg = Empty()
        self.plan_pub.publish(plan_msg)
        self.get_logger().info('Triggered planning on /rviz/moveit/plan')

        self.timer.cancel()

        self.create_timer(2.0, self.execute_plan)

    def execute_plan(self):
        execute_msg = Empty()
        self.execute_pub.publish(execute_msg)
        self.get_logger().info('Triggered execution on /rviz/moveit/execute')
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = MoveItGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()