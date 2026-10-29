#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "rg75_300_gripper/srv/gripper_command.hpp"

class MoveItPoseSubscriber : public rclcpp::Node
{
public:
  MoveItPoseSubscriber()
      : Node("moveit_pose_subscriber", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
  }

  void initialize()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "fairino5_v6_group");
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose", 10,
        std::bind(&MoveItPoseSubscriber::pose_callback, this, std::placeholders::_1));
    gripper_client_ = this->create_client<rg75_300_gripper::srv::GripperCommand>("/gripper_command");
    //table as a collision object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = move_group_->getPlanningFrame();
    table.id = "table";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.8, 1.2, 0.05}; // size: x, y, z

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.5; //table position
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.025; //table height above ground

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;

    planning_scene_interface.applyCollisionObjects({table});

    RCLCPP_INFO(this->get_logger(), "MoveItPoseSubscriber initialized, waiting for PoseStamped messages on /object_pose");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (published_)
    {
      RCLCPP_INFO(this->get_logger(), "Received first PoseStamped message, starting processing.");

      RCLCPP_INFO(this->get_logger(), "Received PoseStamped: position [x: %.2f, y: %.2f, z: %.2f], orientation [w: %.2f]",
                  msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.w);
      try
      {
        move_group_->setPoseTarget(msg->pose, "camera_link");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_->plan(plan));
        if (success)
        {
          RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing...");
          move_group_->execute(plan);
          RCLCPP_INFO(this->get_logger(), "Execution completed successfully");

          //Service call for gripper
          if (!gripper_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Gripper service not available!");
          }
          else{
            auto request = std::make_shared<rg75_300_gripper::srv::GripperCommand::Request>();
            request->position = 80; // Set desired position
            request->force = 50;    // Set desired force
            auto result = gripper_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
              RCLCPP_INFO(this->get_logger(), "Gripper command sent successfully");
            }
            else
            {
              RCLCPP_ERROR(this->get_logger(), "Failed to call gripper service");
            }
          }
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Exception during planning/execution: %s", e.what());
      }
      published_ = false;
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Client<rg75_300_gripper::srv::GripperCommand>::SharedPtr gripper_client_;
  bool published_ = true;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItPoseSubscriber>();
  node->initialize();
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  try
  {
    executor->spin();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node->get_logger(), "Exception during spin: %s", e.what());
  }
  executor->cancel();
  rclcpp::shutdown();
  return 0;
}