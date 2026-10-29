#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto const logger = rclcpp::get_logger("hello_moveit");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "fairino5_v6_group");
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 0.954;
    msg.orientation.x = 0.027;
    msg.orientation.y = 0.035;
    msg.orientation.z = -0.296;
    msg.position.x = 0.185;
    msg.position.y = -0.120;
    msg.position.z = 0.791;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  rclcpp::shutdown();
  return 0;
}