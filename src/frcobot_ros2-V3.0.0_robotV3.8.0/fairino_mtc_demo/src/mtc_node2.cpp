#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <chrono>  // For sleeps
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <shape_msgs/msg/solid_primitive.hpp>  // For SolidPrimitive
#include <geometry_msgs/msg/pose.hpp>          // For Pose
#include <geometry_msgs/msg/vector3_stamped.hpp>  // For Vector3Stamped

static const rclcpp::Logger LOGGER = rclcpp::get_logger("fairino_mtc_demo");

namespace mtc = moveit::task_constructor;

class MTCTaskNode {
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  ~MTCTaskNode();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

  void setupPlanningScene();
  void doTask();
  void doReverseTask();

  // Added: Public getter for num_cycles
  int getNumCycles() const;

private:
  mtc::Task _createTask();
  mtc::Task _createReverseTask();
  mtc::Task _task;
  mtc::Task _reverse_task;
  rclcpp::Node::SharedPtr _node;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : _node{std::make_shared<rclcpp::Node>("fairino_mtc_demo_node", options)} {
  // Declare param for cycles
  _node->declare_parameter("num_cycles", 2);
}

MTCTaskNode::~MTCTaskNode() {}

int MTCTaskNode::getNumCycles() const {
  return _node->get_parameter("num_cycles").as_int();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::get_node_base_interface() {
  return _node->get_node_base_interface();
}

void MTCTaskNode:: () {
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "base_link";  // Robot base frame for TF consistency
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = {0.05, 0.05, 0.01};  // length(x), width(y), height(z)

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.1;
  pose.position.y = -0.55;
  pose.position.z = 0.1;  // Z=0.1 places box base at ground level (height=0.01, so top at 0.11)
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  RCLCPP_INFO(LOGGER, "Added box object (0.05x0.05x0.01m) to planning scene");
}

void MTCTaskNode::doTask() {
  _task = _createTask();
  RCLCPP_INFO(LOGGER, "Starting to init forward task!");
  try {
    _task.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, "Forward task init failed: " << e.what());
    return;
  }
  try {
    if (!_task.plan(10)) {
      RCLCPP_ERROR_STREAM(LOGGER, "Forward task planning failed!");
      return;
    }
  } catch (std::runtime_error& e) {  // Fixed: Use std::runtime_error (covers MTC PlannerAborted)
    RCLCPP_ERROR_STREAM(LOGGER, "Forward planning exception: " << e.what());
    return;
  }

  if (!_task.solutions().empty()) {
    auto& sol = *_task.solutions().front();
    _task.introspection().publishSolution(sol);
    auto result = _task.execute(sol);
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(LOGGER, "Forward task execution failed with code: " << result.val);
      return;
    }
    RCLCPP_INFO(LOGGER, "Forward task executed SUCCESS!");
  } else {
    RCLCPP_WARN(LOGGER, "No forward solutions found—skipping execution");
  }
}

void MTCTaskNode::doReverseTask() {
  _reverse_task = _createReverseTask();
  RCLCPP_INFO(LOGGER, "Starting to init reverse task!");
  try {
    _reverse_task.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, "Reverse task init failed: " << e.what());
    return;
  }
  try {
    if (!_reverse_task.plan(10)) {
      RCLCPP_ERROR_STREAM(LOGGER, "Reverse task planning failed!");
      return;
    }
  } catch (std::runtime_error& e) {  // Fixed: Use std::runtime_error
    RCLCPP_ERROR_STREAM(LOGGER, "Reverse planning exception: " << e.what());
    return;
  }

  if (!_reverse_task.solutions().empty()) {
    auto& sol = *_reverse_task.solutions().front();
    _reverse_task.introspection().publishSolution(sol);
    auto result = _reverse_task.execute(sol);
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR_STREAM(LOGGER, "Reverse task execution failed with code: " << result.val);
      return;
    }
    RCLCPP_INFO(LOGGER, "Reverse task executed SUCCESS!");
  } else {
    RCLCPP_WARN(LOGGER, "No reverse solutions found—skipping execution");
  }
  _reverse_task.clear();
}

mtc::Task MTCTaskNode::_createTask() {
  mtc::Task task;
  task.stages()->setName("fairino_mtc_task");
  task.loadRobotModel(_node);
  const auto& arm_group_name = "fairino5_v6_group";
  const auto& tip_frame = "wrist3_link";

  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", tip_frame);

  // Removed: Unused current_state_ptr
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(_node);
  sampling_planner->setProperty("pipeline_id", "ompl");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setStepSize(0.01);
  cartesian_planner->setJumpThreshold(2.0);
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);

  // Move to initial position (Y negative)
  {
    auto stage_move_robot = std::make_unique<mtc::stages::MoveRelative>("movepos1", cartesian_planner);
    stage_move_robot->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_move_robot->setMinMaxDistance(0.05, 0.2);
    stage_move_robot->setIKFrame(tip_frame);
    stage_move_robot->properties().set("marker_ns", "movepos1");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = 0.0;
    vec.vector.y = -1.0;
    vec.vector.z = 0.0;
    stage_move_robot->setDirection(vec);
    task.add(std::move(stage_move_robot));
  }

  // Approach object (down)
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.1, 0.2);
    stage->setIKFrame(tip_frame);
    stage->properties().set("marker_ns", "approach object");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = 0.0;
    vec.vector.y = 0.0;
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // Attach object
  {
    auto stage_grasp = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage_grasp->attachObject("object", tip_frame);
    task.add(std::move(stage_grasp));
  }

  // Lift object (up)
  {
    auto stage_lift = std::make_unique<mtc::stages::MoveRelative>("lift obj", cartesian_planner);
    stage_lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_lift->setMinMaxDistance(0.1, 0.2);
    stage_lift->setIKFrame(tip_frame);
    stage_lift->properties().set("marker_ns", "lift obj");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = 0.0;
    vec.vector.y = 0.0;
    vec.vector.z = 1.0;
    stage_lift->setDirection(vec);
    task.add(std::move(stage_lift));
  }

  // Move to place (X negative)
  {
    auto stage_move_robot2 = std::make_unique<mtc::stages::MoveRelative>("movepos2", cartesian_planner);
    stage_move_robot2->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_move_robot2->setMinMaxDistance(0.1, 0.2);
    stage_move_robot2->setIKFrame(tip_frame);
    stage_move_robot2->properties().set("marker_ns", "movepos2");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = -1.0;
    vec.vector.y = 0.0;
    vec.vector.z = 0.0;
    stage_move_robot2->setDirection(vec);
    task.add(std::move(stage_move_robot2));
  }

  // Put down
  {
    auto stage_put = std::make_unique<mtc::stages::MoveRelative>("put obj", cartesian_planner);
    stage_put->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_put->setMinMaxDistance(0.1, 0.2);
    stage_put->setIKFrame(tip_frame);
    stage_put->properties().set("marker_ns", "put obj");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = 0.0;
    vec.vector.y = 0.0;
    vec.vector.z = -1.0;
    stage_put->setDirection(vec);
    task.add(std::move(stage_put));
  }

  // Detach object
  {
    auto stage_detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage_detach->detachObject("object", tip_frame);
    task.add(std::move(stage_detach));
  }

  // Return home (up)
  {
    auto stage_end = std::make_unique<mtc::stages::MoveRelative>("return home", cartesian_planner);
    stage_end->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_end->setMinMaxDistance(0.1, 0.2);
    stage_end->setIKFrame(tip_frame);
    stage_end->properties().set("marker_ns", "return home");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = 0.0;
    vec.vector.y = 0.0;
    vec.vector.z = 1.0;
    stage_end->setDirection(vec);
    task.add(std::move(stage_end));
  }

  return task;
}

mtc::Task MTCTaskNode::_createReverseTask() {
  mtc::Task task;
  task.stages()->setName("fairino_mtc_reverse_task");
  task.loadRobotModel(_node);
  const auto& arm_group_name = "fairino5_v6_group";
  const auto& tip_frame = "wrist3_link";

  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", tip_frame);

  // Removed: Unused current_state_ptr
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(_node);
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setStepSize(0.01);
  cartesian_planner->setJumpThreshold(2.0);
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);

  // Approach (down) - reverse of return home
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.1, 0.2);
    stage->setIKFrame(tip_frame);
    stage->properties().set("marker_ns", "approach object");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = 0.0;
    vec.vector.y = 0.0;
    vec.vector.z = -1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // Attach
  {
    auto stage_grasp = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage_grasp->attachObject("object", tip_frame);
    task.add(std::move(stage_grasp));
  }

  // Lift (up)
  {
    auto stage_lift = std::make_unique<mtc::stages::MoveRelative>("lift obj", cartesian_planner);
    stage_lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_lift->setMinMaxDistance(0.1, 0.2);
    stage_lift->setIKFrame(tip_frame);
    stage_lift->properties().set("marker_ns", "lift obj");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = 0.0;
    vec.vector.y = 0.0;
    vec.vector.z = 1.0;
    stage_lift->setDirection(vec);
    task.add(std::move(stage_lift));
  }

  // Move back (X positive, reverse of movepos2)
  {
    auto stage_move_robot = std::make_unique<mtc::stages::MoveRelative>("move back", cartesian_planner);
    stage_move_robot->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_move_robot->setMinMaxDistance(0.1, 0.2);
    stage_move_robot->setIKFrame(tip_frame);
    stage_move_robot->properties().set("marker_ns", "move back");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = 1.0;
    vec.vector.y = 0.0;
    vec.vector.z = 0.0;
    stage_move_robot->setDirection(vec);
    task.add(std::move(stage_move_robot));
  }

  // Put down (down)
  {
    auto stage_put = std::make_unique<mtc::stages::MoveRelative>("put obj", cartesian_planner);
    stage_put->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_put->setMinMaxDistance(0.1, 0.2);
    stage_put->setIKFrame(tip_frame);
    stage_put->properties().set("marker_ns", "put obj");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = 0.0;
    vec.vector.y = 0.0;
    vec.vector.z = -1.0;
    stage_put->setDirection(vec);
    task.add(std::move(stage_put));
  }

  // Detach
  {
    auto stage_detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage_detach->detachObject("object", tip_frame);
    task.add(std::move(stage_detach));
  }

  // Lift arm (up)
  {
    auto stage_end = std::make_unique<mtc::stages::MoveRelative>("lift arm", cartesian_planner);
    stage_end->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_end->setMinMaxDistance(0.1, 0.2);
    stage_end->setIKFrame(tip_frame);
    stage_end->properties().set("marker_ns", "lift arm");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = 0.0;
    vec.vector.y = 0.0;
    vec.vector.z = 1.0;
    stage_end->setDirection(vec);
    task.add(std::move(stage_end));
  }

  // Return to start (Y positive, reverse of movepos1)
  {
    auto stage_move_robot2 = std::make_unique<mtc::stages::MoveRelative>("return start", cartesian_planner);
    stage_move_robot2->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_move_robot2->setMinMaxDistance(0.1, 0.2);
    stage_move_robot2->setIKFrame(tip_frame);
    stage_move_robot2->properties().set("marker_ns", "return start");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "base_link";
    vec.vector.x = 0.0;
    vec.vector.y = 1.0;
    vec.vector.z = 0.0;
    stage_move_robot2->setDirection(vec);
    task.add(std::move(stage_move_robot2));
  }

  return task;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->get_node_base_interface());
    executor.spin();
    executor.remove_node(mtc_task_node->get_node_base_interface());
  });

  rclcpp::sleep_for(std::chrono::seconds(2));  // Wait for scene/param load
  mtc_task_node->setupPlanningScene();

  // Fixed: Use getter for private _node
  int num_cycles = mtc_task_node->getNumCycles();
  for (int i = 0; i < num_cycles; ++i) {
    RCLCPP_INFO(LOGGER, "Starting cycle %d/%d", i + 1, num_cycles);
    mtc_task_node->doTask();
    rclcpp::sleep_for(std::chrono::seconds(1));  // Pause between forward/reverse
    mtc_task_node->doReverseTask();
    rclcpp::sleep_for(std::chrono::seconds(2));  // Observe cycle
  }

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}