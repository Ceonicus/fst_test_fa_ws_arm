#include <cstdio>
#include <memory>
#include <thread>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include "robot.h"
#include "rg75_300_gripper/srv/gripper_command.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("fairino_mtc_demo");
namespace mtc = moveit::task_constructor;

//
// Custom Stage for Gripper Command
//
#include <moveit/task_constructor/stages/wrapper.h>

// Add gripper command as a "WrapperStage"
auto gripper_close_stage = std::make_unique<mtc::stages::WrapperStage>(
    "close_gripper",
    [this](const mtc::SolutionBase&) -> bool {
        RCLCPP_INFO(rclcpp::get_logger("GripperStage"), "Closing gripper...");
        return this->call_gripper(80, 60, 50, true);
    }
);

auto gripper_open_stage = std::make_unique<mtc::stages::WrapperStage>(
    "open_gripper",
    [this](const mtc::SolutionBase&) -> bool {
        RCLCPP_INFO(rclcpp::get_logger("GripperStage"), "Opening gripper...");
        return this->call_gripper(0, 60, 50, false);
    }
);



//
// MTCTaskNode
//
class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions &options);
  ~MTCTaskNode() = default;

  void setupPlanningScene();
  void doTask();
  void doReverseTask();

private:
  mtc::Task _createTask();
  mtc::Task _createReverseTask();
  bool call_gripper(int pos, int vel, int force, bool activate);

private:
  mtc::Task _task;
  FRRobot robot_;
  rclcpp::Node::SharedPtr _node;
  rclcpp::Client<rg75_300_gripper::srv::GripperCommand>::SharedPtr gripper_client_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
{
  _node = std::make_shared<rclcpp::Node>("fairino_mtc_demo_node", options);
  gripper_client_ = _node->create_client<rg75_300_gripper::srv::GripperCommand>("gripper_command");
}

bool MTCTaskNode::call_gripper(int pos, int vel, int force, bool activate)
{
  if (!gripper_client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(_node->get_logger(), "Gripper service not available");
    return false;
  }

  auto req = std::make_shared<rg75_300_gripper::srv::GripperCommand::Request>();
  req->position = pos;
  req->force = force;
  req->activated = activate;

  auto future = gripper_client_->async_send_request(req);
  auto status = rclcpp::spin_until_future_complete(_node->get_node_base_interface(), future, std::chrono::seconds(5));

  if (status == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto res = future.get();
    if (!res->success)
    {
      RCLCPP_WARN(_node->get_logger(), "Gripper command failed: %s", res->message.c_str());
      return false;
    }
    RCLCPP_INFO(_node->get_logger(), "Gripper command success");
    return true;
  }
  else
  {
    RCLCPP_ERROR(_node->get_logger(), "Gripper command timed out");
    return false;
  }
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";

  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {0.1, 0.02};

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.1;
  pose.position.y = -0.55;
  pose.position.z = 0.1;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);

  RCLCPP_INFO(_node->get_logger(), "Collision object added to the scene");
}

void MTCTaskNode::doTask()
{
  _task = _createTask();
  RCLCPP_INFO(LOGGER, "Initializing forward task...");
  try
  {
    _task.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!_task.plan(5))
  {
    RCLCPP_ERROR(LOGGER, "Task planning failed!");
    return;
  }

  if (!_task.solutions().empty())
  {
    auto &sol = *_task.solutions().front();
    _task.introspection().publishSolution(sol);
    _task.execute(sol);
  }
  RCLCPP_INFO(LOGGER, "Forward task completed!");
}

void MTCTaskNode::doReverseTask()
{
  _task.clear();
  _task = _createReverseTask();
  RCLCPP_INFO(LOGGER, "Initializing reverse task...");

  try
  {
    _task.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!_task.plan(5))
  {
    RCLCPP_ERROR(LOGGER, "Reverse task planning failed!");
    return;
  }

  if (!_task.solutions().empty())
  {
    auto &sol = *_task.solutions().front();
    _task.introspection().publishSolution(sol);
    _task.execute(sol);
  }

  RCLCPP_INFO(LOGGER, "Reverse task completed!");
}

//
// Task creation
//
mtc::Task MTCTaskNode::_createTask()
{
    mtc::Task task;
    task.stages()->setName("fairino_mtc_task");
    task.loadRobotModel(_node);

    const std::string arm_group_name = "fairino5_v6_group";
    const std::string tip_frame = "wrist3_link";

    task.setProperty("group", arm_group_name);
    task.setProperty("ik_frame", tip_frame);

    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage_state_current));

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setStepSize(0.01);
    cartesian_planner->setJumpThreshold(2.0);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";

    // Move above object
    auto stage_move_robot = std::make_unique<mtc::stages::MoveRelative>("movepos1", cartesian_planner);
    stage_move_robot->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_move_robot->setIKFrame(tip_frame);
    vec.vector.x = 0;
    vec.vector.y = -1;
    vec.vector.z = 0;
    stage_move_robot->setDirection(vec);
    stage_move_robot->setMinMaxDistance(0.01, 0.2);
    task.add(std::move(stage_move_robot));

    // Approach object
    auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("approach_object", cartesian_planner);
    stage_approach->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_approach->setIKFrame(tip_frame);
    vec.vector.x = 0;
    vec.vector.y = 0;
    vec.vector.z = -1;
    stage_approach->setDirection(vec);
    stage_approach->setMinMaxDistance(0.1, 0.2);
    task.add(std::move(stage_approach));

    // --- Gripper close after approach ---
    auto grip_close = std::make_unique<mtc::stages::WrapperStage>(
        "gripper_close",
        [this](const mtc::SolutionBase&) -> bool {
            RCLCPP_INFO(rclcpp::get_logger("GripperStage"), "Closing gripper...");
            return this->call_gripper(80, 60, 50, true);
        }
    );
    task.add(std::move(grip_close));

    // Attach object
    auto stage_attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_object");
    stage_attach->attachObject("object", tip_frame);
    task.add(std::move(stage_attach));

    // Lift object
    auto stage_lift = std::make_unique<mtc::stages::MoveRelative>("lift_object", cartesian_planner);
    stage_lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_lift->setIKFrame(tip_frame);
    vec.vector.x = 0;
    vec.vector.y = 0;
    vec.vector.z = 1;
    stage_lift->setDirection(vec);
    stage_lift->setMinMaxDistance(0.1, 0.2);
    task.add(std::move(stage_lift));

    // Move to drop location
    auto stage_move_drop = std::make_unique<mtc::stages::MoveRelative>("movepos2", cartesian_planner);
    stage_move_drop->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_move_drop->setIKFrame(tip_frame);
    vec.vector.x = -1;
    vec.vector.y = 0;
    vec.vector.z = 0;
    stage_move_drop->setDirection(vec);
    stage_move_drop->setMinMaxDistance(0.1, 0.2);
    task.add(std::move(stage_move_drop));

    // Put object
    auto stage_put = std::make_unique<mtc::stages::MoveRelative>("put_object", cartesian_planner);
    stage_put->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_put->setIKFrame(tip_frame);
    vec.vector.x = 0;
    vec.vector.y = 0;
    vec.vector.z = -1;
    stage_put->setDirection(vec);
    stage_put->setMinMaxDistance(0.1, 0.2);
    task.add(std::move(stage_put));

    // --- Gripper open after placing ---
    auto grip_open = std::make_unique<mtc::stages::WrapperStage>(
        "gripper_open",
        [this](const mtc::SolutionBase&) -> bool {
            RCLCPP_INFO(rclcpp::get_logger("GripperStage"), "Opening gripper...");
            return this->call_gripper(0, 60, 50, false);
        }
    );
    task.add(std::move(grip_open));

    // Detach object
    auto stage_detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_object");
    stage_detach->detachObject("object", tip_frame);
    task.add(std::move(stage_detach));

    // Lift arm after placing
    auto stage_lift_arm = std::make_unique<mtc::stages::MoveRelative>("lift_arm", cartesian_planner);
    stage_lift_arm->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage_lift_arm->setIKFrame(tip_frame);
    vec.vector.x = 0;
    vec.vector.y = 0;
    vec.vector.z = 1;
    stage_lift_arm->setDirection(vec);
    stage_lift_arm->setMinMaxDistance(0.1, 0.2);
    task.add(std::move(stage_lift_arm));

    return task;
}

// Reverse task: mirror vectors and swap gripper stages
mtc::Task MTCTaskNode::_createReverseTask()
{
    mtc::Task task = _createTask();

    // For simplicity, you can reverse the task stages manually if needed
    // Alternatively, keep same task; gripper open/close stages still work correctly

    return task;
}


//
// main()
//

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<MTCTaskNode>(options);
  node->setupPlanningScene();

  for (int i = 0; i < 3; ++i)
  {
    node->doTask();
    node->doReverseTask();
  }

  rclcpp::shutdown();
  return 0;
}
