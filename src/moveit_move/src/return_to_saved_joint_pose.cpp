#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

class ReturnToSavedJointPose : public rclcpp::Node
{
public:
  ReturnToSavedJointPose()
  : Node("return_to_saved_joint_pose",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    declare_parameter_if_not_declared<std::string>("planning_group", "fairino5_v6_group");
    declare_parameter_if_not_declared<double>("j1", -1.4155149936414828);
    declare_parameter_if_not_declared<double>("j2", -1.4576652436883306);
    declare_parameter_if_not_declared<double>("j3", -1.8781316413347113);
    declare_parameter_if_not_declared<double>("j4", 0.2190782725278865);
    declare_parameter_if_not_declared<double>("j5", 2.206984498534039);
    declare_parameter_if_not_declared<double>("j6", 2.3956715922950256);
    declare_parameter_if_not_declared<double>("max_joint_delta_rad", 0.50);
    declare_parameter_if_not_declared<double>("already_at_target_tolerance_rad", 0.002);
    declare_parameter_if_not_declared<double>("planning_time_sec", 5.0);
    declare_parameter_if_not_declared<double>("velocity_scaling", 0.002);
    declare_parameter_if_not_declared<double>("acceleration_scaling", 0.002);
    declare_parameter_if_not_declared<bool>("execute_motion", false);
    declare_parameter_if_not_declared<bool>("allow_reset_execution", false);
  }

  void initialize()
  {
    planning_group_ = get_parameter("planning_group").as_string();
    target_values_by_joint_ = {
      {"j1", get_parameter("j1").as_double()},
      {"j2", get_parameter("j2").as_double()},
      {"j3", get_parameter("j3").as_double()},
      {"j4", get_parameter("j4").as_double()},
      {"j5", get_parameter("j5").as_double()},
      {"j6", get_parameter("j6").as_double()}};
    max_joint_delta_rad_ = sanitize_positive(
      get_parameter("max_joint_delta_rad").as_double(), "max_joint_delta_rad", 0.50);
    already_at_target_tolerance_rad_ = sanitize_positive(
      get_parameter("already_at_target_tolerance_rad").as_double(),
      "already_at_target_tolerance_rad", 0.002);
    planning_time_sec_ = sanitize_positive(
      get_parameter("planning_time_sec").as_double(), "planning_time_sec", 5.0);
    velocity_scaling_ = sanitize_scaling(
      get_parameter("velocity_scaling").as_double(), "velocity_scaling", 0.002);
    acceleration_scaling_ = sanitize_scaling(
      get_parameter("acceleration_scaling").as_double(), "acceleration_scaling", 0.002);
    execute_motion_ = get_parameter("execute_motion").as_bool();
    allow_reset_execution_ = get_parameter("allow_reset_execution").as_bool();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->setPlanningTime(planning_time_sec_);
    move_group_->setNumPlanningAttempts(1);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ReturnToSavedJointPose::joint_state_callback, this, std::placeholders::_1));
    joint_state_wait_start_ = now();

    RCLCPP_WARN(
      get_logger(),
      "SAFE RESET UTILITY: return_to_saved_joint_pose uses only joint value targets. "
      "It does not call setPoseTarget, does not call computeCartesianPath, and does not "
      "command gripper or suction hardware.");
    RCLCPP_INFO(
      get_logger(),
      "Configured reset: planning_group=%s, max_joint_delta_rad=%.6f, "
      "already_at_target_tolerance_rad=%.6f, planning_time_sec=%.6f, "
      "velocity_scaling=%.6f, acceleration_scaling=%.6f, execute_motion=%s, "
      "allow_reset_execution=%s. Waiting up to %.3f seconds for one /joint_states message.",
      planning_group_.c_str(),
      max_joint_delta_rad_,
      already_at_target_tolerance_rad_,
      planning_time_sec_,
      velocity_scaling_,
      acceleration_scaling_,
      execute_motion_ ? "true" : "false",
      allow_reset_execution_ ? "true" : "false",
      joint_state_timeout_sec_);

    if (execute_motion_ && !allow_reset_execution_) {
      RCLCPP_ERROR(
        get_logger(),
        "execute_motion:=true was requested, but allow_reset_execution:=false. "
        "Execution is blocked. Planning-only validation may still run.");
    }
    if (execute_motion_ && allow_reset_execution_) {
      RCLCPP_WARN(
        get_logger(),
        "REAL ROBOT MOTION ENABLED: this node will execute one low-speed joint-space "
        "reset if all safety checks pass. Gripper and suction hardware are not commanded.");
    }

    run_timer_ = create_wall_timer(
      100ms, std::bind(&ReturnToSavedJointPose::run_once, this));
  }

  int exit_code() const
  {
    return run_successful_ ? 0 : 1;
  }

private:
  template <typename ParameterT>
  void declare_parameter_if_not_declared(const std::string & name, const ParameterT & default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<ParameterT>(name, default_value);
    }
  }

  void run_once()
  {
    if (has_run_) {
      return;
    }

    if (!has_latest_joint_state_) {
      const double elapsed_sec = (now() - joint_state_wait_start_).seconds();
      if (!waiting_for_joint_state_logged_) {
        waiting_for_joint_state_logged_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Waiting for the first /joint_states message before planning the saved joint reset.");
      }
      if (elapsed_sec < joint_state_timeout_sec_) {
        return;
      }

      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: no /joint_states message received within %.3f seconds. "
        "Refusing to plan or execute.",
        joint_state_timeout_sec_);
      run_successful_ = false;
      rclcpp::shutdown();
      return;
    }

    has_run_ = true;
    if (run_timer_) {
      run_timer_->cancel();
    }

    try {
      run_successful_ = plan_reset();
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "Exception during saved joint reset planning: %s", exc.what());
      run_successful_ = false;
    }

    rclcpp::shutdown();
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_joint_state_ = *msg;
    has_latest_joint_state_ = true;
  }

  bool plan_reset()
  {
    const auto robot_model = move_group_->getRobotModel();
    if (!robot_model) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: failed to read MoveIt robot model. "
        "Refusing to plan or execute.");
      return false;
    }

    const auto * joint_model_group = robot_model->getJointModelGroup(planning_group_);
    if (joint_model_group == nullptr) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: planning group '%s' was not found in the robot model. "
        "Refusing to plan or execute.",
        planning_group_.c_str());
      return false;
    }

    const auto & moveit_joint_names = joint_model_group->getVariableNames();
    if (moveit_joint_names.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: planning group '%s' has no joint variables. "
        "Refusing to plan or execute.",
        planning_group_.c_str());
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "MoveIt joint variable order for planning group '%s': %s. "
      "Latest /joint_states stamp=%d.%09u, message order=%s",
      planning_group_.c_str(),
      join(moveit_joint_names).c_str(),
      latest_joint_state_.header.stamp.sec,
      latest_joint_state_.header.stamp.nanosec,
      join(latest_joint_state_.name).c_str());

    if (!target_covers_moveit_group(moveit_joint_names)) {
      return false;
    }

    const auto current_values_by_joint = current_values_from_joint_state(moveit_joint_names);
    if (current_values_by_joint.empty()) {
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Reset target values are mapped by MoveIt joint name, not by /joint_states order.");
    double max_current_to_target_delta = 0.0;
    if (!current_to_target_guard_passed(
        current_values_by_joint, moveit_joint_names, max_current_to_target_delta))
    {
      return false;
    }

    if (max_current_to_target_delta <= already_at_target_tolerance_rad_) {
      RCLCPP_INFO(
        get_logger(),
        "Robot is already at the saved reset joint pose. Largest current-to-target delta "
        "is %.12f rad, tolerance is %.12f rad. No planning or execution needed.",
        max_current_to_target_delta,
        already_at_target_tolerance_rad_);
      return true;
    }

    move_group_->clearPoseTargets();
    moveit::core::RobotState start_state(robot_model);
    start_state.setToDefaultValues();
    start_state.setVariablePositions(current_values_by_joint);
    start_state.update(true);
    move_group_->setStartState(start_state);

    const bool target_set = move_group_->setJointValueTarget(target_values_by_joint_);
    if (!target_set) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: MoveIt rejected the joint value target, likely due to bounds. "
        "Refusing to plan or execute.");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool plan_success = static_cast<bool>(move_group_->plan(plan));
    if (!plan_success) {
      RCLCPP_ERROR(get_logger(), "Saved joint reset planning failed. No execution attempted.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Saved joint reset planning succeeded.");
    if (!planned_delta_guard_passed(plan)) {
      return false;
    }

    if (execute_motion_ && allow_reset_execution_) {
      RCLCPP_WARN(
        get_logger(),
        "Executing saved joint reset now at velocity_scaling=%.6f and "
        "acceleration_scaling=%.6f. Gripper and suction hardware are not commanded.",
        velocity_scaling_,
        acceleration_scaling_);
      const bool execute_success = static_cast<bool>(move_group_->execute(plan));
      if (execute_success) {
        RCLCPP_INFO(get_logger(), "Saved joint reset execution completed successfully.");
        return true;
      } else {
        RCLCPP_ERROR(get_logger(), "Saved joint reset execution failed or was aborted.");
        return false;
      }
    }

    if (execute_motion_ && !allow_reset_execution_) {
      RCLCPP_ERROR(
        get_logger(),
        "Execution request blocked: allow_reset_execution is false.");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Execution remains disabled. Set both execute_motion:=true and "
      "allow_reset_execution:=true only for a supervised reset movement.");
    return true;
  }

  bool target_covers_moveit_group(const std::vector<std::string> & moveit_joint_names) const
  {
    for (const auto & joint_name : moveit_joint_names) {
      if (target_values_by_joint_.find(joint_name) == target_values_by_joint_.end()) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: planning group joint '%s' has no user-facing target parameter. "
          "This utility only accepts logical robot joints j1 through j6.",
          joint_name.c_str());
        return false;
      }
    }

    for (const auto & target_entry : target_values_by_joint_) {
      if (std::find(moveit_joint_names.begin(), moveit_joint_names.end(), target_entry.first) ==
        moveit_joint_names.end())
      {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: target joint '%s' is not in MoveIt planning group '%s'. "
          "Refusing to plan or execute.",
          target_entry.first.c_str(),
          planning_group_.c_str());
        return false;
      }
    }

    return true;
  }

  std::map<std::string, double> current_values_from_joint_state(
    const std::vector<std::string> & moveit_joint_names) const
  {
    std::map<std::string, double> current_values_by_joint;
    for (const auto & joint_name : moveit_joint_names) {
      const auto name_it = std::find(
        latest_joint_state_.name.begin(), latest_joint_state_.name.end(), joint_name);
      if (name_it == latest_joint_state_.name.end()) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: latest /joint_states message does not contain joint '%s'. "
          "Refusing to plan or execute.",
          joint_name.c_str());
        return {};
      }

      const auto index = static_cast<std::size_t>(
        std::distance(latest_joint_state_.name.begin(), name_it));
      if (index >= latest_joint_state_.position.size()) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: latest /joint_states message has joint '%s' at index %zu, "
          "but position array size is only %zu. Refusing to plan or execute.",
          joint_name.c_str(),
          index,
          latest_joint_state_.position.size());
        return {};
      }

      const double position = latest_joint_state_.position[index];
      if (!std::isfinite(position)) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: latest /joint_states position for joint '%s' is not finite. "
          "Refusing to plan or execute.",
          joint_name.c_str());
        return {};
      }

      current_values_by_joint[joint_name] = position;
    }

    return current_values_by_joint;
  }

  bool current_to_target_guard_passed(
    const std::map<std::string, double> & current_values_by_joint,
    const std::vector<std::string> & moveit_joint_names,
    double & max_delta) const
  {
    bool passed = true;
    max_delta = 0.0;
    for (const auto & joint_name : moveit_joint_names) {
      const double current = current_values_by_joint.at(joint_name);
      const double target = target_values_by_joint_.at(joint_name);
      const double delta = std::fabs(target - current);
      if (std::isfinite(delta) && delta > max_delta) {
        max_delta = delta;
      }
      RCLCPP_INFO(
        get_logger(),
        "Cached current joint value from /joint_states: %s=%.12f rad",
        joint_name.c_str(),
        current);
      RCLCPP_INFO(
        get_logger(),
        "Saved reset target joint value: %s=%.12f rad",
        joint_name.c_str(),
        target);
      RCLCPP_INFO(
        get_logger(),
        "Pre-plan joint check: %s current=%.12f rad, target=%.12f rad, "
        "delta=%.12f rad, limit=%.12f rad",
        joint_name.c_str(),
        current,
        target,
        delta,
        max_joint_delta_rad_);
      if (!std::isfinite(delta) || delta > max_joint_delta_rad_) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: current-to-target joint delta for %s exceeds limit "
          "(current=%.12f rad, target=%.12f rad, delta=%.12f rad, limit=%.12f rad). "
          "Refusing to plan or execute.",
          joint_name.c_str(),
          current,
          target,
          delta,
          max_joint_delta_rad_);
        passed = false;
      }
    }

    return passed;
  }

  bool planned_delta_guard_passed(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan) const
  {
    const auto & joint_trajectory = plan.trajectory_.joint_trajectory;
    const auto & joint_names = joint_trajectory.joint_names;
    const auto & points = joint_trajectory.points;

    if (joint_names.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: planned trajectory has no joint names. Refusing execution.");
      return false;
    }
    if (points.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: planned trajectory has no joint points. Refusing execution.");
      return false;
    }

    const auto & start_positions = points.front().positions;
    const auto & final_positions = points.back().positions;
    if (start_positions.size() != joint_names.size() ||
      final_positions.size() != joint_names.size())
    {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: planned trajectory position count does not match joint names "
        "(joints=%zu, start_positions=%zu, final_positions=%zu). Refusing execution.",
        joint_names.size(),
        start_positions.size(),
        final_positions.size());
      return false;
    }

    bool passed = true;
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
      const double start = start_positions[index];
      const double final = final_positions[index];
      const double delta = std::fabs(final - start);
      RCLCPP_INFO(
        get_logger(),
        "Planned joint delta: %s start=%.12f rad, final=%.12f rad, "
        "delta=%.12f rad, limit=%.12f rad",
        joint_names[index].c_str(),
        start,
        final,
        delta,
        max_joint_delta_rad_);
      if (!std::isfinite(delta) || delta > max_joint_delta_rad_) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: planned joint delta for %s exceeds limit "
          "(start=%.12f rad, final=%.12f rad, delta=%.12f rad, limit=%.12f rad). "
          "Refusing execution.",
          joint_names[index].c_str(),
          start,
          final,
          delta,
          max_joint_delta_rad_);
        passed = false;
      }
    }

    return passed;
  }

  double sanitize_positive(
    const double value,
    const std::string & parameter_name,
    const double default_value) const
  {
    if (!std::isfinite(value) || value <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "%s must be finite and > 0. Using %.6f instead.",
        parameter_name.c_str(),
        default_value);
      return default_value;
    }

    return value;
  }

  double sanitize_scaling(
    const double value,
    const std::string & parameter_name,
    const double default_value) const
  {
    if (!std::isfinite(value) || value <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "%s must be finite and > 0. Using %.6f instead.",
        parameter_name.c_str(),
        default_value);
      return default_value;
    }

    if (value > 1.0) {
      RCLCPP_WARN(
        get_logger(),
        "%s must be <= 1.0. Clamping to 1.0.",
        parameter_name.c_str());
      return 1.0;
    }

    return value;
  }

  static std::string join(const std::vector<std::string> & values)
  {
    std::ostringstream output;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        output << ", ";
      }
      output << values[index];
    }
    return output.str();
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr run_timer_;
  std::string planning_group_;
  std::map<std::string, double> target_values_by_joint_;
  sensor_msgs::msg::JointState latest_joint_state_;
  rclcpp::Time joint_state_wait_start_;
  double joint_state_timeout_sec_{5.0};
  double max_joint_delta_rad_{0.50};
  double already_at_target_tolerance_rad_{0.002};
  double planning_time_sec_{5.0};
  double velocity_scaling_{0.002};
  double acceleration_scaling_{0.002};
  bool execute_motion_{false};
  bool allow_reset_execution_{false};
  bool has_run_{false};
  bool has_latest_joint_state_{false};
  bool waiting_for_joint_state_logged_{false};
  bool run_successful_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReturnToSavedJointPose>();
  try {
    node->initialize();
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to initialize saved joint reset node: %s. "
      "Use `ros2 launch moveit_move return_to_saved_joint_pose.launch.py` so this node "
      "receives robot_description, robot_description_semantic, kinematics, and planning "
      "pipeline parameters.",
      exc.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  try {
    executor.spin();
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(node->get_logger(), "Exception during spin: %s", exc.what());
  }

  executor.cancel();
  const int exit_code = node->exit_code();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
