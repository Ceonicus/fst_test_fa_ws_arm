#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ContactBoxPlanDebug : public rclcpp::Node
{
public:
  ContactBoxPlanDebug()
  : Node("contact_box_plan_debug",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    declare_parameter_if_not_declared<std::string>("input_topic", "/contact_box_pose_debug");
    declare_parameter_if_not_declared<std::string>("planning_group", "fairino5_v6_group");
    declare_parameter_if_not_declared<std::string>("target_link", "wrist3_link");
    declare_parameter_if_not_declared<std::string>("planning_mode", "pose_target");
    declare_parameter_if_not_declared<bool>("execute_motion", false);
    declare_parameter_if_not_declared<double>("approach_offset_x", 0.0);
    declare_parameter_if_not_declared<double>("approach_offset_y", 0.0);
    declare_parameter_if_not_declared<double>("approach_offset_z", 0.0);
    declare_parameter_if_not_declared<bool>("use_tool_offset", true);
    declare_parameter_if_not_declared<double>("tool_offset_x", 0.0);
    declare_parameter_if_not_declared<double>("tool_offset_y", 0.0);
    declare_parameter_if_not_declared<double>("tool_offset_z", 0.263);
    declare_parameter_if_not_declared<bool>("use_fixed_orientation", false);
    declare_parameter_if_not_declared<double>("target_qx", 0.0);
    declare_parameter_if_not_declared<double>("target_qy", 0.0);
    declare_parameter_if_not_declared<double>("target_qz", 0.0);
    declare_parameter_if_not_declared<double>("target_qw", 1.0);
    declare_parameter_if_not_declared<bool>("plan_once", true);
    declare_parameter_if_not_declared<bool>("use_tool_standoff", true);
    declare_parameter_if_not_declared<double>("standoff_distance", 0.05);
    declare_parameter_if_not_declared<bool>("allow_standoff_execution", false);
    declare_parameter_if_not_declared<double>("velocity_scaling", 0.001);
    declare_parameter_if_not_declared<double>("acceleration_scaling", 0.001);
    declare_parameter_if_not_declared<double>("max_wrist_target_distance", 0.05);
    declare_parameter_if_not_declared<double>("cartesian_eef_step", 0.001);
    declare_parameter_if_not_declared<double>("cartesian_jump_threshold", 0.0);
    declare_parameter_if_not_declared<double>("cartesian_min_fraction", 0.99);
    declare_parameter_if_not_declared<double>("max_joint_delta_rad", 0.08);
    declare_parameter_if_not_declared<bool>("lock_contact_pose", true);
    declare_parameter_if_not_declared<int>("contact_pose_sample_count", 5);
    declare_parameter_if_not_declared<double>("contact_pose_sample_timeout_sec", 5.0);
    declare_parameter_if_not_declared<double>("contact_pose_max_spread_m", 0.015);
    declare_parameter_if_not_declared<std::string>("locked_contact_pose_source", "live_average");
    declare_parameter_if_not_declared<double>("locked_contact_x", 0.0);
    declare_parameter_if_not_declared<double>("locked_contact_y", 0.0);
    declare_parameter_if_not_declared<double>("locked_contact_z", 0.0);
    declare_parameter_if_not_declared<bool>("exit_after_completion", true);
  }

  int exit_code() const
  {
    return exit_code_;
  }

  void initialize()
  {
    input_topic_ = get_parameter("input_topic").as_string();
    planning_group_ = get_parameter("planning_group").as_string();
    target_link_ = get_parameter("target_link").as_string();
    planning_mode_ = get_parameter("planning_mode").as_string();
    if (planning_mode_ != "pose_target" && planning_mode_ != "cartesian") {
      throw std::runtime_error(
              "Invalid planning_mode '" + planning_mode_ +
              "'. Allowed values are 'pose_target' and 'cartesian'.");
    }
    get_parameter_or<bool>("execute_motion", execute_motion_, false);
    approach_offset_x_ = get_parameter("approach_offset_x").as_double();
    approach_offset_y_ = get_parameter("approach_offset_y").as_double();
    approach_offset_z_ = get_parameter("approach_offset_z").as_double();
    use_tool_offset_ = get_parameter("use_tool_offset").as_bool();
    tool_offset_x_ = get_parameter("tool_offset_x").as_double();
    tool_offset_y_ = get_parameter("tool_offset_y").as_double();
    tool_offset_z_ = get_parameter("tool_offset_z").as_double();
    use_fixed_orientation_ = get_parameter("use_fixed_orientation").as_bool();
    target_qx_ = get_parameter("target_qx").as_double();
    target_qy_ = get_parameter("target_qy").as_double();
    target_qz_ = get_parameter("target_qz").as_double();
    target_qw_ = get_parameter("target_qw").as_double();
    plan_once_ = get_parameter("plan_once").as_bool();
    get_parameter_or<bool>("exit_after_completion", exit_after_completion_, plan_once_);
    use_tool_standoff_ = get_parameter("use_tool_standoff").as_bool();
    standoff_distance_ = get_parameter("standoff_distance").as_double();
    allow_standoff_execution_ = get_parameter("allow_standoff_execution").as_bool();
    velocity_scaling_ = sanitize_scaling(
      get_parameter("velocity_scaling").as_double(), "velocity_scaling");
    acceleration_scaling_ = sanitize_scaling(
      get_parameter("acceleration_scaling").as_double(), "acceleration_scaling");
    max_wrist_target_distance_ = sanitize_positive(
      get_parameter("max_wrist_target_distance").as_double(), "max_wrist_target_distance", 0.05);
    cartesian_eef_step_ = sanitize_positive(
      get_parameter("cartesian_eef_step").as_double(), "cartesian_eef_step", 0.001);
    cartesian_jump_threshold_ = sanitize_nonnegative(
      get_parameter("cartesian_jump_threshold").as_double(), "cartesian_jump_threshold", 0.0);
    cartesian_min_fraction_ = sanitize_fraction(
      get_parameter("cartesian_min_fraction").as_double(), "cartesian_min_fraction", 0.99);
    max_joint_delta_rad_ = sanitize_positive(
      get_parameter("max_joint_delta_rad").as_double(), "max_joint_delta_rad", 0.08);
    lock_contact_pose_ = get_parameter("lock_contact_pose").as_bool();
    contact_pose_sample_count_ = sanitize_positive_int(
      static_cast<int>(get_parameter("contact_pose_sample_count").as_int()),
      "contact_pose_sample_count", 5);
    contact_pose_sample_timeout_sec_ = sanitize_positive(
      get_parameter("contact_pose_sample_timeout_sec").as_double(),
      "contact_pose_sample_timeout_sec", 5.0);
    contact_pose_max_spread_m_ = sanitize_positive(
      get_parameter("contact_pose_max_spread_m").as_double(),
      "contact_pose_max_spread_m", 0.015);
    locked_contact_pose_source_ = get_parameter("locked_contact_pose_source").as_string();
    if (locked_contact_pose_source_ != "live_average" &&
      locked_contact_pose_source_ != "manual_params")
    {
      throw std::runtime_error(
              "Invalid locked_contact_pose_source '" + locked_contact_pose_source_ +
              "'. Allowed values are 'live_average' and 'manual_params'.");
    }
    locked_contact_x_ = get_parameter("locked_contact_x").as_double();
    locked_contact_y_ = get_parameter("locked_contact_y").as_double();
    locked_contact_z_ = get_parameter("locked_contact_z").as_double();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    if (planning_mode_ == "cartesian" && !move_group_->setEndEffectorLink(target_link_)) {
      throw std::runtime_error(
              "Failed to set target_link '" + target_link_ +
              "' as the MoveIt end-effector link for Cartesian planning.");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, shared_from_this(), true);

    move_group_->startStateMonitor(5.0);
    RCLCPP_INFO(
      get_logger(),
      "Started MoveIt current-state monitor and waited for fresh joint state.");

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, 10,
      std::bind(&ContactBoxPlanDebug::pose_callback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ContactBoxPlanDebug::joint_state_callback, this, std::placeholders::_1));
    lock_contact_pose_start_time_ = now();
    lock_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ContactBoxPlanDebug::lock_timer_callback, this));

    RCLCPP_WARN(
      get_logger(),
      "DEBUG ONLY: contact_box_plan_debug subscribes to %s and performs MoveIt planning only. "
      "It does not publish /object_pose, executes only when execute_motion and "
      "allow_standoff_execution are both true, and does not command gripper or suction hardware.",
      input_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Configured planning: planning_group=%s, target_link=%s, planning_mode=%s, "
      "execute_motion=%s, "
      "approach_offset=(%.4f, %.4f, %.4f), use_tool_offset=%s, "
      "tool_offset=(%.4f, %.4f, %.4f), use_fixed_orientation=%s, "
      "target_orientation=(%.4f, %.4f, %.4f, %.4f), "
      "use_tool_standoff=%s, standoff_distance=%.4f, "
      "allow_standoff_execution=%s, velocity_scaling=%.4f, acceleration_scaling=%.4f, "
      "max_wrist_target_distance=%.4f, cartesian_eef_step=%.4f, "
      "cartesian_jump_threshold=%.4f, cartesian_min_fraction=%.4f, "
      "max_joint_delta_rad=%.4f, lock_contact_pose=%s, contact_pose_sample_count=%d, "
      "contact_pose_sample_timeout_sec=%.4f, contact_pose_max_spread_m=%.4f, "
      "locked_contact_pose_source=%s, locked_contact=(%.4f, %.4f, %.4f), "
      "exit_after_completion=%s",
      planning_group_.c_str(), target_link_.c_str(), planning_mode_.c_str(),
      execute_motion_ ? "true" : "false",
      approach_offset_x_, approach_offset_y_, approach_offset_z_,
      use_tool_offset_ ? "true" : "false", tool_offset_x_, tool_offset_y_, tool_offset_z_,
      use_fixed_orientation_ ? "true" : "false",
      target_qx_, target_qy_, target_qz_, target_qw_,
      use_tool_standoff_ ? "true" : "false", standoff_distance_,
      allow_standoff_execution_ ? "true" : "false",
      velocity_scaling_, acceleration_scaling_,
      max_wrist_target_distance_, cartesian_eef_step_,
      cartesian_jump_threshold_, cartesian_min_fraction_,
      max_joint_delta_rad_,
      lock_contact_pose_ ? "true" : "false",
      contact_pose_sample_count_,
      contact_pose_sample_timeout_sec_,
      contact_pose_max_spread_m_,
      locked_contact_pose_source_.c_str(),
      locked_contact_x_, locked_contact_y_, locked_contact_z_,
      exit_after_completion_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "plan_once=%s. When true, the node plans for the first received contact-box pose only.",
      plan_once_ ? "true" : "false");

    RCLCPP_INFO(
      get_logger(),
      "MoveIt planning frame=%s, configured end-effector link=%s",
      move_group_->getPlanningFrame().c_str(), move_group_->getEndEffectorLink().c_str());

    if (use_tool_offset_) {
      RCLCPP_WARN(
        get_logger(),
        "TEMPORARY TOOL OFFSET: interpreting tool_offset as target_link->suction-contact "
        "in the target_link frame. The default z=0.263 m is the validated "
        "testing_home_pose_v5_36cm_reset suction-contact offset; final pickup work should "
        "still add the measured contact point to the URDF as suction_tcp.");
      if (target_link_ != "wrist3_link") {
        RCLCPP_WARN(
          get_logger(),
          "use_tool_offset:=true is intended for the temporary wrist3_link->suction-contact "
          "estimate, but target_link is %s.",
          target_link_.c_str());
      }
    }

    if (use_fixed_orientation_) {
      RCLCPP_WARN(
        get_logger(),
        "FIXED ORIENTATION ENABLED: using target quaternion "
        "(%.4f, %.4f, %.4f, %.4f) for the plan-only wrist target. "
        "This should represent a suction-facing wrist3_link orientation.",
        target_qx_, target_qy_, target_qz_, target_qw_);
    }

    if (execute_motion_ && !allow_standoff_execution_) {
      RCLCPP_ERROR(
        get_logger(),
        "execute_motion:=true was requested, but allow_standoff_execution:=false. "
        "Execution is blocked. Set both flags true only for a supervised low-speed "
        "standoff dry-run with suction disabled.");
    }

    if (execute_motion_ && allow_standoff_execution_) {
      RCLCPP_WARN(
        get_logger(),
        "REAL ROBOT MOTION ENABLED: this node will execute exactly one planned movement "
        "to the standoff pose only. Suction/gripper hardware is still not commanded. "
        "Velocity scaling=%.4f, acceleration scaling=%.4f.",
        velocity_scaling_, acceleration_scaling_);
    }
  }

private:
  struct Vector3
  {
    double x;
    double y;
    double z;
  };

  template <typename ParameterT>
  void declare_parameter_if_not_declared(const std::string & name, const ParameterT & default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter<ParameterT>(name, default_value);
    }
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (planning_mode_ == "cartesian" && lock_contact_pose_) {
      handle_locked_contact_pose_sample(*msg);
      return;
    }

    plan_from_contact_pose(*msg);
  }

  void plan_from_contact_pose(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (plan_once_ && has_planned_) {
      return;
    }

    if (plan_once_) {
      has_planned_ = true;
      RCLCPP_INFO(
        get_logger(),
        "plan_once:=true, accepting first contact-box pose and ignoring future pose updates.");
    }

    geometry_msgs::msg::PoseStamped desired_contact_pose = msg;
    if (desired_contact_pose.header.frame_id.empty()) {
      desired_contact_pose.header.frame_id = move_group_->getPlanningFrame();
      RCLCPP_WARN(
        get_logger(),
        "Received pose with empty frame_id; using MoveIt planning frame %s.",
        desired_contact_pose.header.frame_id.c_str());
    }

    desired_contact_pose.pose.position.x += approach_offset_x_;
    desired_contact_pose.pose.position.y += approach_offset_y_;
    desired_contact_pose.pose.position.z += approach_offset_z_;

    if (use_fixed_orientation_) {
      desired_contact_pose.pose.orientation.x = target_qx_;
      desired_contact_pose.pose.orientation.y = target_qy_;
      desired_contact_pose.pose.orientation.z = target_qz_;
      desired_contact_pose.pose.orientation.w = target_qw_;
    }

    normalize_or_reset_orientation(desired_contact_pose);

    Vector3 rotated_tool_offset{0.0, 0.0, 0.0};
    Vector3 standoff_vector{0.0, 0.0, 0.0};

    if (use_tool_offset_ || use_tool_standoff_) {
      const Vector3 tool_offset{tool_offset_x_, tool_offset_y_, tool_offset_z_};
      rotated_tool_offset = rotate_vector_by_quaternion(desired_contact_pose.pose.orientation, tool_offset);
    }

    if (use_tool_standoff_) {
      const double tool_offset_norm = vector_norm(rotated_tool_offset);
      if (tool_offset_norm < 1e-6) {
        RCLCPP_WARN(
          get_logger(),
          "use_tool_standoff:=true requested, but rotated tool offset is near zero. "
          "Skipping standoff adjustment.");
      } else {
        standoff_vector = scale(rotated_tool_offset, standoff_distance_ / tool_offset_norm);
        desired_contact_pose.pose.position.x -= standoff_vector.x;
        desired_contact_pose.pose.position.y -= standoff_vector.y;
        desired_contact_pose.pose.position.z -= standoff_vector.z;
      }
    }

    geometry_msgs::msg::PoseStamped target_pose = desired_contact_pose;
    if (use_tool_offset_) {
      target_pose.pose.position.x -= rotated_tool_offset.x;
      target_pose.pose.position.y -= rotated_tool_offset.y;
      target_pose.pose.position.z -= rotated_tool_offset.z;
    }
    normalize_or_reset_orientation(target_pose);

    RCLCPP_INFO(
      get_logger(),
      "Received contact-box debug pose frame=%s stamp=%d.%09u. Desired temporary suction "
      "pose after approach/standoff: position=(%.4f, %.4f, %.4f), "
      "orientation=(%.4f, %.4f, %.4f, %.4f). Planning target for %s: "
      "position=(%.4f, %.4f, %.4f), orientation=(%.4f, %.4f, %.4f, %.4f), "
      "use_tool_offset=%s, rotated_tool_offset=(%.4f, %.4f, %.4f), "
      "use_tool_standoff=%s, standoff_distance=%.4f, standoff_vector=(%.4f, %.4f, %.4f).",
      target_pose.header.frame_id.c_str(), msg.header.stamp.sec, msg.header.stamp.nanosec,
      desired_contact_pose.pose.position.x, desired_contact_pose.pose.position.y,
      desired_contact_pose.pose.position.z,
      desired_contact_pose.pose.orientation.x, desired_contact_pose.pose.orientation.y,
      desired_contact_pose.pose.orientation.z, desired_contact_pose.pose.orientation.w,
      target_link_.c_str(),
      target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
      target_pose.pose.orientation.x, target_pose.pose.orientation.y,
      target_pose.pose.orientation.z, target_pose.pose.orientation.w,
      use_tool_offset_ ? "true" : "false",
      rotated_tool_offset.x, rotated_tool_offset.y, rotated_tool_offset.z,
      use_tool_standoff_ ? "true" : "false", standoff_distance_,
      standoff_vector.x, standoff_vector.y, standoff_vector.z);

    try {
      move_group_->clearPoseTargets();
      if (!set_start_state_from_latest_joint_state()) {
        request_completion_shutdown(
          false, "SAFETY BLOCKED: failed to set planning start state from /joint_states.");
        return;
      }

      const std::string planning_frame = move_group_->getPlanningFrame();
      geometry_msgs::msg::Pose current_wrist_pose;
      try {
        const auto current_transform = tf_buffer_->lookupTransform(
          planning_frame, target_link_, tf2::TimePointZero);
        current_wrist_pose.position.x = current_transform.transform.translation.x;
        current_wrist_pose.position.y = current_transform.transform.translation.y;
        current_wrist_pose.position.z = current_transform.transform.translation.z;
        current_wrist_pose.orientation = current_transform.transform.rotation;
      } catch (const tf2::TransformException & exc) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: failed to lookup latest TF transform %s -> %s: %s. "
          "Refusing to plan or execute.",
          planning_frame.c_str(), target_link_.c_str(), exc.what());
        move_group_->clearPoseTargets();
        request_completion_shutdown(
          false, "SAFETY BLOCKED: failed to read latest wrist TF for target distance check.");
        return;
      }
      if (target_pose.header.frame_id != planning_frame) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: target pose frame '%s' does not match MoveIt planning frame '%s'. "
          "Refusing to plan or execute because wrist-distance and Cartesian waypoints are "
          "evaluated in the planning frame.",
          target_pose.header.frame_id.c_str(),
          planning_frame.c_str());
        move_group_->clearPoseTargets();
        request_completion_shutdown(
          false, "SAFETY BLOCKED: target pose frame does not match planning frame.");
        return;
      }
      const auto & target_position = target_pose.pose.position;

      const double dx = target_position.x - current_wrist_pose.position.x;
      const double dy = target_position.y - current_wrist_pose.position.y;
      const double dz = target_position.z - current_wrist_pose.position.z;
      const double wrist_target_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      RCLCPP_WARN(
        get_logger(),
        "Safety check: current %s position=(%.4f, %.4f, %.4f), "
        "target position=(%.4f, %.4f, %.4f), "
        "delta=(%.4f, %.4f, %.4f), distance=%.4f m, limit=%.4f m",
        target_link_.c_str(),
        current_wrist_pose.position.x, current_wrist_pose.position.y, current_wrist_pose.position.z,
        target_position.x, target_position.y, target_position.z,
        dx, dy, dz,
        wrist_target_distance,
        max_wrist_target_distance_);

      if (wrist_target_distance > max_wrist_target_distance_) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: wrist target distance %.4f m exceeds limit %.4f m. "
          "Refusing to plan or execute.",
          wrist_target_distance,
          max_wrist_target_distance_);
        move_group_->clearPoseTargets();
        request_completion_shutdown(
          false, "SAFETY BLOCKED: wrist target distance exceeded configured limit.");
        return;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = false;
      if (planning_mode_ == "pose_target") {
        move_group_->setPoseTarget(target_pose, target_link_);
        success = static_cast<bool>(move_group_->plan(plan));
      } else {
        move_group_->setPoseReferenceFrame(planning_frame);
        const auto waypoints =
          make_cartesian_waypoints(current_wrist_pose, target_pose.pose, wrist_target_distance);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = move_group_->computeCartesianPath(
          waypoints, cartesian_eef_step_, cartesian_jump_threshold_, trajectory, true);
        RCLCPP_INFO(
          get_logger(),
          "Cartesian path planning for target_link=%s produced fraction=%.4f with %zu "
          "waypoints and %zu trajectory points. Required minimum fraction=%.4f.",
          target_link_.c_str(),
          fraction,
          waypoints.size(),
          trajectory.joint_trajectory.points.size(),
          cartesian_min_fraction_);
        if (fraction < cartesian_min_fraction_) {
          RCLCPP_ERROR(
            get_logger(),
            "SAFETY BLOCKED: Cartesian path fraction %.4f is below cartesian_min_fraction %.4f. "
            "No execution attempted.",
            fraction,
            cartesian_min_fraction_);
          move_group_->clearPoseTargets();
          request_completion_shutdown(
            false, "SAFETY BLOCKED: Cartesian path fraction was below configured minimum.");
          return;
        }
        if (!retime_cartesian_trajectory(trajectory)) {
          move_group_->clearPoseTargets();
          request_completion_shutdown(
            false, "SAFETY BLOCKED: Cartesian trajectory retiming failed.");
          return;
        }
        plan.trajectory_ = trajectory;
        success = true;
      }
      if (success) {
        if (!joint_delta_guard_passed(plan)) {
          move_group_->clearPoseTargets();
          request_completion_shutdown(
            false, "SAFETY BLOCKED: planned joint delta exceeded configured limit.");
          return;
        }
        if (execute_motion_ && allow_standoff_execution_) {
          RCLCPP_WARN(
            get_logger(),
            "Planning succeeded for target_link=%s using planning_mode=%s. Executing one "
            "low-speed standoff dry-run movement now. Suction/gripper hardware is not commanded.",
            target_link_.c_str(),
            planning_mode_.c_str());
          const bool execute_success = static_cast<bool>(move_group_->execute(plan));
          if (execute_success) {
            RCLCPP_INFO(
              get_logger(),
              "Standoff dry-run execution completed successfully.");
            request_completion_shutdown(
              true, "Standoff dry-run execution completed successfully.");
          } else {
            RCLCPP_ERROR(
              get_logger(),
              "Standoff dry-run execution failed or was aborted.");
            request_completion_shutdown(
              false, "Standoff dry-run execution failed or was aborted.");
          }
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Planning succeeded for target_link=%s using planning_mode=%s. "
            "Execution remains disabled.",
            target_link_.c_str(),
            planning_mode_.c_str());
          if (execute_motion_ && !allow_standoff_execution_) {
            request_completion_shutdown(
              false,
              "Execution request blocked because allow_standoff_execution is false.");
          } else {
            request_completion_shutdown(
              true, "Planning succeeded and execution remains disabled.");
          }
        }
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Planning failed for target_link=%s using planning_mode=%s. No execution attempted.",
          target_link_.c_str(),
          planning_mode_.c_str());
        request_completion_shutdown(false, "Planning failed. No execution attempted.");
      }

      if (execute_motion_ && !allow_standoff_execution_) {
        RCLCPP_ERROR(
          get_logger(),
          "Execution request blocked: allow_standoff_execution is false.");
      }
      move_group_->clearPoseTargets();
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "Exception during plan-only contact-box planning: %s", exc.what());
      request_completion_shutdown(false, "Exception during contact-box planning.");
    }
  }

  void handle_locked_contact_pose_sample(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (plan_once_ && has_planned_) {
      return;
    }

    if (locked_contact_pose_source_ == "manual_params") {
      return;
    }

    geometry_msgs::msg::PoseStamped sample = msg;
    const std::string planning_frame = move_group_->getPlanningFrame();
    if (sample.header.frame_id.empty()) {
      sample.header.frame_id = planning_frame;
      RCLCPP_WARN(
        get_logger(),
        "Received contact sample with empty frame_id; using MoveIt planning frame %s.",
        planning_frame.c_str());
    }

    if (sample.header.frame_id != planning_frame) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: contact pose sample frame '%s' does not match MoveIt planning "
        "frame '%s'. Locked Cartesian contact pose must be in base/planning frame.",
        sample.header.frame_id.c_str(),
        planning_frame.c_str());
      has_planned_ = true;
      request_completion_shutdown(
        false, "SAFETY BLOCKED: contact pose sample frame did not match planning frame.");
      return;
    }

    normalize_or_reset_orientation(sample);
    contact_pose_samples_.push_back(sample);
    RCLCPP_INFO(
      get_logger(),
      "Collected locked contact pose sample %zu/%d: position=(%.4f, %.4f, %.4f)",
      contact_pose_samples_.size(),
      contact_pose_sample_count_,
      sample.pose.position.x,
      sample.pose.position.y,
      sample.pose.position.z);

    if (contact_pose_samples_.size() < static_cast<std::size_t>(contact_pose_sample_count_)) {
      return;
    }

    if (!lock_contact_pose_from_live_samples()) {
      has_planned_ = true;
      request_completion_shutdown(
        false, "SAFETY BLOCKED: failed to acquire stable locked contact pose.");
      return;
    }

    plan_locked_contact_pose_if_ready();
  }

  void lock_timer_callback()
  {
    if (has_planned_ || planning_mode_ != "cartesian" || !lock_contact_pose_) {
      if (lock_timer_) {
        lock_timer_->cancel();
      }
      return;
    }

    if (locked_contact_pose_source_ == "manual_params") {
      if (!locked_contact_pose_ready_) {
        lock_contact_pose_from_manual_params();
      }
      plan_locked_contact_pose_if_ready();
      return;
    }

    if (contact_pose_samples_.size() >= static_cast<std::size_t>(contact_pose_sample_count_)) {
      if (locked_contact_pose_ready_) {
        plan_locked_contact_pose_if_ready();
      }
      return;
    }

    maybe_fail_lock_wait_for_timeout("waiting for live /contact_box_pose_debug samples");
  }

  bool plan_locked_contact_pose_if_ready()
  {
    if (!locked_contact_pose_ready_) {
      return false;
    }
    if (!has_latest_joint_state_) {
      maybe_fail_lock_wait_for_timeout("waiting for /joint_states before locked contact pose plan");
      return false;
    }

    if (lock_timer_) {
      lock_timer_->cancel();
    }
    plan_from_contact_pose(locked_contact_pose_);
    return true;
  }

  void maybe_fail_lock_wait_for_timeout(const std::string & waiting_for)
  {
    const double elapsed_sec = (now() - lock_contact_pose_start_time_).seconds();
    if (elapsed_sec <= contact_pose_sample_timeout_sec_) {
      return;
    }

    RCLCPP_ERROR(
      get_logger(),
      "SAFETY BLOCKED: timed out after %.3f seconds while %s. Collected %zu/%d "
      "contact pose samples. Refusing to plan or execute.",
      contact_pose_sample_timeout_sec_,
      waiting_for.c_str(),
      contact_pose_samples_.size(),
      contact_pose_sample_count_);
    has_planned_ = true;
    request_completion_shutdown(false, "SAFETY BLOCKED: timed out waiting for locked contact pose.");
    if (lock_timer_) {
      lock_timer_->cancel();
    }
  }

  bool lock_contact_pose_from_live_samples()
  {
    if (contact_pose_samples_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: no contact pose samples available to lock. Refusing to plan.");
      return false;
    }

    Vector3 average{0.0, 0.0, 0.0};
    for (const auto & sample : contact_pose_samples_) {
      average.x += sample.pose.position.x;
      average.y += sample.pose.position.y;
      average.z += sample.pose.position.z;
    }
    const double sample_count = static_cast<double>(contact_pose_samples_.size());
    average = scale(average, 1.0 / sample_count);

    double max_deviation = 0.0;
    for (const auto & sample : contact_pose_samples_) {
      const Vector3 delta{
        sample.pose.position.x - average.x,
        sample.pose.position.y - average.y,
        sample.pose.position.z - average.z};
      const double deviation = vector_norm(delta);
      if (deviation > max_deviation) {
        max_deviation = deviation;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Locked contact pose sample stability: samples=%zu, average=(%.4f, %.4f, %.4f), "
      "max_deviation=%.4f m, limit=%.4f m",
      contact_pose_samples_.size(),
      average.x, average.y, average.z,
      max_deviation,
      contact_pose_max_spread_m_);

    if (max_deviation > contact_pose_max_spread_m_) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: contact pose sample spread %.4f m exceeds limit %.4f m. "
        "Refusing to plan or execute.",
        max_deviation,
        contact_pose_max_spread_m_);
      return false;
    }

    locked_contact_pose_ = contact_pose_samples_.back();
    locked_contact_pose_.pose.position.x = average.x;
    locked_contact_pose_.pose.position.y = average.y;
    locked_contact_pose_.pose.position.z = average.z;
    locked_contact_pose_ready_ = true;
    RCLCPP_WARN(
      get_logger(),
      "Locked contact pose acquired from live_average: x=%.4f, y=%.4f, z=%.4f. "
      "Future YOLO/contact-pose updates will not change this Cartesian target.",
      locked_contact_pose_.pose.position.x,
      locked_contact_pose_.pose.position.y,
      locked_contact_pose_.pose.position.z);
    return true;
  }

  void lock_contact_pose_from_manual_params()
  {
    locked_contact_pose_.header.frame_id = move_group_->getPlanningFrame();
    locked_contact_pose_.header.stamp = now();
    locked_contact_pose_.pose.position.x = locked_contact_x_;
    locked_contact_pose_.pose.position.y = locked_contact_y_;
    locked_contact_pose_.pose.position.z = locked_contact_z_;
    locked_contact_pose_.pose.orientation.x = 0.0;
    locked_contact_pose_.pose.orientation.y = 0.0;
    locked_contact_pose_.pose.orientation.z = 0.0;
    locked_contact_pose_.pose.orientation.w = 1.0;
    locked_contact_pose_ready_ = true;
    RCLCPP_WARN(
      get_logger(),
      "Locked contact pose acquired from manual_params: x=%.4f, y=%.4f, z=%.4f. "
      "Future YOLO/contact-pose updates will not change this Cartesian target.",
      locked_contact_pose_.pose.position.x,
      locked_contact_pose_.pose.position.y,
      locked_contact_pose_.pose.position.z);
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_joint_state_ = *msg;
    has_latest_joint_state_ = true;
  }

  bool set_start_state_from_latest_joint_state()
  {
    const auto robot_model = move_group_->getRobotModel();
    if (!robot_model) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: failed to read MoveIt robot model for start state. "
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

    std::map<std::string, double> joint_values;
    if (!current_joint_values_from_latest_joint_state(
        joint_model_group->getVariableNames(), joint_values))
    {
      return false;
    }

    moveit::core::RobotState start_state(robot_model);
    start_state.setToDefaultValues();
    start_state.setVariablePositions(joint_values);
    start_state.update(true);
    move_group_->setStartState(start_state);
    RCLCPP_INFO(
      get_logger(),
      "MoveIt planning start state set from cached /joint_states by joint name.");
    return true;
  }

  bool current_joint_values_from_latest_joint_state(
    const std::vector<std::string> & joint_names,
    std::map<std::string, double> & joint_values) const
  {
    joint_values.clear();
    if (!has_latest_joint_state_) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: no cached /joint_states message is available. "
        "Refusing to plan or execute.");
      return false;
    }

    for (const auto & joint_name : joint_names) {
      const auto name_it = std::find(
        latest_joint_state_.name.begin(), latest_joint_state_.name.end(), joint_name);
      if (name_it == latest_joint_state_.name.end()) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: cached /joint_states message does not contain joint '%s'. "
          "Refusing to plan or execute.",
          joint_name.c_str());
        return false;
      }

      const auto index = static_cast<std::size_t>(
        std::distance(latest_joint_state_.name.begin(), name_it));
      if (index >= latest_joint_state_.position.size()) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: cached /joint_states has joint '%s' at index %zu, "
          "but position array size is only %zu. Refusing to plan or execute.",
          joint_name.c_str(),
          index,
          latest_joint_state_.position.size());
        return false;
      }

      const double position = latest_joint_state_.position[index];
      if (!std::isfinite(position)) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: cached /joint_states position for joint '%s' is not finite. "
          "Refusing to plan or execute.",
          joint_name.c_str());
        return false;
      }

      joint_values[joint_name] = position;
    }

    return true;
  }

  bool retime_cartesian_trajectory(moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    log_trajectory_timing("Before Cartesian retiming", trajectory);

    const auto robot_model = move_group_->getRobotModel();
    if (!robot_model) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: failed to read MoveIt robot model for Cartesian retiming. "
        "Refusing execution.");
      return false;
    }

    const auto * joint_model_group = robot_model->getJointModelGroup(planning_group_);
    if (joint_model_group == nullptr) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: planning group '%s' was not found for Cartesian retiming. "
        "Refusing execution.",
        planning_group_.c_str());
      return false;
    }

    std::map<std::string, double> joint_values;
    if (!current_joint_values_from_latest_joint_state(
        joint_model_group->getVariableNames(), joint_values))
    {
      return false;
    }

    moveit::core::RobotState reference_state(robot_model);
    reference_state.setToDefaultValues();
    reference_state.setVariablePositions(joint_values);
    reference_state.update(true);

    robot_trajectory::RobotTrajectory robot_trajectory(robot_model, planning_group_);
    robot_trajectory.setRobotTrajectoryMsg(reference_state, trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    const bool retimed = time_parameterization.computeTimeStamps(
      robot_trajectory, velocity_scaling_, acceleration_scaling_);
    if (!retimed) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: Cartesian trajectory retiming failed with velocity_scaling=%.6f "
        "and acceleration_scaling=%.6f. Refusing execution.",
        velocity_scaling_,
        acceleration_scaling_);
      return false;
    }

    robot_trajectory.getRobotTrajectoryMsg(trajectory);
    RCLCPP_INFO(
      get_logger(),
      "Cartesian trajectory retiming succeeded with velocity_scaling=%.6f and "
      "acceleration_scaling=%.6f.",
      velocity_scaling_,
      acceleration_scaling_);
    log_trajectory_timing("After Cartesian retiming", trajectory);
    return true;
  }

  void log_trajectory_timing(
    const std::string & label,
    const moveit_msgs::msg::RobotTrajectory & trajectory) const
  {
    const auto & points = trajectory.joint_trajectory.points;
    if (points.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "%s: trajectory_points=0, velocity_scaling=%.6f, acceleration_scaling=%.6f",
        label.c_str(),
        velocity_scaling_,
        acceleration_scaling_);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s: trajectory_points=%zu, first_time_from_start=%.6f sec, "
      "last_time_from_start=%.6f sec, velocity_scaling=%.6f, acceleration_scaling=%.6f",
      label.c_str(),
      points.size(),
      duration_to_seconds(points.front().time_from_start),
      duration_to_seconds(points.back().time_from_start),
      velocity_scaling_,
      acceleration_scaling_);
  }

  static double duration_to_seconds(const builtin_interfaces::msg::Duration & duration)
  {
    return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
  }

  void request_completion_shutdown(const bool success, const std::string & reason)
  {
    if (!plan_once_ || !exit_after_completion_) {
      return;
    }

    exit_code_ = success ? 0 : 1;
    if (success) {
      RCLCPP_INFO(
        get_logger(),
        "%s exit_after_completion:=true, shutting down cleanly.",
        reason.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "%s exit_after_completion:=true, shutting down with failure status.",
        reason.c_str());
    }

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void normalize_or_reset_orientation(geometry_msgs::msg::PoseStamped & target_pose)
  {
    const auto & q = target_pose.pose.orientation;
    const double norm = std::sqrt(
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm < 1e-6) {
      RCLCPP_WARN(
        get_logger(),
        "Received invalid target orientation; using identity orientation for plan-only debug.");
      target_pose.pose.orientation.x = 0.0;
      target_pose.pose.orientation.y = 0.0;
      target_pose.pose.orientation.z = 0.0;
      target_pose.pose.orientation.w = 1.0;
      return;
    }

    target_pose.pose.orientation.x /= norm;
    target_pose.pose.orientation.y /= norm;
    target_pose.pose.orientation.z /= norm;
    target_pose.pose.orientation.w /= norm;
  }

  Vector3 rotate_vector_by_quaternion(
    const geometry_msgs::msg::Quaternion & q,
    const Vector3 & vector) const
  {
    const Vector3 q_vec{q.x, q.y, q.z};
    const Vector3 t = scale(cross(q_vec, vector), 2.0);
    return add(add(vector, scale(t, q.w)), cross(q_vec, t));
  }

  static Vector3 cross(const Vector3 & a, const Vector3 & b)
  {
    return Vector3{
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x};
  }

  static Vector3 scale(const Vector3 & vector, const double scalar)
  {
    return Vector3{vector.x * scalar, vector.y * scalar, vector.z * scalar};
  }

  static Vector3 add(const Vector3 & a, const Vector3 & b)
  {
    return Vector3{a.x + b.x, a.y + b.y, a.z + b.z};
  }

  static double vector_norm(const Vector3 & vector)
  {
    return std::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
  }

  std::vector<geometry_msgs::msg::Pose> make_cartesian_waypoints(
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & target_pose,
    const double distance) const
  {
    std::size_t waypoint_count = 1;
    if (distance > 1e-9) {
      waypoint_count = static_cast<std::size_t>(std::ceil(distance / cartesian_eef_step_));
      if (waypoint_count < 1) {
        waypoint_count = 1;
      }
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(waypoint_count);
    for (std::size_t index = 1; index <= waypoint_count; ++index) {
      const double alpha =
        static_cast<double>(index) / static_cast<double>(waypoint_count);
      geometry_msgs::msg::Pose waypoint;
      waypoint.position.x =
        start_pose.position.x + alpha * (target_pose.position.x - start_pose.position.x);
      waypoint.position.y =
        start_pose.position.y + alpha * (target_pose.position.y - start_pose.position.y);
      waypoint.position.z =
        start_pose.position.z + alpha * (target_pose.position.z - start_pose.position.z);
      waypoint.orientation = target_pose.orientation;
      waypoints.push_back(waypoint);
    }

    return waypoints;
  }

  bool joint_delta_guard_passed(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    const auto & joint_trajectory = plan.trajectory_.joint_trajectory;
    const auto & joint_names = joint_trajectory.joint_names;
    const auto & points = joint_trajectory.points;

    if (joint_names.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: generated trajectory has no joint names. Refusing execution.");
      return false;
    }
    if (points.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: generated trajectory has no joint points. Refusing execution.");
      return false;
    }

    const auto & start_positions = points.front().positions;
    const auto & final_positions = points.back().positions;
    if (start_positions.size() != joint_names.size() ||
      final_positions.size() != joint_names.size())
    {
      RCLCPP_ERROR(
        get_logger(),
        "SAFETY BLOCKED: generated trajectory position count does not match joint names "
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
        "Joint delta check: %s start=%.6f rad, final=%.6f rad, delta=%.6f rad, "
        "limit=%.6f rad",
        joint_names[index].c_str(),
        start,
        final,
        delta,
        max_joint_delta_rad_);
      if (!std::isfinite(delta) || delta > max_joint_delta_rad_) {
        RCLCPP_ERROR(
          get_logger(),
          "SAFETY BLOCKED: joint %s delta exceeded limit "
          "(start=%.6f rad, final=%.6f rad, delta=%.6f rad, limit=%.6f rad).",
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

  double sanitize_scaling(double value, const std::string & parameter_name)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "%s must be finite and > 0. Using 0.001 instead.",
        parameter_name.c_str());
      return 0.001;
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

  int sanitize_positive_int(
    int value,
    const std::string & parameter_name,
    const int default_value)
  {
    if (value <= 0) {
      RCLCPP_WARN(
        get_logger(),
        "%s must be > 0. Using %d instead.",
        parameter_name.c_str(),
        default_value);
      return default_value;
    }

    return value;
  }

  double sanitize_positive(
    double value,
    const std::string & parameter_name,
    const double default_value)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "%s must be finite and > 0. Using %.4f instead.",
        parameter_name.c_str(),
        default_value);
      return default_value;
    }

    return value;
  }

  double sanitize_nonnegative(
    double value,
    const std::string & parameter_name,
    const double default_value)
  {
    if (!std::isfinite(value) || value < 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "%s must be finite and >= 0. Using %.4f instead.",
        parameter_name.c_str(),
        default_value);
      return default_value;
    }

    return value;
  }

  double sanitize_fraction(
    double value,
    const std::string & parameter_name,
    const double default_value)
  {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
      RCLCPP_WARN(
        get_logger(),
        "%s must be finite and between 0.0 and 1.0. Using %.4f instead.",
        parameter_name.c_str(),
        default_value);
      return default_value;
    }

    return value;
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr lock_timer_;
  std::string input_topic_;
  std::string planning_group_;
  std::string target_link_;
  std::string planning_mode_{"pose_target"};
  bool execute_motion_{false};
  bool exit_after_completion_{true};
  int exit_code_{0};
  double approach_offset_x_{0.0};
  double approach_offset_y_{0.0};
  double approach_offset_z_{0.0};
  bool use_tool_offset_{true};
  double tool_offset_x_{0.0};
  double tool_offset_y_{0.0};
  double tool_offset_z_{0.263};
  bool use_fixed_orientation_{false};
  double target_qx_{0.0};
  double target_qy_{0.0};
  double target_qz_{0.0};
  double target_qw_{1.0};
  bool plan_once_{true};
  bool has_planned_{false};
  bool use_tool_standoff_{true};
  double standoff_distance_{0.05};
  bool allow_standoff_execution_{false};
  double velocity_scaling_{0.001};
  double acceleration_scaling_{0.001};
  double max_wrist_target_distance_{0.05};
  double cartesian_eef_step_{0.001};
  double cartesian_jump_threshold_{0.0};
  double cartesian_min_fraction_{0.99};
  double max_joint_delta_rad_{0.08};
  bool lock_contact_pose_{true};
  int contact_pose_sample_count_{5};
  double contact_pose_sample_timeout_sec_{5.0};
  double contact_pose_max_spread_m_{0.015};
  std::string locked_contact_pose_source_{"live_average"};
  double locked_contact_x_{0.0};
  double locked_contact_y_{0.0};
  double locked_contact_z_{0.0};
  std::vector<geometry_msgs::msg::PoseStamped> contact_pose_samples_;
  geometry_msgs::msg::PoseStamped locked_contact_pose_;
  bool locked_contact_pose_ready_{false};
  rclcpp::Time lock_contact_pose_start_time_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool has_latest_joint_state_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContactBoxPlanDebug>();
  try {
    node->initialize();
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to initialize MoveIt plan-only node: %s. "
      "Use `ros2 launch moveit_move contact_box_plan_debug.launch.py` so this node "
      "receives robot_description, robot_description_semantic, kinematics, and planning "
      "pipeline parameters. Standalone `ros2 run` does not inherit parameters from /move_group.",
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
  rclcpp::shutdown();
  return node->exit_code();
}
