#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "visibility_control.h"
#include <vector>
#include "libfairino/include/robot.h"
#include <mutex>
#include <memory>
#include <atomic>
#include <thread>
#include <queue>
#include <array>
#include <cstdint>
#include <string>
#include <functional>
#include "fairino_hardware/gripper_config_hardware.hpp"  // For gripper constants
#include "fairino_msgs/srv/fairino_suction_command.hpp"
#include <std_msgs/msg/bool.hpp>
bool gripper_activated_ = false;  // Gripper activation status
constexpr int SUCTION_INDEX = 1;  // Controller labtoolchange.lua uses slot 1 for EVS08 suction.
// Constants (if not in gripper_config.hpp)
constexpr double MAX_FINGER_POS = 0.04;  // Max finger displacement (radians)

// #include "rg75_300_gripper/srv/gripper_command.hpp"


#define CONTROLLER_IP_ADDRESS "192.168.58.2"
struct CommandSnapshot {
    std::array<double, 9> jnt_position_command;
};
namespace fairino_hardware
{

class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  //FAIRINO_HARDWARE_PUBLIC
  //hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  void setup_init_subscribers();
  bool execute_suction_command(int8_t command, std::string &message);
  int call_sdk_with_motion_guard(const std::string &label, const std::function<int()> &sdk_call);
  hardware_interface::return_type record_servoj_skip(const char *reason);
  void reset_servoj_health_counters();
  void handle_suction_command(
    const std::shared_ptr<fairino_msgs::srv::FairinoSuctionCommand::Request> request,
    std::shared_ptr<fairino_msgs::srv::FairinoSuctionCommand::Response> response);
  double _jnt_position_command[9];
  double _jnt_velocity_command[9];
  double _jnt_torque_command[9];
  double _jnt_position_state[9];
  double _last_known_position_state[9];
  double _jnt_velocity_state[9];
  double _jnt_torque_state[9];
  int _control_mode;
  int sdk_pos;
  int prev_sdk_pos;
  int feed_back_pose;
  std::string _controller_ip = CONTROLLER_IP_ADDRESS;

  /* test start */
  // std::shared_ptr<rclcpp::Node> node_;
  // std::mutex sdk_mutex_;
  // rclcpp::Service<rg75_300_gripper::srv::GripperCommand>::SharedPtr gripper_srv_;
  /* test end */
  FRRobot _ptr_robot;
  uint8_t prev_gripper_pos = 20;

  enum class ToolState {
    NONE,
    SUCTION,
    UNKNOWN
  };

  ToolState current_tool_ = ToolState::NONE;
  bool suction_initialized_ = false;
  bool suction_gripper_activated_ = false;
  bool suction_state_unknown_ = false;

  bool finger_gripper_initialized_ = false;
  bool finger_gripper_activated_ = false;
  double global_suction_value = 0.0;
  double global_finger_value = 0.0;
  bool motion_faulted_ = false;
  int consecutive_servoj_failure_count_ = 0;
  int consecutive_servoj_skip_count_ = 0;

  bool initialize_suction();
  void initialize_finger_gripper();
  CommandSnapshot capture_command_snapshot();
  void replay_pending_commands();
  rclcpp::Node::SharedPtr init_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr suction_init_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finger_gripper_init_sub_;
  rclcpp::Service<fairino_msgs::srv::FairinoSuctionCommand>::SharedPtr suction_command_srv_;
  std::mutex suction_command_mutex_;
  std::mutex sdk_command_mutex_;
  std::atomic<bool> tool_command_in_progress_{false};
  std::thread init_spin_thread_;


  // std::unique_ptr<FRRobot> _ptr_robot;
};

} //end namespace


#endif
