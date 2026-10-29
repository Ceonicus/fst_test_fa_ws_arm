#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware
{
    hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo &sysinfo)
    {
        if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        info_ = sysinfo; // info_ is defined in parent class
        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // Command interfaces
            if (joint.command_interfaces.size() != 1)
            { // Enable servoJ
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                             joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Joint '%s' has %s command interface as first. '%s' expected.",
                             joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
            // State interfaces
            if (joint.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interfaces. 1 expected.",
                             joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Joint '%s' has %s state interface as first. '%s' expected.", joint.name.c_str(),
                             joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    } // end on_init

    std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        // Export joint-related state interfaces (position only; velocity/effort reserved)
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));
            // state_interfaces.emplace_back(hardware_interface::StateInterface(
            //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state[i]));
            // state_interfaces.emplace_back(hardware_interface::StateInterface(
            //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));
            // command_interfaces.emplace_back(hardware_interface::CommandInterface(  // Reserved for torque control
            //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command[i]));
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        using namespace std::chrono_literals;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting hardware interface... please wait...");
        // Initialize variables
        // _ptr_robot = std::make_unique<FRRobot>();  // Create robot instance (shared for arm and gripper, same IP)
        for (int i = 0; i < 6; i++)
        { // Arm joints only
            _jnt_position_command[i] = 0.0;
            _jnt_velocity_command[i] = 0.0;
            _jnt_torque_command[i] = 0.0;
            _jnt_position_state[i] = 0.0;
            _jnt_velocity_state[i] = 0.0;
            _jnt_torque_state[i] = 0.0;
        }
        _jnt_position_command[6] = -MAX_FINGER_POS;              // Closed left finger
        _jnt_position_command[7] = MAX_FINGER_POS;               // Closed right finger
        _control_mode = 0;                                       // Default: position control (0=position, 1=torque, 2=velocity)
        int returncode = _ptr_robot.RPC(_controller_ip.c_str()); // Connect (same IP for arm/gripper)
        rclcpp::sleep_for(200ms);
        if (returncode != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "SDK connection failed! Check if port is occupied.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "SDK connected successfully!");
        }
        rclcpp::sleep_for(500ms);
        // Gripper: Configure and activate with retries (using shared connection)
        _ptr_robot.SetReConnectParam(true, 30000, 500);
        rclcpp::sleep_for(1s);
        returncode = _ptr_robot.SetGripperConfig(GRIPPER_COMPANY, GRIPPER_DEVICE, GRIPPER_SOFTWARE, GRIPPER_BUS);
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "SetGripperConfig return=%d", returncode);
        if (returncode != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper config failed (err=%d). Verify RG75-300 params.", returncode);
        }
        rclcpp::sleep_for(1s);
        // Reset gripper
        _ptr_robot.ActGripper(GRIPPER_INDEX, 0);
        rclcpp::sleep_for(1s);
        // Activate gripper with retries
        gripper_activated_ = false;
        for (int retry = 0; retry < ACTIVATION_RETRIES; ++retry)
        {
            returncode = _ptr_robot.ActGripper(GRIPPER_INDEX, 1);
            if (returncode == 0)
            {
                // Set activated on success (status check commented if unavailable)
                // int status = _ptr_robot.GetGripperActivateStatus(GRIPPER_INDEX);
                // if (status == 1) {
                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper activated on retry %d", retry + 1);
                gripper_activated_ = true;
                break;
                // }
            }
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper activation failed on retry %d (err=%d). Retrying...", retry + 1, returncode);
            rclcpp::sleep_for(std::chrono::milliseconds(ACTIVATION_DELAY_MS));
        }
        if (gripper_activated_)
        {
            // Initial closed move (blocking)
            int init_pos = 80; // Min 20% (partial closed limit)
            returncode = _ptr_robot.MoveGripper(GRIPPER_INDEX, init_pos, DEFAULT_VELOCITY, DEFAULT_FORCE,
                                                DEFAULT_MAX_TIME, 0 /*blocking*/, DEFAULT_TYPE, DEFAULT_ROT_NUM, DEFAULT_ROT_VEL, DEFAULT_ROT_TORQUE);
            if (returncode != 0)
            {
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Initial gripper close failed (err=%d)", returncode);
            }
            rclcpp::sleep_for(1s);
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper activation failed after %d retries. Continuing without gripper.", ACTIVATION_RETRIES);
        }
        // Arm init: Sync current positions
        JointPos jntpos;
        returncode = _ptr_robot.GetActualJointPosDegree(0, &jntpos);
        if (returncode == 0)
        {
            for (int j = 0; j < 6; j++)
            {
                _jnt_position_command[j] = jntpos.jPos[j] / 180.0 * M_PI;
            }
            _jnt_position_command[6] = -MAX_FINGER_POS;
            _jnt_position_command[7] = MAX_FINGER_POS;
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Initial command positions: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                        _jnt_position_command[0], _jnt_position_command[1], _jnt_position_command[2],
                        _jnt_position_command[3], _jnt_position_command[4], _jnt_position_command[5]);
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Hardware interface activated successfully!");
            return hardware_interface::CallbackReturn::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to read initial joint positions (err=%d). Check communication.", returncode);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Deactivating hardware interface... please wait...");
        _ptr_robot.StopMotion(); // Stop arm motion
        if (gripper_activated_)
        {
            _ptr_robot.ActGripper(GRIPPER_INDEX, 0); // Deactivate gripper
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper deactivated.");
        }
        _ptr_robot.CloseRPC(); // Close connection (single instance)
        // _ptr_robot.release();
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Hardware interface deactivated successfully!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // Read arm joints (0-5) from feedback
        JointPos state_data;
        int returncode = _ptr_robot.GetActualJointPosDegree(1, &state_data);
        if (returncode == 0)
        {
            for (int i = 0; i < 6; i++)
            {
                _jnt_position_state[i] = state_data.jPos[i] / 180.0 * M_PI; // Convert to radians
            }
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to read arm joint positions (err=%d)", returncode);
        }
        // Read gripper fingers (6-7)
        if (gripper_activated_)
        {
            uint16_t fault = 0;
            uint8_t gripper_pos = 0;
            returncode = _ptr_robot.GetGripperCurPosition(&fault, &gripper_pos);
            // if (returncode == 0 && fault == 0) {
            gripper_pos = 0;
            if (gripper_pos == 0)
            {
                // RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper position read: %d%% (err=%d, fault=%u)", gripper_pos, returncode, fault);
                gripper_pos = sdk_pos;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper position read: %d%% (err=%d, fault=%u)", gripper_pos, returncode, fault);
            }

            _jnt_position_state[6] = -0.02 * ((-sdk_pos + 80.0) / 60.0 + 1.0);
            ; // Left finger
            _jnt_position_state[7] = 0.02 * ((-sdk_pos + 80.0) / 60.0 + 1.0);
            ; // Right finger
            prev_gripper_pos = gripper_pos;
            // } else {
            //     RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to read gripper position (err=%d, fault=%u)", returncode, fault);
            // }
        }
        else
        {
            // Deactivated: Report closed
            _jnt_position_state[6] = -MAX_FINGER_POS;
            _jnt_position_state[7] = MAX_FINGER_POS;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (_control_mode == 0)
        { // Position control mode
            // Validate arm commands
            if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[5],
                            [](double c)
                            { return !std::isfinite(c); }))
            {
                return hardware_interface::return_type::ERROR;
            }
            const double THRESH = 0.01;
            bool need_servoj = false;

            for (int j = 0; j < 6; j++)
            {
                if (std::abs(_jnt_position_command[j] - _jnt_position_state[j]) > THRESH)
                {
                    need_servoj = true;
                    break;
                }
            }
            // Arm: Send ServoJ for joints 0-5 with valid params to avoid helix start error (code 14)
            // if(need_servoj) {
            // RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Sending ServoJ command to arm.");
            JointPos cmd;
            ExaxisPos extcmd{0, 0, 0, 0};
            for (int j = 0; j < 6; j++)
            {
                cmd.jPos[j] = _jnt_position_command[j] / M_PI * 180.0; // Convert to degrees
            }
            int arm_returncode = _ptr_robot.ServoJ(&cmd, &extcmd, 0, 0, 0.008, 0, 0);

            if (arm_returncode != 0)
            {
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ command failed, error code: %d (helix start point check: ensure valid trajectory)", arm_returncode);
            }
            // return hardware_interface::return_type::OK; // No significant change
            // }

            // Gripper: Send MoveGripper for fingers 6-7 (non-blocking to avoid timeout in loop, potential cause of 73)
            if (gripper_activated_)
            {
                double left_cmd = std::clamp(_jnt_position_command[6], -MAX_FINGER_POS, 0.0);
                double right_cmd = std::clamp(_jnt_position_command[7], 0.0, MAX_FINGER_POS);
                double delta_left = std::abs(left_cmd - _jnt_position_state[6]);
                double delta_right = std::abs(right_cmd - _jnt_position_state[7]);

                // static bool diff = true;
                if (delta_left >= 1e-4 || delta_right >= 1e-4)
                { // Small threshold
                    // diff = false;
                    // Symmetric average for SDK (0=closed, 100=open)
                    double avg_mag = (std::abs(left_cmd) - 0.02 + right_cmd - 0.02) / (2.0 * 0.02);
                    double val = avg_mag * 100.0; // convert to 0–100
                    sdk_pos = static_cast<int>(80.0 - (val / 100.0) * (80.0 - 20.0));
                    // Non-blocking for real-time control (flag=1; blocking may cause timeout err 73)
                    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Sending gripper command to position %d%%", sdk_pos);
                    if (std::abs(sdk_pos - prev_sdk_pos) >= 60)
                    {
                        prev_sdk_pos = sdk_pos;
                        int gripper_returncode = _ptr_robot.MoveGripper(GRIPPER_INDEX, prev_sdk_pos, DEFAULT_VELOCITY, DEFAULT_FORCE,
                                                                        DEFAULT_MAX_TIME, DEFAULT_BLOCK, DEFAULT_TYPE, DEFAULT_ROT_NUM, DEFAULT_ROT_VEL, DEFAULT_ROT_TORQUE);

                        if (gripper_returncode == 0)
                        {
                            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper move OK ");
                        }
                        else
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "MoveGripper failed (err=%d) with pos=%d vel=%d force=%d",
                                         gripper_returncode, prev_sdk_pos, DEFAULT_VELOCITY, DEFAULT_FORCE);
                        }
                    }
                }
            }
        }
        else if (_control_mode == 1)
        { // Torque control mode (reserved)
            if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[5],
                            [](double c)
                            { return !std::isfinite(c); }))
            {
                return hardware_interface::return_type::ERROR;
            }
            // Implement torque write if enabled
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Command send failed: Unrecognized control mode %d", _control_mode);
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }
} // end namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)