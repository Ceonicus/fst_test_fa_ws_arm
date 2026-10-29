#include "fairino_hardware/fairino_hardware_interface.hpp"
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>
#include <array>

bool suction_B_status = false;
std::atomic<bool> suction_inializing{false};
std::thread suction_init_thread_;


std::queue<CommandSnapshot> pending_commands_;   // buffered during init
std::mutex pending_commands_mutex_;              // protect queue access

namespace fairino_hardware
{
    // ------------------------------------------------------------------
    // Helper: capture current command array into a snapshot
    // ------------------------------------------------------------------
    CommandSnapshot FairinoHardwareInterface::capture_command_snapshot()
    {
        CommandSnapshot snap;
        for (int i = 0; i < 9; i++)
            snap.jnt_position_command[i] = _jnt_position_command[i];
        return snap;
    }

    // ------------------------------------------------------------------
    // Helper: replay all buffered snapshots after init completes
    // ------------------------------------------------------------------
    void FairinoHardwareInterface::replay_pending_commands()
    {
        std::queue<CommandSnapshot> local_queue;

        // Drain the shared queue into a local one (minimize lock time)
        {
            std::lock_guard<std::mutex> lock(pending_commands_mutex_);
            std::swap(local_queue, pending_commands_);
        }

        if (local_queue.empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "No buffered commands to replay.");
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "Replaying %zu buffered commands...", local_queue.size());

        while (!local_queue.empty())
        {
            const CommandSnapshot &snap = local_queue.front();

            // --- Arm: ServoJ joints 0-5 ---
            JointPos cmd;
            ExaxisPos extcmd{0, 0, 0, 0};
            for (int j = 0; j < 6; j++)
                cmd.jPos[j] = snap.jnt_position_command[j] / M_PI * 180.0;

            int arm_ret = _ptr_robot.ServoJ(&cmd, &extcmd, 0, 0, 0.008, 0, 0);
            if (arm_ret != 0)
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                            "Replay ServoJ failed (err=%d)", arm_ret);

            // --- Suction: joint 8 ---
            if (suction_gripper_activated_)
            {
                double suction_cmd = snap.jnt_position_command[8];
                int ret = 0;

                if (suction_cmd > 2.0 && suction_cmd < 3.0 && !suction_B_status)
                {
                    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Replay: Suction B ON");
                    ret = _ptr_robot.MoveGripper(SUCTION_INDEX, 0, 70, 60, 6000, 0, 1, 90, 70, 60);
                    suction_B_status = true;
                }
                else if (suction_cmd > 0.0 && suction_cmd < 0.5 && suction_B_status)
                {
                    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Replay: Suction B OFF");
                    ret = _ptr_robot.MoveGripper(SUCTION_INDEX, 0, 70, 60, 6000, 0, 1, 0, 70, 60);
                    suction_B_status = false;
                }

                if (ret != 0)
                    RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                                 "Replay suction MoveGripper failed, error=%d", ret);
            }

            local_queue.pop();
        }

        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Replay complete.");
    }

    // ------------------------------------------------------------------
    // INITIALIZE SUCTION
    // ------------------------------------------------------------------
    void FairinoHardwareInterface::initialize_suction()
    {
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Initializing Suction Tool...");

        _ptr_robot.ResetAllError();

        AxleComParam param(7, 8, 1, 0, 5, 3, 1);
        _ptr_robot.SetAxleCommunicationParam(param);
        rclcpp::sleep_for(std::chrono::seconds(1));

        _ptr_robot.SetAxleLuaEnable(1);
        rclcpp::sleep_for(std::chrono::seconds(1));

        int lua_status = 0;
        _ptr_robot.GetAxleLuaEnableStatus(&lua_status);

        _ptr_robot.SetAxleLuaEnableDeviceType(0, 1, 0);
        rclcpp::sleep_for(std::chrono::seconds(1));

        int func[16] = {1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0};
        _ptr_robot.SetAxleLuaGripperFunc(1, func);
        rclcpp::sleep_for(std::chrono::seconds(1));

        suction_initialized_ = true;
        suction_gripper_activated_ = true;
        finger_gripper_initialized_ = false;
        finger_gripper_activated_ = false;

        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Suction Tool Initialized.");
        // NOTE: replay_pending_commands() and suction_inializing.store(false)
        // are intentionally NOT called here — they happen in on_activate()
        // after "Hardware interface activated successfully!" + 1 sec settle.
    }

    // ------------------------------------------------------------------
    // INITIALIZE FINGER GRIPPER
    // ------------------------------------------------------------------
    void FairinoHardwareInterface::initialize_finger_gripper()
    {
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Initializing Finger Gripper Tool...");

        _ptr_robot.SetReConnectParam(true, 30000, 500);
        rclcpp::sleep_for(std::chrono::seconds(1));

        int returncode = _ptr_robot.SetGripperConfig(
            GRIPPER_COMPANY, GRIPPER_DEVICE, GRIPPER_SOFTWARE, GRIPPER_BUS);

        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "SetGripperConfig return=%d", returncode);

        if (returncode != 0)
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper config failed (err=%d)", returncode);

        rclcpp::sleep_for(std::chrono::seconds(1));

        _ptr_robot.ActGripper(GRIPPER_INDEX, 0);
        rclcpp::sleep_for(std::chrono::seconds(1));

        finger_gripper_activated_ = false;

        for (int retry = 0; retry < ACTIVATION_RETRIES; ++retry)
        {
            returncode = _ptr_robot.ActGripper(GRIPPER_INDEX, 1);
            if (returncode == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Finger gripper activated on retry %d", retry + 1);
                finger_gripper_activated_ = true;
                break;
            }
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Activation failed on retry %d (err=%d)", retry + 1, returncode);
            rclcpp::sleep_for(std::chrono::milliseconds(ACTIVATION_DELAY_MS));
        }

        if (!finger_gripper_activated_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Finger gripper activation failed after %d retries", ACTIVATION_RETRIES);
            return;
        }

        int init_pos = 80;
        returncode = _ptr_robot.MoveGripper(
            GRIPPER_INDEX, init_pos, DEFAULT_VELOCITY, DEFAULT_FORCE,
            DEFAULT_MAX_TIME, DEFAULT_BLOCK, DEFAULT_TYPE,
            DEFAULT_ROT_NUM, DEFAULT_ROT_VEL, DEFAULT_ROT_TORQUE);

        if (returncode != 0)
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Initial finger close failed (err=%d)", returncode);

        finger_gripper_initialized_ = true;
        suction_initialized_ = false;
        suction_gripper_activated_ = false;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Finger Gripper Initialization Complete.");
    }

    // ------------------------------------------------------------------
    // ON INIT
    // ------------------------------------------------------------------
    hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo &sysinfo)
    {
        if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        info_ = sysinfo;
        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
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
    }

    // ------------------------------------------------------------------
    // EXPORT STATE / COMMAND INTERFACES
    // ------------------------------------------------------------------
    std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));
        return command_interfaces;
    }

    // ------------------------------------------------------------------
    // ON ACTIVATE
    // ------------------------------------------------------------------
    hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        using namespace std::chrono_literals;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting hardware interface... please wait...");

        for (int i = 0; i < 6; i++)
        {
            _jnt_position_command[i] = 0.0;
            _jnt_velocity_command[i] = 0.0;
            _jnt_torque_command[i]   = 0.0;
            _jnt_position_state[i]   = 0.0;
            _jnt_velocity_state[i]   = 0.0;
            _jnt_torque_state[i]     = 0.0;
        }
        _jnt_position_command[6] = -MAX_FINGER_POS;
        _jnt_position_command[7] =  MAX_FINGER_POS;
        _jnt_position_command[8] = 0.0;
        _control_mode = 0;

        int returncode = _ptr_robot.RPC(_controller_ip.c_str());
        rclcpp::sleep_for(200ms);
        if (returncode != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "SDK connection failed! Check if port is occupied.");
            suction_inializing.store(false); // clear gate on failure
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "SDK connected successfully!");
        rclcpp::sleep_for(500ms);
        _ptr_robot.ServoMoveStart();

        // If triggered from background suction reinit thread, run suction init now
        if (suction_inializing.load()){
            initialize_suction();
        }
            

        JointPos jntpos;
        returncode = _ptr_robot.GetActualJointPosDegree(0, &jntpos);
        if (returncode == 0)
        {
            for (int j = 0; j < 6; j++)
                _jnt_position_command[j] = jntpos.jPos[j] / 180.0 * M_PI;

            _jnt_position_command[6] = -MAX_FINGER_POS;
            _jnt_position_command[7] =  MAX_FINGER_POS;
            _jnt_position_command[8] = 0.0;

            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Initial command positions: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                        _jnt_position_command[0], _jnt_position_command[1], _jnt_position_command[2],
                        _jnt_position_command[3], _jnt_position_command[4], _jnt_position_command[5]);

            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Hardware interface activated successfully!");

            rclcpp::sleep_for(std::chrono::seconds(1));  // settle

            // Open the gate FIRST — let read() resume getting real positions
            suction_inializing.store(false);

            return hardware_interface::CallbackReturn::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "Failed to read initial joint positions (err=%d). Check communication.", returncode);
            suction_inializing.store(false); // clear gate on failure
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // ------------------------------------------------------------------
    // ON DEACTIVATE
    // ------------------------------------------------------------------
    hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Deactivating hardware interface... please wait...");

        // Wait for background suction init to finish before deactivating
        if (suction_init_thread_.joinable())
        {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Waiting for suction init thread to finish...");
            suction_init_thread_.join();
        }

        _ptr_robot.StopMotion();
        if (finger_gripper_activated_)
        {
            _ptr_robot.ActGripper(GRIPPER_INDEX, 0);
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper deactivated.");
        }
        _ptr_robot.CloseRPC();
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Hardware interface deactivated successfully!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ------------------------------------------------------------------
    // READ
    // ------------------------------------------------------------------
    hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // During suction reinit — feed last known positions so RViz stays correct
        if (suction_inializing.load())
        {
            for (int i = 0; i < 9; i++)
                _jnt_position_state[i] = _last_known_position_state[i];
            return hardware_interface::return_type::OK;
        }

        // Read arm joints (0-5)
        JointPos state_data;
        int returncode = _ptr_robot.GetActualJointPosDegree(1, &state_data);
        if (returncode == 0)
        {
            for (int i = 0; i < 6; i++)
                _jnt_position_state[i] = state_data.jPos[i] / 180.0 * M_PI;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Failed to read arm joint positions (err=%d)", returncode);
        }

        // Read gripper fingers (6-7)
        if (finger_gripper_activated_)
        {
            uint16_t fault = 0;
            uint8_t gripper_pos = 0;
            returncode = _ptr_robot.GetGripperCurPosition(&fault, &gripper_pos);
            gripper_pos = 0;
            if (gripper_pos == 0)
                gripper_pos = sdk_pos;
            else
                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                            "Gripper position read: %d%% (err=%d, fault=%u)", gripper_pos, returncode, fault);

            _jnt_position_state[6] = -0.02 * ((-sdk_pos + 80.0) / 60.0 + 1.0);
            _jnt_position_state[7] =  0.02 * ((-sdk_pos + 80.0) / 60.0 + 1.0);
            prev_gripper_pos = gripper_pos;
        }
        else
        {
            _jnt_position_state[6] = -MAX_FINGER_POS;
            _jnt_position_state[7] =  MAX_FINGER_POS;
        }

        _jnt_position_state[8] = global_suction_value;
        return hardware_interface::return_type::OK;
    }


    // ------------------------------------------------------------------
    // WRITE
    // ------------------------------------------------------------------
    hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (_control_mode == 0)
        {
            if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[8],
                            [](double c) { return !std::isfinite(c); }))
                return hardware_interface::return_type::ERROR;

            global_suction_value = _jnt_position_command[8];

            // -------------------------------------------------------
            // FINGER GRIPPER INIT TRIGGER
            // -------------------------------------------------------
            if (_jnt_position_command[8] < -2.50)
            {
                if (!finger_gripper_initialized_)
                    initialize_finger_gripper();
            }
            // -------------------------------------------------------
            // SUCTION INIT TRIGGER — spawn background thread
            // -------------------------------------------------------
            else if (_jnt_position_command[8] < -1.50 && _jnt_position_command[8] > -2.50 && _last_known_position_state[8] !=_jnt_position_state[8] )
            {
                if (!suction_initialized_ && !suction_inializing.load())
                {
                    // Save last known joint states before going dark
                    for (int i = 0; i < 9; i++)
                        _last_known_position_state[i] = _jnt_position_state[i];

                    suction_inializing.store(true);

                    // Clear any stale buffered commands before starting fresh
                    {
                        std::lock_guard<std::mutex> lock(pending_commands_mutex_);
                        std::queue<CommandSnapshot> empty;
                        std::swap(pending_commands_, empty);
                    }

                    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Spawning suction init thread...");

                    if (suction_init_thread_.joinable())
                        suction_init_thread_.join();

                    suction_init_thread_ = std::thread([this]() {
                        _ptr_robot.StopMotion();
                        _ptr_robot.CloseRPC();
                        rclcpp_lifecycle::State dummy_state;
                        on_activate(dummy_state);
                        // replay_pending_commands() and suction_inializing.store(false)
                        // called inside on_activate() after "activated successfully" + 1sec
                    });
                    suction_init_thread_.detach();
                }
            }
            // -------------------------------------------------------
            // NORMAL OPERATION
            // -------------------------------------------------------
            else
            {
                // Buffer commands while suction reinit is running, then return
                if (suction_inializing.load())
                {
                    std::lock_guard<std::mutex> lock(pending_commands_mutex_);
                    pending_commands_.push(capture_command_snapshot());
                    return hardware_interface::return_type::OK;
                }

                // --- Arm: ServoJ joints 0-5 ---
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

                if (need_servoj)
                {
                    JointPos cmd;
                    ExaxisPos extcmd{0, 0, 0, 0};
                    for (int j = 0; j < 6; j++)
                        cmd.jPos[j] = _jnt_position_command[j] / M_PI * 180.0;

                    int arm_returncode = _ptr_robot.ServoJ(&cmd, &extcmd, 0, 0, 0.008, 0, 0);
                    if (arm_returncode != 0)
                        RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                                    "ServoJ command failed, error code: %d", arm_returncode);
                }

                // --- Finger gripper: joints 6-7 ---
                if (finger_gripper_activated_)
                {
                    double left_cmd  = std::clamp(_jnt_position_command[6], -MAX_FINGER_POS, 0.0);
                    double right_cmd = std::clamp(_jnt_position_command[7],  0.0, MAX_FINGER_POS);
                    double delta_left  = std::abs(left_cmd  - _jnt_position_state[6]);
                    double delta_right = std::abs(right_cmd - _jnt_position_state[7]);

                    if (delta_left >= 1e-4 || delta_right >= 1e-4)
                    {
                        double avg_mag = (std::abs(left_cmd) - 0.02 + right_cmd - 0.02) / (2.0 * 0.02);
                        double val = avg_mag * 100.0;
                        sdk_pos = static_cast<int>(80.0 - (val / 100.0) * (80.0 - 20.0));

                        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                                    "Sending gripper command to position %d%%", sdk_pos);

                        if (std::abs(sdk_pos - prev_sdk_pos) >= 60)
                        {
                            prev_sdk_pos = sdk_pos;
                            int gripper_returncode = _ptr_robot.MoveGripper(
                                GRIPPER_INDEX, prev_sdk_pos, DEFAULT_VELOCITY, DEFAULT_FORCE,
                                DEFAULT_MAX_TIME, DEFAULT_BLOCK, DEFAULT_TYPE,
                                DEFAULT_ROT_NUM, DEFAULT_ROT_VEL, DEFAULT_ROT_TORQUE);

                            if (gripper_returncode == 0)
                                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Gripper move OK");
                            else
                                RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                                             "MoveGripper failed (err=%d) pos=%d vel=%d force=%d",
                                             gripper_returncode, prev_sdk_pos, DEFAULT_VELOCITY, DEFAULT_FORCE);
                        }
                    }
                }

                // --- Suction gripper: joint 8 ---
                if (suction_gripper_activated_)
                {
                    double suction_cmd = _jnt_position_command[8];
                    int ret = 0;

                    // SUCTION B ON
                    if (suction_cmd > 2.0 && suction_cmd < 3.0 && !suction_B_status)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Suction B ON %f", suction_cmd);
                        ret = _ptr_robot.MoveGripper(SUCTION_INDEX, 0, 70, 60, 6000, 0, 1, 90, 70, 60);
                        suction_B_status = true;
                    }
                    // SUCTION B OFF
                    else if (suction_cmd > 0.0 && suction_cmd < 0.5 && suction_B_status)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Suction B OFF");
                        ret = _ptr_robot.MoveGripper(SUCTION_INDEX, 0, 70, 60, 6000, 0, 1, 0, 70, 60);
                        suction_B_status = false;
                    }

                    if (ret != 0)
                        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                                     "Suction MoveGripper failed, error=%d", ret);
                }
            }
        }
        else if (_control_mode == 1)
        {
            if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[5],
                            [](double c) { return !std::isfinite(c); }))
                return hardware_interface::return_type::ERROR;
            // Torque control reserved
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "Command send failed: Unrecognized control mode %d", _control_mode);
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

} // end namespace fairino_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)