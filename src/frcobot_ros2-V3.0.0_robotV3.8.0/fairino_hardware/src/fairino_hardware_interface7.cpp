#include "fairino_hardware/fairino_hardware_interface.hpp"
#include <atomic>
#include <chrono>
#include <thread>

bool suction_A_status = false;
bool suction_B_status = false;
std::atomic<bool> suction_inializing{false};
std::atomic<bool> finger_gripper_initalizing{false};
std::thread suction_init_thread_;
std::thread finger_gripper_init_thread_;

namespace fairino_hardware
{
    static constexpr int8_t SUCTION_COMMAND_STATUS = 0;
    static constexpr int8_t SUCTION_COMMAND_A_ON = 1;
    static constexpr int8_t SUCTION_COMMAND_A_OFF = 2;
    static constexpr int8_t SUCTION_COMMAND_B_ON = 3;
    static constexpr int8_t SUCTION_COMMAND_B_OFF = 4;
    static constexpr int8_t SUCTION_COMMAND_ALL_ON = 5;
    static constexpr int8_t SUCTION_COMMAND_ALL_OFF = 6;
    static constexpr uint8_t FAIRINO_GRIPPER_NON_BLOCKING = 1;
    static constexpr int EVS08_CHANNEL_A_ON_POSITION = 100;
    static constexpr int EVS08_CHANNEL_A_OFF_POSITION = 0;
    static constexpr int EVS08_CHANNEL_B_OFF_ROT_NUM = 0;
    static constexpr int EVS08_VACUUM_MAX = 70;
    static constexpr int EVS08_VACUUM_MIN = 60;
    static constexpr int EVS08_MAX_TIME_MS = 6000;
    static constexpr int EVS08_ROTATING_GRIPPER_TYPE = 1;
    static constexpr int SERVOJ_FAILURE_THRESHOLD = 5;
    static constexpr int SERVOJ_SKIP_THRESHOLD = 100;

    class ScopedToolCommandFlag
    {
    public:
        explicit ScopedToolCommandFlag(std::atomic<bool> &flag)
            : flag_(flag)
        {
            flag_.store(true);
        }

        ~ScopedToolCommandFlag()
        {
            flag_.store(false);
        }

        ScopedToolCommandFlag(const ScopedToolCommandFlag &) = delete;
        ScopedToolCommandFlag &operator=(const ScopedToolCommandFlag &) = delete;

    private:
        std::atomic<bool> &flag_;
    };

    std::string describe_fairino_error(int code)
    {
        switch (code)
        {
            case 14:
                return "ERR_EXECUTION_FAILED: the command failed to be executed";
            case 73:
                return "ERR_GRIPPER_MOTION: the jaw movement is wrong";
            default:
                return "unknown FAIRINO SDK error";
        }
    }

    std::string suction_command_name(int8_t command)
    {
        switch (command)
        {
            case SUCTION_COMMAND_A_ON:
                return "A_ON";
            case SUCTION_COMMAND_A_OFF:
                return "A_OFF";
            case SUCTION_COMMAND_B_ON:
                return "B_ON";
            case SUCTION_COMMAND_B_OFF:
                return "B_OFF";
            case SUCTION_COMMAND_ALL_ON:
                return "ALL_ON";
            case SUCTION_COMMAND_ALL_OFF:
                return "ALL_OFF";
            case SUCTION_COMMAND_STATUS:
                return "STATUS";
            default:
                return "UNKNOWN";
        }
    }

    // ------------------------------------------------------------------
    // INITIALIZE SUCTION
    // ------------------------------------------------------------------
    int FairinoHardwareInterface::call_sdk_with_motion_guard(
        const std::string &label,
        const std::function<int()> &sdk_call)
    {
        ScopedToolCommandFlag tool_command_guard(tool_command_in_progress_);
        std::unique_lock<std::mutex> sdk_lock(sdk_command_mutex_);

        const auto start = std::chrono::steady_clock::now();
        const int returncode = sdk_call();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();

        if (returncode == 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "%s returned OK in %ld ms", label.c_str(), elapsed_ms);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "%s failed with error code %d (%s) after %ld ms",
                         label.c_str(), returncode,
                         describe_fairino_error(returncode).c_str(), elapsed_ms);
        }

        if (elapsed_ms > 250)
        {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                        "%s held the FAIRINO SDK for %ld ms; keep tool calls short to preserve ServoJ health.",
                        label.c_str(), elapsed_ms);
        }

        return returncode;
    }

    bool FairinoHardwareInterface::initialize_suction()
    {
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Initializing Suction Tool...");
        suction_initialized_ = false;
        suction_gripper_activated_ = false;
        current_tool_ = ToolState::UNKNOWN;

        if (motion_faulted_)
        {
            suction_state_unknown_ = true;
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "Suction preparation rejected because arm motion is faulted. Restart/recover the robot stack before preparing suction.");
            return false;
        }

        auto run_step = [&](const std::string &label, const std::function<int()> &sdk_call) -> bool
        {
            const int returncode = call_sdk_with_motion_guard(label, sdk_call);
            if (returncode != 0)
            {
                suction_initialized_ = false;
                suction_gripper_activated_ = false;
                suction_state_unknown_ = true;
                current_tool_ = ToolState::UNKNOWN;
                return false;
            }
            return true;
        };

        if (!run_step("ResetAllError", [this]() { return _ptr_robot.ResetAllError(); }))
            return false;

        // This suction path is the FAIRINO axle-Lua peripheral/gripper protocol,
        // not simple flange ToolDO. Current hardware testing reports that
        // prepare + A_ON + ALL_OFF can leave the controller accepting motion
        // commands while the physical robot does not move until controller power
        // cycle. Do not add recovery calls here without SDK/vendor evidence.
        AxleComParam param(7, 8, 1, 0, 5, 3, 1);
        if (!run_step("SetAxleCommunicationParam", [this, param]() { return _ptr_robot.SetAxleCommunicationParam(param); }))
            return false;
        rclcpp::sleep_for(std::chrono::seconds(1));

        if (!run_step("SetAxleLuaEnable(1)", [this]() { return _ptr_robot.SetAxleLuaEnable(1); }))
            return false;
        rclcpp::sleep_for(std::chrono::seconds(1));

        int lua_status = 0;
        if (!run_step("GetAxleLuaEnableStatus", [this, &lua_status]() { return _ptr_robot.GetAxleLuaEnableStatus(&lua_status); }))
            return false;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "Suction axle Lua enable status=%d", lua_status);

        if (!run_step("SetAxleLuaEnableDeviceType(0,1,0)", [this]() { return _ptr_robot.SetAxleLuaEnableDeviceType(0, 1, 0); }))
            return false;
        rclcpp::sleep_for(std::chrono::seconds(1));

        int func[16] = {1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0};
        if (!run_step("SetAxleLuaGripperFunc", [this, &func]() { return _ptr_robot.SetAxleLuaGripperFunc(SUCTION_INDEX, func); }))
            return false;
        rclcpp::sleep_for(std::chrono::seconds(1));

        const int activate_ret = call_sdk_with_motion_guard(
            "ActGripper(suction,1)",
            [this]() { return _ptr_robot.ActGripper(SUCTION_INDEX, 1); });
        if (activate_ret != 0)
        {
            suction_initialized_ = false;
            suction_gripper_activated_ = false;
            suction_state_unknown_ = true;
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "Suction ActGripper(%d,1) failed with error code %d (%s); suction remains uninitialized.",
                         SUCTION_INDEX, activate_ret, describe_fairino_error(activate_ret).c_str());
            return false;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        suction_initialized_ = true;
        suction_gripper_activated_ = true;
        suction_state_unknown_ = false;
        current_tool_ = ToolState::SUCTION;
        suction_A_status = false;
        suction_B_status = false;
        finger_gripper_initialized_ = false;
        finger_gripper_activated_ = false;

        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Suction Tool Initialized.");
        return true;
    }

    // ------------------------------------------------------------------
    // SUCTION COMMAND SERVICE
    // ------------------------------------------------------------------
    bool FairinoHardwareInterface::execute_suction_command(int8_t command, std::string &message)
    {
        if (command != SUCTION_COMMAND_STATUS &&
            command != SUCTION_COMMAND_A_ON &&
            command != SUCTION_COMMAND_A_OFF &&
            command != SUCTION_COMMAND_B_ON &&
            command != SUCTION_COMMAND_B_OFF &&
            command != SUCTION_COMMAND_ALL_ON &&
            command != SUCTION_COMMAND_ALL_OFF)
        {
            message = "Unsupported suction command. Project commands: 0=STATUS, 1=A_ON/top cups only, 2=A_OFF stop-all, 6=ALL_OFF stop-all.";
            return false;
        }

        std::lock_guard<std::mutex> lock(suction_command_mutex_);

        if (command == SUCTION_COMMAND_STATUS)
        {
            const bool preparing = suction_inializing.load() || finger_gripper_initalizing.load();
            const bool prepared = suction_initialized_ &&
                                  suction_gripper_activated_ &&
                                  current_tool_ == ToolState::SUCTION &&
                                  !suction_state_unknown_ &&
                                  !preparing;
            message = "Suction status: prepared=" + std::string(prepared ? "true" : "false") +
                      ", preparing=" + std::string(preparing ? "true" : "false") +
                      ", state_unknown=" + std::string(suction_state_unknown_ ? "true" : "false") +
                      ", A=" + std::string(suction_A_status ? "ON" : "OFF") +
                      ", B=" + std::string(suction_B_status ? "ON" : "OFF") +
                      ". STATUS is read-only and did not command suction hardware.";
            return prepared;
        }

        if (command == SUCTION_COMMAND_B_ON ||
            command == SUCTION_COMMAND_B_OFF ||
            command == SUCTION_COMMAND_ALL_ON)
        {
            message = "Command " + suction_command_name(command) +
                      " is disabled for this project. The workflow uses suction A/top two cups only; use A_ON for top cups or ALL_OFF for EVS08 stop-all/break-vacuum release.";
            return false;
        }

        if (suction_inializing.load() || finger_gripper_initalizing.load())
        {
            message = "Tool preparation is in progress; suction command rejected.";
            return false;
        }

        auto mark_suction_unknown = [&](const std::string &detail)
        {
            suction_state_unknown_ = true;
            suction_initialized_ = false;
            suction_gripper_activated_ = false;
            current_tool_ = ToolState::UNKNOWN;
            message = detail + " Physical suction state is unknown; run ALL_OFF EVS08 stop-all/break-vacuum or recover manually before any further ON command.";
        };

        auto send_evs08_project_state = [&](const std::string &target, bool desired_a_on) -> bool
        {
            const int channel_a_position = desired_a_on ? EVS08_CHANNEL_A_ON_POSITION : EVS08_CHANNEL_A_OFF_POSITION;
            const int channel_b_rot_num = EVS08_CHANNEL_B_OFF_ROT_NUM;

            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "%s EVS08 project-state command: A/top=%s B/bottom=OFF. This project intentionally uses only suction A/top two cups.",
                        target.c_str(), desired_a_on ? "ON" : "OFF");
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Normal release uses MoveGripper stop-all/break-vacuum with B forced OFF; ActGripper(%d,0) is intentionally avoided because it may deactivate/reset the peripheral slot.",
                        SUCTION_INDEX);

            const int move_ret = call_sdk_with_motion_guard(
                target + " EVS08 MoveGripper full-state",
                [this, channel_a_position, channel_b_rot_num]() {
                    return _ptr_robot.MoveGripper(
                        SUCTION_INDEX,
                        channel_a_position, EVS08_VACUUM_MAX, EVS08_VACUUM_MIN, EVS08_MAX_TIME_MS,
                        FAIRINO_GRIPPER_NON_BLOCKING, EVS08_ROTATING_GRIPPER_TYPE,
                        channel_b_rot_num, EVS08_VACUUM_MAX, EVS08_VACUUM_MIN);
                });

            if (move_ret != 0)
            {
                mark_suction_unknown(target + " EVS08 MoveGripper full-state failed with error code " +
                                     std::to_string(move_ret) + " (" +
                                     describe_fairino_error(move_ret) + ").");
                return false;
            }

            suction_A_status = desired_a_on;
            suction_B_status = false;
            suction_state_unknown_ = false;
            suction_initialized_ = true;
            suction_gripper_activated_ = true;
            current_tool_ = ToolState::SUCTION;

            message = target + " EVS08 project-state command sent successfully: A/top=" +
                      std::string(desired_a_on ? "ON" : "OFF") + ", B/bottom=OFF" +
                      ". ActGripper(" + std::to_string(SUCTION_INDEX) +
                      ",0) was not called for normal release.";
            return true;
        };

        if (command == SUCTION_COMMAND_ALL_OFF || command == SUCTION_COMMAND_A_OFF)
            return send_evs08_project_state("ALL_OFF", false);

        if (motion_faulted_)
        {
            message = "Arm motion is faulted; only ALL_OFF recovery is allowed. Restart/recover the robot stack before further suction ON commands.";
            return false;
        }

        if (suction_state_unknown_ || current_tool_ == ToolState::UNKNOWN)
        {
            message = "Suction state is UNKNOWN; only ALL_OFF EVS08 stop-all/break-vacuum release is allowed before any further suction command.";
            return false;
        }

        if (!suction_initialized_ || !suction_gripper_activated_ || current_tool_ != ToolState::SUCTION)
        {
            message = "Suction is not prepared; run /initialize_suction successfully before EVS08 suction commands.";
            return false;
        }

        std::string target = suction_command_name(command);

        switch (command)
        {
            case SUCTION_COMMAND_A_ON:
                return send_evs08_project_state(target, true);
            default:
                message = "Unsupported suction command state.";
                return false;
        }
    }

    void FairinoHardwareInterface::handle_suction_command(
        const std::shared_ptr<fairino_msgs::srv::FairinoSuctionCommand::Request> request,
        std::shared_ptr<fairino_msgs::srv::FairinoSuctionCommand::Response> response)
    {
        response->success = execute_suction_command(request->command, response->message);

        if (request->command == SUCTION_COMMAND_STATUS)
        {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Suction STATUS queried: success=%s, %s",
                        response->success ? "true" : "false",
                        response->message.c_str());
            return;
        }

        if (response->success)
        {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Suction command %d (%s) accepted: %s",
                        request->command,
                        suction_command_name(request->command).c_str(),
                        response->message.c_str());
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Suction command %d (%s) rejected: %s",
                        request->command,
                        suction_command_name(request->command).c_str(),
                        response->message.c_str());
        }
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
    // SETUP INIT TOPIC SUBSCRIBERS
    // Called once after first successful activation.
    // ------------------------------------------------------------------
    void FairinoHardwareInterface::setup_init_subscribers()
    {
        if (init_node_)
            return;  // already set up

        init_node_ = std::make_shared<rclcpp::Node>("fairino_init_listener");

        suction_init_sub_ = init_node_->create_subscription<std_msgs::msg::Bool>(
            "/initialize_suction", rclcpp::QoS(1).reliable(),
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                if (!msg->data) return;
                if (suction_initialized_ && current_tool_ == ToolState::SUCTION && !suction_state_unknown_)
                {
                    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                                "/initialize_suction ignored: suction is already prepared.");
                    return;
                }
                if (suction_inializing.load())
                {
                    RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                                "/initialize_suction ignored: suction preparation is already in progress.");
                    return;
                }

                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                            "Received /initialize_suction: preparing suction on the active SDK connection.");

                for (int i = 0; i < 9; i++)
                    _last_known_position_state[i] = _jnt_position_state[i];

                suction_inializing.store(true);

                if (suction_init_thread_.joinable())
                    suction_init_thread_.join();

                suction_init_thread_ = std::thread([this]()
                {
                    const bool prepared = initialize_suction();
                    if (!prepared)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                                     "Suction preparation failed. Suction state is UNKNOWN; use ALL_OFF/manual recovery before ON commands.");
                    }
                    suction_inializing.store(false);
                });
            });

        finger_gripper_init_sub_ = init_node_->create_subscription<std_msgs::msg::Bool>(
            "/initialize_finger_gripper", rclcpp::QoS(1).reliable(),
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                if (!msg->data) return;
                if (finger_gripper_initialized_ || finger_gripper_initalizing.load()) return;

                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                            "Received /initialize_finger_gripper — spawning init thread...");

                for (int i = 0; i < 9; i++)
                    _last_known_position_state[i] = _jnt_position_state[i];

                finger_gripper_initalizing.store(true);

                if (finger_gripper_init_thread_.joinable())
                    finger_gripper_init_thread_.join();

                finger_gripper_init_thread_ = std::thread([this]()
                {
                    _ptr_robot.StopMotion();
                    _ptr_robot.CloseRPC();
                    rclcpp_lifecycle::State dummy_state;
                    on_activate(dummy_state);
                });
                finger_gripper_init_thread_.detach();
            });

        suction_command_srv_ = init_node_->create_service<fairino_msgs::srv::FairinoSuctionCommand>(
            "/fairino_suction_command",
            [this](
                const std::shared_ptr<fairino_msgs::srv::FairinoSuctionCommand::Request> request,
                std::shared_ptr<fairino_msgs::srv::FairinoSuctionCommand::Response> response)
            {
                handle_suction_command(request, response);
            });

        // Spin the listener node in its own thread
        init_spin_thread_ = std::thread([this]()
        {
            rclcpp::spin(init_node_);
        });
        init_spin_thread_.detach();

        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                    "Init subscribers ready: /initialize_suction | /initialize_finger_gripper | /fairino_suction_command");
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
        (void)previous_state;
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
        current_tool_ = ToolState::NONE;
        suction_initialized_ = false;
        suction_gripper_activated_ = false;
        suction_state_unknown_ = false;
        motion_faulted_ = false;
        reset_servoj_health_counters();

        int returncode = _ptr_robot.RPC(_controller_ip.c_str());
        rclcpp::sleep_for(200ms);
        if (returncode != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "SDK connection failed! Check if port is occupied.");
            suction_inializing.store(false);
            finger_gripper_initalizing.store(false);
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "SDK connected successfully!");
        rclcpp::sleep_for(500ms);
        returncode = _ptr_robot.ServoMoveStart();
        if (returncode != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "ServoMoveStart failed (err=%d). Arm control is not healthy.", returncode);
            suction_inializing.store(false);
            finger_gripper_initalizing.store(false);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (finger_gripper_initalizing.load())
            initialize_finger_gripper();

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

            suction_inializing.store(false);
            finger_gripper_initalizing.store(false);

            // Start listening for init topic commands (first activation only)
            setup_init_subscribers();

            return hardware_interface::CallbackReturn::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                         "Failed to read initial joint positions (err=%d). Check communication.", returncode);
            suction_inializing.store(false);
            finger_gripper_initalizing.store(false);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // ------------------------------------------------------------------
    // ON DEACTIVATE
    // ------------------------------------------------------------------
    hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Deactivating hardware interface... please wait...");

        // Shutdown the init listener node cleanly
        if (init_node_)
        {
            init_node_->get_node_base_interface()->get_context()->shutdown("deactivating hardware interface");
            init_node_.reset();
        }

        if (suction_init_thread_.joinable())
        {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Waiting for suction init thread to finish...");
            suction_init_thread_.join();
        }
        if (finger_gripper_init_thread_.joinable())
        {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Waiting for finger gripper init thread to finish...");
            finger_gripper_init_thread_.join();
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

    void FairinoHardwareInterface::reset_servoj_health_counters()
    {
        consecutive_servoj_failure_count_ = 0;
        consecutive_servoj_skip_count_ = 0;
    }

    hardware_interface::return_type FairinoHardwareInterface::record_servoj_skip(const char *reason)
    {
        ++consecutive_servoj_skip_count_;

        if (consecutive_servoj_skip_count_ == 1 || consecutive_servoj_skip_count_ % 25 == 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                        "ServoJ skipped while %s (consecutive skips=%d)",
                        reason, consecutive_servoj_skip_count_);
        }

        if (consecutive_servoj_skip_count_ >= SERVOJ_SKIP_THRESHOLD)
        {
            if (!motion_faulted_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                             "ServoJ skipped for %d consecutive write cycles while %s; physical robot may not have moved; recovery required.",
                             consecutive_servoj_skip_count_, reason);
            }
            motion_faulted_ = true;
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    // ------------------------------------------------------------------
    // READ
    // ------------------------------------------------------------------
    hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        // Legacy finger-gripper initialization can still pause SDK reads. Suction preparation
        // must not freeze arm feedback except during the short guarded SDK calls below.
        if (finger_gripper_initalizing.load())
        {
            for (int i = 0; i < 9; i++)
                _jnt_position_state[i] = _last_known_position_state[i];
            return hardware_interface::return_type::OK;
        }

        if (tool_command_in_progress_.load())
            return hardware_interface::return_type::OK;

        std::unique_lock<std::mutex> sdk_lock(sdk_command_mutex_, std::try_to_lock);
        if (!sdk_lock.owns_lock() || tool_command_in_progress_.load())
            return hardware_interface::return_type::OK;

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
        (void)time;
        (void)period;
        if (_control_mode == 0)
        {
            if (motion_faulted_)
                return hardware_interface::return_type::ERROR;

            if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[8],
                            [](double c) { return !std::isfinite(c); }))
                return hardware_interface::return_type::ERROR;

            global_suction_value = _jnt_position_command[8];

            if (finger_gripper_initalizing.load())
                return record_servoj_skip("legacy finger-gripper initialization is in progress");

            if (tool_command_in_progress_.load())
                return record_servoj_skip("a guarded SDK tool command is in progress");

            std::unique_lock<std::mutex> sdk_lock(sdk_command_mutex_, std::try_to_lock);
            if (!sdk_lock.owns_lock() || tool_command_in_progress_.load())
                return record_servoj_skip("the FAIRINO SDK mutex is busy");

            // --- Arm: ServoJ joints 0-5 ---
            JointPos cmd;
            ExaxisPos extcmd{0, 0, 0, 0};
            for (int j = 0; j < 6; j++)
                cmd.jPos[j] = _jnt_position_command[j] / M_PI * 180.0;

            int arm_returncode = _ptr_robot.ServoJ(&cmd, &extcmd, 0, 0, 0.008, 0, 0);
            if (arm_returncode != 0)
            {
                ++consecutive_servoj_failure_count_;
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                            "ServoJ command failed, error code: %d (consecutive failures=%d)",
                            arm_returncode, consecutive_servoj_failure_count_);

                if (consecutive_servoj_failure_count_ >= SERVOJ_FAILURE_THRESHOLD)
                {
                    motion_faulted_ = true;
                    RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                                 "ServoJ failed repeatedly; physical robot may not have moved; recovery required.");
                    return hardware_interface::return_type::ERROR;
                }
            }
            else
            {
                reset_servoj_health_counters();
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

            sdk_lock.unlock();

            // Legacy joint-8 suction commands are disabled. Suction must be commanded through
            // /fairino_suction_command so tool SDK calls never run from write().
            static bool warned_legacy_suction_command = false;
            const double suction_cmd = _jnt_position_command[8];
            if (!warned_legacy_suction_command && (suction_cmd > 2.0 || (suction_cmd > 0.0 && suction_cmd < 0.5)))
            {
                warned_legacy_suction_command = true;
                RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"),
                             "Ignoring legacy joint-8 suction command %.3f; use /fairino_suction_command instead.",
                             suction_cmd);
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
