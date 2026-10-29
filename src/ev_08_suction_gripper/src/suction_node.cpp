#include "rclcpp/rclcpp.hpp"
#include "ev_08_suction_gripper/msg/suction_state.hpp"
#include "ev_08_suction_gripper/srv/suction_command.hpp"
#include "robot.h"   // Fairino SDK
#include "suction_config.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class SuctionNode : public rclcpp::Node
{
public:
    SuctionNode() : Node("suction_node"), forceA_(0.0), forceB_(0.0)
    {
        // Create service
        suction_service_ = this->create_service<ev_08_suction_gripper::srv::SuctionCommand>(
            "suction_command",
            std::bind(&SuctionNode::handle_suction_request, this, _1, _2));

        // Optional (not used now, but prepared)
        suction_pub_ = this->create_publisher<ev_08_suction_gripper::msg::SuctionState>(
            "suction_state", 10);

        RCLCPP_INFO(this->get_logger(), "Starting SuctionNode...");
        initialize_gripper();
    }

    ~SuctionNode() {
        robot_.CloseRPC();
    }

private:
    FRRobot robot_;
    float forceA_;
    float forceB_;

    rclcpp::Service<ev_08_suction_gripper::srv::SuctionCommand>::SharedPtr suction_service_;
    rclcpp::Publisher<ev_08_suction_gripper::msg::SuctionState>::SharedPtr suction_pub_;

    // ------------------------------------------------------------------
    // INITIALIZATION
    // ------------------------------------------------------------------
    void initialize_gripper()
    {
        ROBOT_STATE_PKG pkg = {};
        while (rclcpp::ok()) {
            int ret = robot_.RPC(ROBOT_IP);
            if (ret == 0) {
                RCLCPP_INFO(this->get_logger(), "Connected to robot.");
                break;
            }
            RCLCPP_ERROR(this->get_logger(), "Failed to connect (err=%d). Retrying...", ret);
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        char version[64] = "";
        robot_.GetSDKVersion(version);
        RCLCPP_INFO(this->get_logger(), "SDK version: %s", version);

        robot_.ResetAllError();

        // Communication setup
        AxleComParam param(7, 8, 1, 0, 5, 3, 1);
        robot_.SetAxleCommunicationParam(param);
        rclcpp::sleep_for(std::chrono::seconds(1));

        robot_.SetAxleLuaEnable(1);
        rclcpp::sleep_for(std::chrono::seconds(1));

        int luaStatus = 0;
        robot_.GetAxleLuaEnableStatus(&luaStatus);
        RCLCPP_INFO(this->get_logger(), "Lua Enable Status = %d", luaStatus);

        robot_.SetAxleLuaEnableDeviceType(0, 1, 0);
        rclcpp::sleep_for(std::chrono::seconds(1));

        int func[16] = {1,1,1,1,1,0,1,0,1,0,1,1,0,0,0,0};
        robot_.SetAxleLuaGripperFunc(1, func);
        rclcpp::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "Suction gripper initialization complete.");
    }

    // ------------------------------------------------------------------
    // SERVICE CALLBACK
    // ------------------------------------------------------------------
    void handle_suction_request(
        const std::shared_ptr<ev_08_suction_gripper::srv::SuctionCommand::Request> req,
        std::shared_ptr<ev_08_suction_gripper::srv::SuctionCommand::Response> res)
    {
        int ret = -1;

        switch (req->command)
        {
            // ======================================================
            //                     SUCTION A
            // ======================================================
            case 1:
                RCLCPP_INFO(this->get_logger(), "Actuating suction A");
                ret = robot_.MoveGripper(
                    SUCTION_INDEX,
                    100, 70, 60, 6000,   // A suction
                    1, 1,
                    0, 30, 20
                );
                break;

            case 2:
                RCLCPP_INFO(this->get_logger(), "Releasing suction A");
                ret = robot_.MoveGripper(
                    SUCTION_INDEX,
                    0, 70, 60, 6000,     // A release
                    1, 1,
                    0, 70, 60
                );
                break;

            // ======================================================
            //                     SUCTION B
            // ======================================================
            case 3:
                RCLCPP_INFO(this->get_logger(), "Actuating suction B");
                ret = robot_.MoveGripper(
                    SUCTION_INDEX,
                    0, 70, 60, 6000,     // B suction
                    0, 1,
                    90, 70, 60
                );
                break;

            case 4:
                RCLCPP_INFO(this->get_logger(), "Releasing suction B");
                ret = robot_.MoveGripper(
                    SUCTION_INDEX,
                    0, 70, 60, 6000,     // B release
                    0, 1,
                    0, 70, 60
                );
                break;

            // ======================================================
            //                BOTH A + B ON                          
            // ======================================================
            case 5:
                RCLCPP_INFO(this->get_logger(), "Actuating BOTH A + B suction");

                // Step 1: A ON and B ON
                ret = robot_.MoveGripper(
                    SUCTION_INDEX,
                    100, 70, 60, 6000,   // A suction
                    1, 1,
                    90, 70, 60
                );
                if (ret != 0) {
                    res->success = false;
                    res->message = "Failed to actuate suction A, err=" + std::to_string(ret);
                    return;
                }

                // rclcpp::sleep_for(std::chrono::milliseconds(200));

                // Step 2: B ON
                // ret = robot_.MoveGripper(
                //     SUCTION_INDEX,
                //     0, 70, 60, 6000,     // B suction
                //     0, 1,
                //     90, 70, 60
                // );
                break;

            // ======================================================
            default:
                res->success = false;
                res->message = "Invalid command. Use 1=A_ON, 2=A_OFF, 3=B_ON, 4=B_OFF, 5=BOTH_ON";
                return;
        }

        if (ret != 0) {
            res->success = false;
            res->message = "MoveGripper failed, error=" + std::to_string(ret);
            RCLCPP_ERROR(this->get_logger(), "MoveGripper returned %d", ret);
        } else {
            res->success = true;
            res->message = "Command executed successfully";
            RCLCPP_INFO(this->get_logger(), "MoveGripper success");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SuctionNode>());
    rclcpp::shutdown();
    return 0;
}