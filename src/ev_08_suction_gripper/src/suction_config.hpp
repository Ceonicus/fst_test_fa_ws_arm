#ifndef GRIPPER_CONFIG_HPP
#define GRIPPER_CONFIG_HPP

#pragma once
#include <cstdint>

//robot connection
constexpr const char* ROBOT_IP = "192.168.58.2";

//gripper index (0 to 15, matches bit index in GetGripperActivateStatus)
constexpr int SUCTION_INDEX = 1;

#endif // GRIPPER_CONFIG_HPP