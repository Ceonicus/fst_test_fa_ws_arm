#ifndef GRIPPER_CONFIG_HW_HPP
#define GRIPPER_CONFIG_HW_HPP

#pragma once
#include <cstdint>

//robot connection
constexpr const char* ROBOT_IP = "192.168.58.2";

//gripper index (0 to 15, matches bit index in GetGripperActivateStatus)
constexpr int GRIPPER_INDEX = 2;

//gripper configuration parameters
constexpr int GRIPPER_COMPANY  = 6;
constexpr int GRIPPER_DEVICE   = 0;
constexpr int GRIPPER_SOFTWARE = 0;
constexpr int GRIPPER_BUS      = 2;

//default move gripper parameters
constexpr int DEFAULT_POSITION   = 3;     //Percentage of position, range[0~100]
constexpr int DEFAULT_VELOCITY   = 30;    //Percentage of velocity, range[0~100]
constexpr int DEFAULT_FORCE      = 20;    //Percentage of torque, range[0~100]
constexpr int DEFAULT_MAX_TIME   = 5000;  //Maximum wait time, range[0~30000], unit: ms
constexpr uint8_t DEFAULT_BLOCK  = 0;     //0- blocking, 1- non-blocking
constexpr int DEFAULT_TYPE       = 0;     //grippr type, 0-parallel clamp; 1- Rotating claw

//leave the below 3 variable values at zero for parallel gripper
constexpr double DEFAULT_ROT_NUM = 0.0;   //Number of turns[0~100]
constexpr int DEFAULT_ROT_VEL    = 0;     //Percent rotation speed[0~100]
constexpr int DEFAULT_ROT_TORQUE = 0;     //Percentage of rotating torque [0~100]

// Activation wait settings
constexpr int ACTIVATION_RETRIES = 50;    // loop count
constexpr int ACTIVATION_DELAY_MS = 100;  // delay between retries

// State publisher settings
constexpr int STATE_PUB_PERIOD_MS = 500;  // publish every 500 ms
#endif // GRIPPER_CONFIG_HW_HPP
