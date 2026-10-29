cmake_minimum_required(VERSION 3.8)
project(rg75_300_gripper)

# -------------------------
# 1. Dependencies
# -------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -------------------------
# 2. Interfaces (msg/srv)
# -------------------------
set(msg_files
  "msg/GripperState.msg"
)
set(srv_files
  "srv/GripperCommand.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)

# -------------------------
# 3. Executable
# -------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(gripper_node src/gripper_node.cpp)
ament_target_dependencies(gripper_node rclcpp)

# Link against generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# -------------------------
# 4. Fairino SDK (local path)
# -------------------------
# Adjust base path if you move SDK
set(FAIRINO_SDK_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../frcobot_ros2-V3.0.0_robotV3.8.0/fairino_hardware/libfairino"
)

set(FAIRINO_INCLUDE_DIR "${FAIRINO_SDK_DIR}/include")
# Use symlink libfairino.so so it works for any version
set(FAIRINO_LIB "${FAIRINO_SDK_DIR}/lib/libfairino.so")

target_include_directories(gripper_node PUBLIC ${FAIRINO_INCLUDE_DIR})
target_link_libraries(gripper_node
  ${cpp_typesupport_target}
  ${FAIRINO_LIB}
)

# -------------------------
# 5. Install & Export
# -------------------------
install(TARGETS gripper_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()