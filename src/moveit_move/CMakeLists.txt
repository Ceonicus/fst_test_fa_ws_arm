cmake_minimum_required(VERSION 3.8)
project(moveit_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fairino_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rg75_300_gripper REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ContactBoxCoverRemoval.action"
)

add_executable(moveit_move_node src/moveit_move.cpp)
set_target_properties(moveit_move_node PROPERTIES OUTPUT_NAME moveit_move)
target_include_directories(moveit_move_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_move_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_move_node
  "moveit_ros_planning_interface"
  "rclcpp"
  "rg75_300_gripper"
)

add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "rg75_300_gripper"
)

add_executable(contact_box_plan_debug src/contact_box_plan_debug.cpp)
target_include_directories(contact_box_plan_debug PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(contact_box_plan_debug PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  contact_box_plan_debug
  "geometry_msgs"
  "moveit_core"
  "moveit_ros_planning_interface"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
)

add_executable(contact_box_cover_removal_action_server src/contact_box_cover_removal_action_server.cpp)
target_include_directories(contact_box_cover_removal_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(contact_box_cover_removal_action_server PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  contact_box_cover_removal_action_server
  "fairino_msgs"
  "geometry_msgs"
  "moveit_core"
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(contact_box_cover_removal_action_server "${cpp_typesupport_target}")

add_executable(return_to_saved_joint_pose src/return_to_saved_joint_pose.cpp)
target_include_directories(return_to_saved_joint_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(return_to_saved_joint_pose PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  return_to_saved_joint_pose
  "moveit_ros_planning_interface"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS
  moveit_move_node
  hello_moveit
  contact_box_plan_debug
  contact_box_cover_removal_action_server
  return_to_saved_joint_pose
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
