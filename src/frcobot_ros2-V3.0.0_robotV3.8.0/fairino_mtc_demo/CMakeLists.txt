cmake_minimum_required(VERSION 3.8)
project(fairino_mtc_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

get_filename_component(FAIRINO_SDK_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../fairino_hardware/libfairino"
  ABSOLUTE)
set(FAIRINO_SDK_INCLUDE_DIR "${FAIRINO_SDK_DIR}/include")
set(FAIRINO_LIBRARY_REAL "${FAIRINO_SDK_DIR}/lib/libfairino.so.2.2.0")
set(FAIRINO_RUNTIME_LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/fairino_lib")

if(NOT EXISTS "${FAIRINO_SDK_INCLUDE_DIR}/robot.h")
  message(FATAL_ERROR "FAIRINO SDK header not found: ${FAIRINO_SDK_INCLUDE_DIR}/robot.h")
endif()

if(NOT EXISTS "${FAIRINO_LIBRARY_REAL}")
  message(FATAL_ERROR "FAIRINO SDK library not found: ${FAIRINO_LIBRARY_REAL}")
endif()

file(MAKE_DIRECTORY "${FAIRINO_RUNTIME_LIB_DIR}")
file(REMOVE
  "${FAIRINO_RUNTIME_LIB_DIR}/libfairino.so"
  "${FAIRINO_RUNTIME_LIB_DIR}/libfairino.so.2")
file(CREATE_LINK "${FAIRINO_LIBRARY_REAL}" "${FAIRINO_RUNTIME_LIB_DIR}/libfairino.so.2" SYMBOLIC)
file(CREATE_LINK "libfairino.so.2" "${FAIRINO_RUNTIME_LIB_DIR}/libfairino.so" SYMBOLIC)

#for mtc_test.cpp
# find_package(rg75_300_gripper REQUIRED)

# add_executable(mtc_node src/mtc_node.cpp) <--- uncomment for original mtc_node
add_executable(mtc_node src/mtc_node.cpp)
target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${FAIRINO_SDK_INCLUDE_DIR})
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mtc_node
  moveit_task_constructor_core
  rclcpp
  # rg75_300_gripper
)

target_link_libraries(mtc_node
  ${FAIRINO_LIBRARY_REAL}
)
set_target_properties(mtc_node PROPERTIES
  BUILD_RPATH "${FAIRINO_RUNTIME_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN/.."
)

add_compile_options(-g)

install(TARGETS mtc_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES "${FAIRINO_LIBRARY_REAL}"
  DESTINATION lib
  RENAME libfairino.so.2.2.0)
install(CODE [=[
  set(_fairino_lib_dir "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib")
  file(REMOVE
    "${_fairino_lib_dir}/libfairino.so"
    "${_fairino_lib_dir}/libfairino.so.2")
  file(CREATE_LINK "libfairino.so.2.2.0"
    "${_fairino_lib_dir}/libfairino.so.2" SYMBOLIC)
  file(CREATE_LINK "libfairino.so.2"
    "${_fairino_lib_dir}/libfairino.so" SYMBOLIC)
]=])

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.bak" EXCLUDE)

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

ament_package()
