# FAIRINO Hardware Fixes

Purpose: document path and runtime-library fixes that matter for future build/debug work.

## fairino_hardware

Verified current state:

- `fairino_hardware/CMakeLists.txt` links against the local SDK library under the package source tree.
- It creates local `libfairino.so.2` and `libfairino.so` symlinks from `libfairino.so.2.2.0`.
- It copies `libfairino.so.2.2.0` into the install prefix and creates install-tree symlinks.
- Current hardware header includes local package config:

```text
fairino_hardware/gripper_config_hardware.hpp
```

Verified from backup context:

- A backup header shows an older absolute include path for gripper config.
- Current source no longer uses that absolute include.

## fairino_mtc_demo

Verified current state:

- `fairino_mtc_demo/CMakeLists.txt` derives `FAIRINO_SDK_DIR` from `${CMAKE_CURRENT_SOURCE_DIR}/../fairino_hardware/libfairino`.
- It checks for `robot.h` and `libfairino.so.2.2.0`.
- It creates a build-time runtime lib directory with `libfairino.so.2` and `libfairino.so`.
- It installs `libfairino.so.2.2.0` and recreates install-tree symlinks.
- Launch files use package-relative MoveIt config paths through `MoveItConfigsBuilder`.

Verified from backup context:

- Backup files show older hardcoded FAIRINO SDK and MoveIt config paths.
- Current active files avoid those legacy absolute paths.

## ev_08_suction_gripper

Verified current state:

- `ev_08_suction_gripper/CMakeLists.txt` derives the FAIRINO SDK path from this workspace source tree.
- It links `suction_node` against `libfairino.so.2.2.0`.

Verified from backup context:

- A backup CMake file shows an older hardcoded FAIRINO library path.
- Current active CMake uses the local workspace-relative SDK path.

## Practical Debugging Notes

- Do not reintroduce legacy absolute workspace paths.
- If runtime loading fails for `libfairino`, inspect the symlinks in both:

```text
src/frcobot_ros2-V3.0.0_robotV3.8.0/fairino_hardware/libfairino/lib
install/lib
```

- If MoveIt launches but hardware does not connect, inspect:

```text
src/frcobot_ros2-V3.0.0_robotV3.8.0/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
src/frcobot_ros2-V3.0.0_robotV3.8.0/fairino_hardware/include/fairino_hardware/fairino_hardware_interface.hpp
src/frcobot_ros2-V3.0.0_robotV3.8.0/fairino_hardware/src/fairino_hardware_interface7.cpp
```
