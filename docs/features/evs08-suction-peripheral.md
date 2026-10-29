# EVS08 Suction Peripheral

## Current Status

The JODELL EVS08 suction end-effector is controlled through the FAIRINO peripheral / axle-Lua / gripper protocol over RS485, not through simple DO in the current ROS path.

Confirmed configuration:

- Suction gripper ID: `1`
- Finger gripper ID: `2`
- `SUCTION_INDEX` remains `1`

Previously reported by user on 2026-07-03: the peripheral suction path can leave the FAIRINO controller in a non-motion state requiring full controller power cycle even when ROS/MoveIt later reports motion success. Treat this path as manual-only until hardware validation proves the new release path preserves controller motion.

## Architecture

Runtime interface:

- Topic: `/initialize_suction`
- Service: `/fairino_suction_command`
- Service type: `fairino_msgs/srv/FairinoSuctionCommand`
- Implementation: `fairino_hardware/src/fairino_hardware_interface7.cpp`
- Operator helper scripts: `scripts/suction_init.sh`, `scripts/suction_on.sh`, `scripts/suction_off.sh`, dry-run by default and hardware-gated with `--execute`

`/initialize_suction` still performs the existing peripheral setup sequence and calls `ActGripper(SUCTION_INDEX,1)` for activation/init. Do not expand runtime `SetAxleLua*` setup without new SDK/vendor evidence.

This project workflow uses suction A / top two cups only. Suction B / bottom cups and ALL_ON are intentionally disabled. Runtime suction commands use one full EVS08 `MoveGripper` state per command with channel B forced OFF.

Normal operator workflow:

```bash
./scripts/suction_init.sh --execute
./scripts/suction_on.sh --execute
./scripts/suction_off.sh --execute
```

`scripts/suction_init.sh` publishes `/initialize_suction` and polls read-only service command `0 = STATUS` when available. STATUS does not command suction hardware.

## EVS08 Mapping

```text
MoveGripper(index, position, velocity, force, max_time, block, type, rotNum, rotVel, rotTorque)
```

- `index`: FAIRINO gripper/device ID, `1` for suction in this controller
- `position`: channel A start/stop, `100` ON, `0` OFF
- `velocity`: channel A max vacuum/pressure, currently `70`
- `force`: channel A min vacuum/pressure, currently `60`
- `max_time`: currently `6000`
- `block`: currently `1`
- `type`: rotating-gripper mode, currently `1`
- `rotNum`: channel B start/stop; this project always sends `0` OFF
- `rotVel`: channel B max vacuum/pressure, currently `70`
- `rotTorque`: channel B min vacuum/pressure, currently `60`

## Command Table

```text
A/top cups ON, B/bottom cups forced OFF:
  MoveGripper(1,100,70,60,6000,1,1,0,70,60)

OFF / stop-all / break-vacuum:
  MoveGripper(1,0,70,60,6000,1,1,0,70,60)
```

Service command `1 = A_ON` sends the top-cups-only command. Service commands `2 = A_OFF` and `6 = ALL_OFF` both send the stop-all/break-vacuum command. Commands `3 = B_ON`, `4 = B_OFF`, and `5 = ALL_ON` are retained as numeric constants for compatibility but are rejected before any SDK call.

## Safety Notes

- Do not use `ActGripper(1,0)` for normal OFF/ALL_OFF release. It may deactivate/reset the FAIRINO peripheral slot and is a suspected contributor to the controller non-motion fault.
- Do not modify Lua or firmware files from ROS-side changes.
- Do not command suction without explicit operator approval and `--execute`.
- Do not turn on suction B / bottom cups in this project workflow.
- Do not integrate suction into automated cover removal until post-suction robot motion is validated.
- No real suction/grip feedback is currently available; service success only means the FAIRINO SDK call returned success.
- Lua and firmware are vendor/controller-side artifacts and are not modified by this ROS workflow.

## Hardware Validation Plan

1. Power cycle the robot/controller.
2. Confirm the Web UI still shows suction gripper ID `1` and finger gripper ID `2`.
3. Start ROS/MoveIt as usual and verify a known-safe motion before suction.
4. Run `./scripts/suction_init.sh --execute` only when the operator explicitly approves suction preparation.
5. Run `./scripts/suction_off.sh --execute` first. It should command EVS08 stop-all/break-vacuum and should not create suction.
6. Verify robot motion still works.
7. Run `./scripts/suction_on.sh --execute` to turn on suction A / top two cups only.
8. Run `./scripts/suction_off.sh --execute` to release with EVS08 stop-all/break-vacuum, then verify motion.
9. Stop testing immediately if any OFF/release command fails or if robot motion is not physically healthy after suction.
