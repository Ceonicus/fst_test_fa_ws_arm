# Suction Control

Purpose: document the operator-facing EVS08 suction commands used by the contact-box workflow.

Source of truth: [Contact-Box Cover-Removal Developer Guide](../contact-box-vision-to-motion-developer-guide.md). See also [EVS08 Suction Peripheral](../evs08-suction-peripheral.md).

## Hardware Context

- End effector: JODELL EVS08 electric suction gripper.
- FAIRINO protocol path: RS485 / Peripheral / axle-Lua.
- Suction gripper ID: `1`.
- Finger gripper ID: `2`.
- Project workflow uses suction A / top two cups only.
- Suction B / bottom cups is intentionally disabled.

## Main Scripts

- `scripts/suction_init.sh`
- `scripts/suction_on.sh`
- `scripts/suction_off.sh`

## Operator Commands

```bash
./scripts/suction_init.sh --execute
./scripts/suction_on.sh --execute
./scripts/suction_off.sh --execute
```

Dry-run mode is the default when `--execute` is omitted.

## Safety Notes

- `suction_init.sh` prepares the peripheral and should not turn suction ON.
- `suction_on.sh` turns ON A/top cups and forces B/bottom cups OFF.
- `suction_off.sh` sends EVS08 stop-all/break-vacuum and avoids `ActGripper(1,0)`.
- Do not enable suction B in normal project workflows.
- Do not modify Lua or firmware for normal ROS tuning.
