# ROS 2 Workspace Helper Scripts

Normal terminal flow:

Terminal 1:

```bash
./scripts/launch_robot.sh
```

Terminal 2:

```bash
./scripts/launch_realsense.sh
```

Terminal 3:

```bash
./scripts/launch_apriltag.sh
```

Terminal 4:

```bash
./scripts/view_camera.sh
```

Terminal 5:

```bash
./scripts/capture.sh 10
```

After capture:

```bash
./scripts/verify_latest_handeye.sh
```

Other helpers:

```bash
./scripts/check_system.sh
```

Checks ROS package prefixes and active old workspace references.

```bash
./scripts/build_safe.sh
```

Builds slowly but safely on Jetson.

```bash
./scripts/stop_ros.sh
```

Stops common ROS processes if the graph gets messy.
