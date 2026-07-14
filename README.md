# CR5 A4 ROS 1 Demo — clean workspace

A curated ROS 1 Noetic catkin workspace for the first-stage DOBOT CR5 A4 raster coverage demo.

## Included

```text
src/dobot_bringup       V3 TCP/IP driver, services, feedback and trajectory Action
src/dobot_description   CR5-only URDF and CR5 meshes, including current Tool_end/camera transforms
src/cr5_moveit          CR5 MoveIt 1 configuration
src/dobot_moveit        DOBOT_TYPE-based MoveIt launcher
src/a4_spray_demo       reviewed A4 raster demo and preflight check
```

## Excluded from the active workspace

- `slamit`
- full RealSense wrapper
- V4 driver
- Gazebo and GUI sample packages
- generated `build/` and `devel/`

The old full `cr5_ws.zip` should be retained as a read-only reference.

## First checkout on Ubuntu 20.04 + ROS Noetic

```bash
cd ~/cr5_a4_ros1
./scripts/build.sh
source devel/setup.bash
```

Robot-facing Ethernet configuration, where `enp3s0` is an example interface:

```bash
sudo ./scripts/configure_robot_network.sh enp3s0
./scripts/network_check.sh
```

## Start sequence

Terminal 1:

```bash
roscore
```

Terminal 2:

```bash
cd ~/cr5_a4_ros1
./scripts/start_driver.sh
```

Terminal 3:

```bash
cd ~/cr5_a4_ros1
./scripts/start_moveit.sh
```

Terminal 4:

```bash
cd ~/cr5_a4_ros1
./scripts/preflight.sh
```

## Demo stages

Plan the 100 × 30 mm test:

```bash
./scripts/plan_small.sh
```

After physical safety checks, controller-side SpeedFactor=5 and successful preflight:

```bash
./scripts/enable_robot_5pct.sh
./scripts/preflight.sh
./scripts/execute_small.sh
```

Only after the small test succeeds:

```bash
./scripts/plan_a4.sh
```

Full A4 real execution remains an explicit manual launch command in the package README so it cannot be started accidentally from a convenience script.

## Important

`Tool_end` in the current URDF has a large offset from `Link6`. Verify it against the physical tool before any real motion.
The existing driver sends ServoJ points at a fixed period and does not follow MoveIt `time_from_start`; this demo validates geometric coverage only.
