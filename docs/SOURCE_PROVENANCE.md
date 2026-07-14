# Source provenance

This clean demo repository was curated from the uploaded `cr5_ws.zip`.

## DOBOT source

- Original repository: `https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git`
- Recorded branch: `main`
- Recorded base commit: `8f2ef927a517e295cfb45d6fb66ca1e035e734e7`
- The uploaded working tree contained local modifications, especially in:
  - `dobot_bringup`
  - `dobot_description/urdf/cr5_robot.urdf`
  - `cr5_moveit`
  - local ArUco and hand-eye calibration files

The curated repository preserves the uploaded working copies of the packages needed by the first-stage CR5 demo. It is not a clean checkout of the upstream commit.

## Preserved lab-specific values

- `DOBOT_TYPE=cr5`
- Controller IP from the local launch file: `192.168.110.214`
- Suggested robot-facing laptop IP: `192.168.110.100/24`
- V3 ports: `29999`, `30003`, `30004`
- MoveIt group: `cr5_arm`
- Action: `/cr5_robot/joint_controller/follow_joint_trajectory`
- Candidate tool frame: `Tool_end`

## Deliberately excluded from catkin src

- `slamit`: CUDA/TensorRT-heavy and unnecessary for the first demo
- full `realsense-ros`: unnecessary for the first demo
- V4 driver, Gazebo, Qt and RViz control samples
- `build/` and `devel/` outputs

The old complete archive should be retained offline as a reference snapshot.
