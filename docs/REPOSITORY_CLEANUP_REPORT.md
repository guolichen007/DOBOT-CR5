# 工程整理报告 — 2026-07-22

## Repository
- Repository: guolichen007/DOBOT-CR5
- Default branch: main
- Development branch: develop/multi-camera-calibration
- Stable tag: calibration-baseline-20260722
- Backup: /home/ydkj/DOBOT-CR5-before-cleanup-20260722_165759.bundle (11 MB)

## HEAD
- Before: b12838f (feature/calibration-target-v1)
- Develop: da88881
- Main: d979d84

## Remote Branches
- Before: 12 (main + 5 feature + 6 fix)
- After: 2 (main, develop/multi-camera-calibration)
- Deleted: 11 remote branches

## Renamed (28 files)
run_scene_v337.sh → run_simulation.sh
scene_v33_spray.launch → spray_simulation.launch
use_spray_session_v337.sh → use_simulation_session.sh
models/calibration_target_v1/ → models/calibration_target/
config/calibration_target_v1.yaml → config/calibration/calibration_target.yaml
spawn_cameras_v31.py → spawn_fixed_cameras.py
object_yaw_v31.py → control_object_yaw.py
object_pose_tf_v31.py → publish_object_pose.py
spray_simulator_v33.py → spray_simulator.py
(and 20 more — see git log for full list)

## Deleted (25+ files)
Old xacro, duplicate assets, superseded V2/V31/V33/V337 scripts

## Asset SHA-256 (all verified)
charuco_front.png: 88bd0f1959...
charuco_left.png: 4d658ccf...
charuco_back.png: 0301d4d9...
apriltag_right.png: 0cb8caf9...
apriltag_top.png: f7b68467...

## Build
- catkin_make: PASS
- aruco_compat import: PASS
- Geometry validation: CALIBRATION_TARGET_GEOMETRY_PASS
- XML validation: PASS

## Not Completed
- Production three-camera extrinsics, Bundle Adjustment, TSDF, spray path, real robot

## Next: docs/NEXT_MULTI_CAMERA_CALIBRATION.md
