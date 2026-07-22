#!/usr/bin/env bash
set -euo pipefail

# Search legacy workspaces for D455/CR5 fixed extrinsics, URDF frames, camera
# launch files, and hand-eye calibration artifacts. This script only reads files.

STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="${HOME}/cr5_book_calibration_inventory_${STAMP}.txt"

if (($# > 0)); then
  ROOTS=("$@")
else
  ROOTS=("${HOME}/cr5_ws" "${HOME}/cr5_ros1_ws")
fi

{
  echo "CR5 / D455 calibration inventory"
  echo "Generated: $(date -Is)"
  echo "Roots: ${ROOTS[*]}"
  echo

  for root in "${ROOTS[@]}"; do
    echo "================================================================"
    echo "ROOT: ${root}"
    echo "================================================================"
    if [[ ! -d "${root}" ]]; then
      echo "MISSING DIRECTORY"
      echo
      continue
    fi

    echo "-- Candidate calibration/model/launch files --"
    find "${root}" -type f \
      \( -iname '*calib*' -o -iname '*hand*eye*' -o -iname '*camera*' \
         -o -iname '*realsense*' -o -iname '*.urdf' -o -iname '*.xacro' \
         -o -iname '*.launch' -o -iname '*.yaml' -o -iname '*.yml' \
         -o -iname '*.json' -o -iname '*.engine' -o -iname '*.onnx' \
         -o -iname '*.pt' \) \
      -print | sort
    echo

    echo "-- Frame/extrinsic references with line numbers --"
    while IFS= read -r -d '' file; do
      grep -nEHi \
        'camera(_color)?(_optical)?_frame|camera_link|realsense|D455|Link6|Tool_end|spray_tcp|static_transform_publisher|hand.?eye|eye.?in.?hand|extrinsic|calib|origin[[:space:]]+xyz|origin[[:space:]]+rpy' \
        "${file}" 2>/dev/null | sed "s#^#${file}:#" || true
    done < <(find "${root}" -type f \
      \( -name '*.urdf' -o -name '*.xacro' -o -name '*.launch' \
         -o -name '*.yaml' -o -name '*.yml' -o -name '*.json' \
         -o -name '*.xml' -o -name '*.py' -o -name '*.cpp' \
         -o -name '*.h' -o -name '*.hpp' -o -name '*.md' -o -name '*.txt' \) \
      -print0)
    echo
  done

  echo "================================================================"
  echo "LIVE ROS CHECK (only populated when ROS is already running)"
  echo "================================================================"
  if command -v rosparam >/dev/null 2>&1 && rosparam list >/dev/null 2>&1; then
    echo "-- Relevant ROS parameters --"
    rosparam list | grep -Ei 'robot_description|camera|realsense|tool|tcp|calib' || true
    echo
    if rosparam get /robot_description >/tmp/cr5_robot_description_${STAMP}.urdf 2>/dev/null; then
      echo "-- Relevant lines from live /robot_description --"
      grep -nEHi 'camera|Link6|Tool_end|spray_tcp|origin[[:space:]]+xyz|origin[[:space:]]+rpy' \
        /tmp/cr5_robot_description_${STAMP}.urdf || true
    fi
    echo
    echo "-- TF-related nodes --"
    rosnode list 2>/dev/null | grep -Ei 'robot_state_publisher|static|realsense|camera' || true
  else
    echo "ROS master unavailable; live checks skipped."
  fi
} | tee "${REPORT}"

echo
echo "Saved report: ${REPORT}"
echo "Review the report before copying any transform into the new package."
echo "Do not publish the same parent->child transform from both URDF and a static TF node."
