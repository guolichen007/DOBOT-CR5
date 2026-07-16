#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: book_demo_cli.sh <command>

Commands:
  lock       Lock the latest stable book pose. Robot must be stationary.
  clear      Clear locked book pose and detection history.
  plan       Plan only; never sends physical motion.
  clearplan  Clear cached MoveIt plan.
  arm-token  Set the one-shot execution token. Does not move the robot.
  execute    Call execute service. Physical motion occurs only if the planner
             was launched with allow_execution:=true and arm-token was set.
  status     Show relevant topics, lock state, and confirmation parameter.
EOF
}

cmd="${1:-}"
case "${cmd}" in
  lock)
    rosservice call /book_demo/estimator/lock_target '{}'
    ;;
  clear)
    rosservice call /book_demo/estimator/clear_target '{}'
    ;;
  plan)
    rosservice call /book_demo/planner/plan_path '{}'
    ;;
  clearplan)
    rosservice call /book_demo/planner/clear_plan '{}'
    ;;
  arm-token)
    echo 'This only sets a confirmation token; the execute service is still required.'
    rosparam set /book_demo/confirm_execute CR5_BOOK_DRY_RUN_EXECUTE
    ;;
  execute)
    echo 'WARNING: this service can cause physical CR5 motion.'
    rosservice call /book_demo/planner/execute_path '{}'
    ;;
  status)
    echo '--- target lock ---'
    rostopic echo -n 1 /book_demo/estimator/target_locked || true
    echo '--- locked pose ---'
    rostopic echo -n 1 /book_demo/estimator/locked_pose || true
    echo '--- locked size ---'
    rostopic echo -n 1 /book_demo/estimator/locked_size || true
    echo '--- execute confirmation ---'
    rosparam get /book_demo/confirm_execute 2>/dev/null || echo '<unset>'
    ;;
  *)
    usage
    exit 2
    ;;
esac
