"""
CR5 Reconstruction — 评估运行器模块.

职责:
  - 封装 evaluator subprocess 调用链
  - fail-closed: 任何失败必须抛出异常

禁止导入:
  - open3d
  - cv2
  - rospy
  - Gazebo
  - RGB-D IO
  - TSDF
  - mesh cleanup
  - numpy, scipy

只允许使用 Python 标准库 + quality_contract.
"""
import os
import subprocess
import sys

from cr5_spray_perception.reconstruction.quality_contract import (
    sanitize_evaluation_label,
    load_and_validate_evaluation_metrics,
)


def run_evaluator(evaluator_script, recon_mesh, output_dir, release_id,
                  visible_gt, roi_min, roi_max, timeout_s=180, env=None,
                  model_pose_json=None):
    """运行外部 evaluator subprocess 并返回结构化结果.

    调用链:
      1. 验证 evaluator_script 存在
      2. 验证 recon_mesh 存在
      3. 验证 visible_gt 存在
      4. 生成安全 eval_label
      5. 构造 evaluator 命令
      6. subprocess.run(..., check=True)
      7. 确认 metrics 文件存在
      8. 调用 load_and_validate_evaluation_metrics
      9. 返回结构化结果

    任何失败必须抛出异常. 不吞错误.

    Args:
        evaluator_script: evaluator Python 脚本的绝对路径.
        recon_mesh: 重建 mesh PLY 文件的绝对路径.
        output_dir: 评估输出目录.
        release_id: 原始 release identity (会经过 sanitize).
        visible_gt: visible GT PLY 文件路径.
        roi_min: [x, y, z] ROI 最小值.
        roi_max: [x, y, z] ROI 最大值.
        timeout_s: subprocess 超时 (秒).
        env: 环境变量 dict, 默认 os.environ.

    Returns:
        dict: {
            "eval_label": str,
            "metrics_path": str,
            "metrics": dict,   # load_and_validate_evaluation_metrics 的返回值
        }

    Raises:
        FileNotFoundError: 必需文件不存在.
        RuntimeError: subprocess 返回非零.
        subprocess.TimeoutExpired: 超时.
        ValueError: 非法 label 或 metrics 验证失败.
    """
    # 1. 验证 evaluator_script 存在
    if not os.path.isfile(evaluator_script):
        raise FileNotFoundError(f"evaluator 脚本不存在: {evaluator_script}")

    # 2. 验证 recon_mesh 存在
    if not os.path.isfile(recon_mesh):
        raise FileNotFoundError(f"重建 mesh 不存在: {recon_mesh}")

    # 3. 验证 visible_gt 存在
    if not os.path.isfile(visible_gt):
        raise FileNotFoundError(f"visible GT 文件不存在: {visible_gt}")

    # 4. 生成安全 eval_label
    eval_label = sanitize_evaluation_label(release_id)

    # 5. 构造 evaluator 命令
    os.makedirs(output_dir, exist_ok=True)
    cmd = [
        sys.executable, evaluator_script,
        "--recon-mesh", recon_mesh,
        "--output-dir", output_dir,
        "--label", eval_label,
        "--visible-gt", visible_gt,
        "--target-roi-min", str(roi_min[0]), str(roi_min[1]), str(roi_min[2]),
        "--target-roi-max", str(roi_max[0]), str(roi_max[1]), str(roi_max[2]),
    ]
    if model_pose_json:
        if not os.path.isfile(model_pose_json):
            raise FileNotFoundError(f"model_pose_json 不存在: {model_pose_json}")
        cmd += ["--model-pose-json", model_pose_json]

    # 6. subprocess.run (fail-closed)
    run_env = env if env is not None else os.environ
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=True,
            env=run_env,
        )
    except subprocess.CalledProcessError as e:
        stderr_tail = e.stderr[-500:] if e.stderr else "(no stderr)"
        raise RuntimeError(
            f"evaluator 失败 (exit={e.returncode}): {stderr_tail}") from e

    # 如果 check=True 但 returncode != 0 (理论上不会到这里, 但防御)
    if result.returncode != 0:
        stderr_tail = result.stderr[-500:] if result.stderr else "(no stderr)"
        raise RuntimeError(
            f"evaluator 返回非零 (exit={result.returncode}): {stderr_tail}")

    # 7. 确认 metrics 文件存在
    metrics_path = os.path.join(output_dir, f"{eval_label}_metrics.json")
    if not os.path.isfile(metrics_path):
        raise FileNotFoundError(
            f"evaluator 运行后 metrics 文件未生成: {metrics_path}")

    # 8. 调用 load_and_validate_evaluation_metrics
    metrics = load_and_validate_evaluation_metrics(metrics_path)

    # 9. 返回结构化结果
    return {
        "eval_label": eval_label,
        "metrics_path": metrics_path,
        "metrics": metrics,
    }
