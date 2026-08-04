"""
CR5 Reconstruction — 纯 Python 质量合约模块.

职责:
  - release identity 解析 (不依赖 git 运行时)
  - evaluation label 安全化
  - evaluation metrics JSON schema 验证
  - acceptance Gate 判断
  - 质量报告段落构造

禁止导入:
  - open3d
  - cv2
  - rospy
  - Gazebo
  - RGB-D IO
  - TSDF
  - mesh cleanup
  - numpy, scipy

只允许使用 Python 标准库.
"""
import json
import math
import os
import re


# ── metrics schema ──────────────────────────────────────────────────────────

REQUIRED_METRIC_KEYS = [
    "accuracy.median_mm", "accuracy.p95_mm", "accuracy.rmse_mm",
    "completeness.median_mm", "completeness.p95_mm",
    "chamfer_mm",
    "accuracy_5mm", "accuracy_10mm", "accuracy_20mm",
    "completeness_5mm", "completeness_10mm", "completeness_20mm",
]

MAX_LABEL_LENGTH = 80


# ── release identity ────────────────────────────────────────────────────────

def resolve_release_id(explicit_release_id=None, git_tag_provider=None, environ=None):
    """解析 release identity.

    优先级:
      1. explicit_release_id (--release-id)
      2. git exact tag (通过 git_tag_provider 可调用对象)
      3. CR5_RELEASE_ID 环境变量
      4. "UNTAGGED"

    Args:
        explicit_release_id: 显式传入的 release id, 或 None.
        git_tag_provider: 无参可调用对象, 返回 tag 字符串或空字符串.
        environ: 环境变量 dict, 默认 os.environ.

    Returns:
        str: 解析后的 release id.
    """
    if explicit_release_id:
        return explicit_release_id

    if git_tag_provider is not None:
        tag = git_tag_provider()
        if tag:
            return tag

    env = environ if environ is not None else os.environ
    env_id = env.get("CR5_RELEASE_ID", "")
    if env_id:
        return env_id

    return "UNTAGGED"


# ── label sanitize ──────────────────────────────────────────────────────────

def sanitize_evaluation_label(release_id):
    """将 release_id 安全化为 evaluator label.

    规则:
      - 只允许字母、数字、下划线
      - 点、横线、斜线和空格统一替换为下划线
      - 连续下划线压缩为单个
      - 去除开头和结尾下划线
      - 空结果回退为 "untagged"
      - 限制最大长度 MAX_LABEL_LENGTH
      - 禁止产生路径分隔符
      - 禁止生成 ".."

    Args:
        release_id: 原始 release identity 字符串.

    Returns:
        str: 安全的 evaluation label.
    """
    if not release_id:
        return "untagged"

    # 替换所有非字母数字字符为下划线
    label = re.sub(r'[^A-Za-z0-9]+', '_', release_id)
    # 压缩连续下划线
    label = re.sub(r'_+', '_', label)
    # 去除首尾下划线
    label = label.strip('_')

    if not label:
        return "untagged"

    if len(label) > MAX_LABEL_LENGTH:
        label = label[:MAX_LABEL_LENGTH].rstrip('_')
        if not label:
            return "untagged"

    # 安全检查: 不得包含路径分隔符或 ..
    if '/' in label or '\\' in label or '..' in label:
        raise ValueError(f"sanitized label 包含非法路径字符: {label!r}")

    return label


# ── metrics validation ──────────────────────────────────────────────────────

def _validate_numeric(value, key, min_val=0.0, max_val=None):
    """验证单个数值 metric. 返回 (float_value, error_message_or_None)."""
    if value is None:
        return None, f"缺少 {key}"
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None, f"{key}={value!r} 类型错误 (期望 int/float, 不是 bool)"
    if not math.isfinite(value):
        return None, f"{key}={value} 不是有限数值"
    if value < min_val:
        return None, f"{key}={value} < {min_val}"
    if max_val is not None and value > max_val:
        return None, f"{key}={value} > {max_val}"
    return float(value), None


def load_and_validate_evaluation_metrics(metrics_path):
    """加载并验证评价 metrics JSON.

    验证 13 个必需字段:
      - accuracy: median_mm, p95_mm, rmse_mm (int/float, finite, >=0)
      - completeness: median_mm, p95_mm (int/float, finite, >=0)
      - chamfer_mm (int/float, finite, >=0)
      - coverage: accuracy_5mm/10mm/20mm, completeness_5mm/10mm/20mm
        (int/float, finite, 0≤value≤1)

    Args:
        metrics_path: metrics JSON 文件路径.

    Returns:
        dict: 验证后的 metrics, 所有值为 float.

    Raises:
        FileNotFoundError: 文件不存在.
        ValueError: JSON 解析失败或 schema 验证失败.
    """
    if not os.path.isfile(metrics_path):
        raise FileNotFoundError(f"metrics 文件不存在: {metrics_path}")

    with open(metrics_path) as f:
        try:
            data = json.load(f)
        except json.JSONDecodeError as e:
            raise ValueError(f"metrics JSON 解析失败: {e}")

    m = data.get("metrics", {})
    if not isinstance(m, dict) or not m:
        raise ValueError("metrics JSON 缺少 'metrics' 字段或为空")

    acc = m.get("accuracy", {})
    comp = m.get("completeness", {})
    cov = m.get("coverage", {})

    validated = {}
    errors = []

    # 距离指标
    for key, src in [
        ("accuracy_median_mm", acc.get("median_mm")),
        ("accuracy_p95_mm", acc.get("p95_mm")),
        ("accuracy_rmse_mm", acc.get("rmse_mm")),
        ("completeness_median_mm", comp.get("median_mm")),
        ("completeness_p95_mm", comp.get("p95_mm")),
        ("chamfer_mm", m.get("chamfer_mm")),
    ]:
        val, err = _validate_numeric(src, key, min_val=0.0)
        if err:
            errors.append(err)
        else:
            validated[key] = val

    # coverage 字段
    for key in [
        "accuracy_5mm", "accuracy_10mm", "accuracy_20mm",
        "completeness_5mm", "completeness_10mm", "completeness_20mm",
    ]:
        val, err = _validate_numeric(cov.get(key), f"coverage.{key}", min_val=0.0, max_val=1.0)
        if err:
            errors.append(err)
        else:
            validated[key] = val

    if errors:
        raise ValueError(
            f"metrics 验证失败 ({len(errors)} errors):\n  " + "\n  ".join(errors))

    return validated


# ── acceptance Gate ─────────────────────────────────────────────────────────

def build_acceptance_result(accuracy_median_mm, accuracy_p95_mm,
                            median_gate_mm=5.0, p95_gate_mm=16.0):
    """构建统一的 acceptance 结果.

    Args:
        accuracy_median_mm: accuracy median (mm).
        accuracy_p95_mm: accuracy P95 (mm).
        median_gate_mm: median gate 阈值 (mm).
        p95_gate_mm: P95 gate 阈值 (mm).

    Returns:
        dict: {
            "accuracy_median_gate_mm": float,
            "accuracy_p95_gate_mm": float,
            "production_pass": bool,
            "reasons": list[str]  (不含空字符串)
        }
    """
    prod_pass = bool(accuracy_median_mm <= median_gate_mm and accuracy_p95_mm <= p95_gate_mm)
    reasons = []

    if prod_pass:
        reasons.append(f"PASS: median<={median_gate_mm}mm and P95<={p95_gate_mm}mm")
    else:
        if accuracy_median_mm > median_gate_mm:
            reasons.append(
                f"FAIL: accuracy median={accuracy_median_mm:.2f}mm > {median_gate_mm}mm")
        if accuracy_p95_mm > p95_gate_mm:
            reasons.append(
                f"FAIL: accuracy P95={accuracy_p95_mm:.2f}mm > {p95_gate_mm}mm")

    # 合同: reasons 不得包含空字符串
    reasons = [r for r in reasons if r.strip()]

    return {
        "accuracy_median_gate_mm": float(median_gate_mm),
        "accuracy_p95_gate_mm": float(p95_gate_mm),
        "production_pass": prod_pass,
        "reasons": reasons,
    }


# ── quality report sections ─────────────────────────────────────────────────

def build_quality_metric_sections(metrics):
    """从已验证的 metrics dict 构建 accuracy / completeness / quality 报告段落.

    Args:
        metrics: load_and_validate_evaluation_metrics 的返回值.

    Returns:
        tuple of (accuracy_section, completeness_section, quality_section)
    """
    accuracy_section = {
        "median_mm": metrics["accuracy_median_mm"],
        "p95_mm": metrics["accuracy_p95_mm"],
        "rmse_mm": metrics["accuracy_rmse_mm"],
        "coverage_5mm": metrics["accuracy_5mm"],
        "coverage_10mm": metrics["accuracy_10mm"],
        "coverage_20mm": metrics["accuracy_20mm"],
        "status": "EVALUATED",
    }

    completeness_section = {
        "scope": "visible_gt_only",
        "median_mm": metrics["completeness_median_mm"],
        "p95_mm": metrics["completeness_p95_mm"],
        "coverage_5mm": metrics["completeness_5mm"],
        "coverage_10mm": metrics["completeness_10mm"],
        "coverage_20mm": metrics["completeness_20mm"],
    }

    quality_section = {"chamfer_mm": metrics["chamfer_mm"]}

    return accuracy_section, completeness_section, quality_section
