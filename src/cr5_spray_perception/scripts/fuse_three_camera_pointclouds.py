#!/usr/bin/env python3
"""CR5 Reconstruction — 三相机点云融合脚本 (v3)."""
import os, sys, argparse, json, yaml, logging, hashlib, subprocess, shutil, datetime, platform
import numpy as np

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("fuse_pointclouds")

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.rgbd_io import (
    load_rgbd_data, validate_rgbd_contract, RGBDData,
)
from cr5_spray_perception.reconstruction.extrinsics import load_calibrated_rig
from cr5_spray_perception.reconstruction.pointcloud_fusion import (
    fuse_three_camera_pointclouds, CAMERA_COLORS, OVERLAP_PAIRS,
    LOCAL_ALIGNMENT_DEFAULTS, GATE_DEFAULTS,
)
from cr5_spray_perception.reconstruction.contracts import REQUIRED_CAMERAS
from cr5_spray_perception.reconstruction.dataset_layout import (
    resolve_capture_group, load_group_manifest,
)
from cr5_spray_perception.reconstruction.registration import load_registration_evidence

EXIT_OK = 0
EXIT_DATA_FAIL = 2
EXIT_GATE_FAIL = 3
EXIT_INTERNAL = 4


def _sha256_file(path):
    """计算文件 SHA256."""
    if not path or not os.path.isfile(path):
        return None
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def _sha256_str(s):
    """计算字符串 SHA256."""
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


def _get_git_sha():
    """获取当前 Git SHA."""
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=WS, stderr=subprocess.DEVNULL
        ).decode().strip()
    except Exception:
        return "unknown"


def _get_git_branch():
    """获取当前 Git 分支."""
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=WS, stderr=subprocess.DEVNULL
        ).decode().strip()
    except Exception:
        return "unknown"


def _get_environment_info():
    """收集运行环境信息."""
    info = {"git_sha": _get_git_sha(), "git_branch": _get_git_branch(),
            "timestamp_utc": datetime.datetime.utcnow().isoformat() + "Z"}
    try:
        info["ros_distro"] = os.environ.get("ROS_DISTRO", "unknown")
    except Exception:
        info["ros_distro"] = "unknown"
    try:
        info["python_version"] = sys.version
    except Exception:
        info["python_version"] = "unknown"
    for pkg in ["numpy", "scipy", "cv2", "open3d", "yaml"]:
        try:
            if pkg == "cv2":
                import cv2; info["opencv_version"] = cv2.__version__
            elif pkg == "yaml":
                import yaml; info["pyyaml_version"] = yaml.__version__
            else:
                mod = __import__(pkg)
                info[f"{pkg}_version"] = getattr(mod, "__version__", "unknown")
        except Exception:
            info[f"{pkg}_version"] = "unknown"
    try:
        import platform; info["ubuntu_version"] = platform.version()
    except Exception:
        info["ubuntu_version"] = "unknown"
    return info


def _save_provenance(output_dir, args, config, config_path, rig, rig_path,
                     evidence, evidence_path, layout, manifest):
    """保存 provenance 目录,包含所有输入副本."""
    prov_dir = os.path.join(output_dir, "provenance")
    os.makedirs(prov_dir, exist_ok=True)

    # 1. 保存 effective config
    effective_yaml_path = os.path.join(prov_dir, "three_camera_reconstruction_effective.yaml")
    shutil.copy2(config_path, effective_yaml_path) if config_path and os.path.isfile(config_path) else None
    effective_config_sha = _sha256_file(effective_yaml_path)

    # 2. 保存 calibrated_rig
    rig_copy = os.path.join(prov_dir, "calibrated_rig_stable_v1.yaml")
    shutil.copy2(rig_path, rig_copy) if rig_path and os.path.isfile(rig_path) else None

    # 3. 保存 depth_registration.json
    if evidence_path and os.path.isfile(evidence_path):
        shutil.copy2(evidence_path, os.path.join(prov_dir, "depth_registration.json"))

    # 4. 保存 group_manifest.json
    manifest_path_src = os.path.join(layout.group_dir, "group_manifest.json")
    if os.path.isfile(manifest_path_src):
        shutil.copy2(manifest_path_src, os.path.join(prov_dir, "group_manifest.json"))

    # 5. 保存 environment.json
    env_info = _get_environment_info()
    env_info["command"] = " ".join(sys.argv)
    env_info["config_path"] = os.path.abspath(config_path) if config_path else "N/A"
    env_info["config_sha256"] = effective_config_sha
    env_info["rig_path"] = os.path.abspath(rig_path)
    env_info["rig_sha256"] = _sha256_file(rig_path)
    env_info["stable_v1_json_sha256"] = rig.get("source_calibration", {}).get("sha256", "N/A") if rig else "N/A"
    if evidence:
        env_info["registration_evidence_path"] = os.path.abspath(evidence_path) if evidence_path else "N/A"
        env_info["registration_evidence_sha256"] = evidence.get("sha256", _sha256_file(evidence_path))
    with open(os.path.join(prov_dir, "environment.json"), "w") as f:
        json.dump(env_info, f, indent=2, default=str)

    # 6. 保存 command.txt
    with open(os.path.join(prov_dir, "command.txt"), "w") as f:
        f.write(" ".join(sys.argv) + "\n")

    # 7. 保存 input manifest (所有输入文件 SHA256)
    input_files = {}
    for cam_name in REQUIRED_CAMERAS:
        cam_dir = layout.camera_dirs.get(cam_name) or os.path.join(layout.group_dir, cam_name)
        for fname in ["color.png", "depth.npy", "color_camera_info.yaml", "depth_camera_info.yaml"]:
            fpath = os.path.join(cam_dir, fname)
            input_files[f"{cam_name}/{fname}"] = {
                "path": fpath, "sha256": _sha256_file(fpath),
                "size_bytes": os.path.getsize(fpath) if os.path.isfile(fpath) else 0}
    with open(os.path.join(prov_dir, "input_manifest.json"), "w") as f:
        json.dump(input_files, f, indent=2, default=str)

    return prov_dir, env_info, input_files


def _merge_config_with_cli(config_file, args):
    """CLI/YAML 优先级: 显式 CLI > YAML > 默认值."""
    if config_file and os.path.isfile(config_file):
        with open(config_file, "r") as f:
            config = yaml.safe_load(f) or {}
        logger.info("配置已加载: %s", config_file)
    else:
        config = {}

    sources = {}
    yaml_keys_formal = set(config.keys()) if config_file and os.path.isfile(config_file) else set()

    # depth parameters (非 None 才覆盖)
    for key, attr in [("depth_min_m", "depth_min"), ("depth_max_m", "depth_max"),
                       ("voxel_downsample_m", "voxel_size")]:
        val = getattr(args, attr, None)
        if val is not None:
            config[key] = val; sources[key] = "cli"

    # roi_min / roi_max: 只有显式传入才覆盖 (None 时不覆盖)
    if args.roi_min is not None:
        config.setdefault("roi_rig", {})["min"] = list(args.roi_min)
        sources["roi_rig.min"] = "cli"
    if args.roi_max is not None:
        config.setdefault("roi_rig", {})["max"] = list(args.roi_max)
        sources["roi_rig.max"] = "cli"

    # 默认值
    config.setdefault("camera_names", REQUIRED_CAMERAS)
    config.setdefault("overlap_thresholds_mm", [10, 20, 30])
    if "local_correspondence_diagnostic" not in config and "common_overlap" not in config:
        config["local_correspondence_diagnostic"] = dict(LOCAL_ALIGNMENT_DEFAULTS)
    if "roi_rig" not in config:
        config["roi_rig"] = {"min": [-0.5, -1.0, 0.0], "max": [2.0, 1.0, 2.5]}

    # 记录每个参数来源
    full_sources = {}
    def _trace(cfg, prefix=""):
        for k, v in cfg.items():
            fk = f"{prefix}.{k}" if prefix else k
            if isinstance(v, dict) and k not in ("min", "max"):
                _trace(v, fk)
            else:
                if fk in sources:
                    full_sources[fk] = sources[fk]
                elif k in sources:
                    full_sources[fk] = sources[k]
                elif k in yaml_keys_formal or (prefix and prefix.split(".")[0] in yaml_keys_formal):
                    full_sources[fk] = "yaml"
                else:
                    full_sources[fk] = "default"
    _trace(config)

    config["_effective_config_sources"] = full_sources
    return config, full_sources


def main():
    parser = argparse.ArgumentParser(description="三相机点云融合 — Stable V1 calibrated_rig")
    parser.add_argument("--dataset", "-d", required=True, help="数据集路径")
    parser.add_argument("--rig", "-r", required=True, help="calibrated_rig.yaml 路径")
    parser.add_argument("--config", "-c", default=None, help="融合配置 YAML")
    parser.add_argument("--output", "-o", required=True, help="输出目录")
    parser.add_argument("--group-id", "-g", type=int, default=0, help="group ID")
    parser.add_argument("--depth-min", type=float, default=None, help="最小深度 (m)")
    parser.add_argument("--depth-max", type=float, default=None, help="最大深度 (m)")
    parser.add_argument("--voxel-size", type=float, default=None, help="体素尺寸 (m)")
    parser.add_argument("--roi-min", nargs=3, type=float, default=None, help="AABB 最小角 x y z")
    parser.add_argument("--roi-max", nargs=3, type=float, default=None, help="AABB 最大角 x y z")
    parser.add_argument("--registration-evidence", default=None, help="depth_registration.json 路径 (默认: group 目录下)")
    args = parser.parse_args()

    config, config_sources = _merge_config_with_cli(args.config, args)

    try:
        layout = resolve_capture_group(args.dataset, args.group_id)
    except FileNotFoundError as e:
        logger.error(str(e)); sys.exit(EXIT_DATA_FAIL)
    logger.info("group_dir: %s", layout.group_dir)

    rig = load_calibrated_rig(args.rig)
    if rig.get("status") != "PASS":
        logger.error("calibrated_rig status=%s, 期望 PASS", rig.get("status"))
        sys.exit(EXIT_DATA_FAIL)
    logger.info("rig_frame: %s, status: %s", rig["rig_frame"], rig["status"])

    # 加载 registration evidence
    evidence = None
    ev_path = args.registration_evidence or os.path.join(layout.group_dir, "depth_registration.json")
    evidence = load_registration_evidence(ev_path)
    if evidence:
        logger.info("registration evidence: %s (source=%s)", ev_path, evidence.get("source", "?"))
    else:
        logger.warning("无 depth_registration.json, color≠depth frame 配准将失败")

    manifest = load_group_manifest(layout.group_dir)

    rgbd_list = []
    for cam_name in REQUIRED_CAMERAS:
        cam_dir = layout.camera_dirs.get(cam_name) or os.path.join(layout.group_dir, cam_name)
        if not os.path.isdir(cam_dir):
            logger.error("相机目录不存在: %s", cam_dir); sys.exit(EXIT_DATA_FAIL)

        rgbd = load_rgbd_data(cam_dir, cam_name)
        rgbd = validate_rgbd_contract(rgbd, require_registered_to_color=True,
                                       registration_evidence=evidence)
        if not rgbd.is_valid:
            logger.error("%s 数据验证失败:", cam_name)
            for e in rgbd.errors: logger.error("  %s", e)
            sys.exit(EXIT_DATA_FAIL)
        for w in rgbd.warnings: logger.warning("  [%s] %s", cam_name, w)
        rgbd_list.append(rgbd)

    try:
        result = fuse_three_camera_pointclouds(rgbd_list, rig, config, args.output, allow_icp=False)
    except RuntimeError as e:
        logger.error("融合失败: %s", e); sys.exit(EXIT_INTERNAL)

    # ── Provenance ──
    prov_dir, env_info, input_files = _save_provenance(
        args.output, args, config, args.config, rig, args.rig,
        evidence, ev_path, layout, manifest)

    print("\n=== 融合统计 ===")
    for cam_name, stats in result["per_camera_stats"].items():
        print(f"  {cam_name}: raw={stats['n_raw']}, crop={stats['n_after_crop']}, final={stats['n_final']}, pixel_safe={stats.get('pixel_correspondence_safe','?')}")
    print(f"  fused: raw={result['fused_stats']['n_points_raw']}, dedup={result['fused_stats']['n_points_deduplicated']}")

    gate = result["gate"]
    gate_name = gate.get("gate_name", "LOCAL_ALIGNMENT_GATE")
    print(f"\n=== {gate_name}: {'PASS' if gate['passed'] else 'FAIL'} ===")
    if gate.get("provisional"):
        print("  ⚠️ 门限为 Gate 1 方向检查临时值, Oracle 完成后重新冻结")
    if gate.get("metric_scope", {}).get("selection_biased"):
        print("  ℹ️ 使用互为最近邻筛选, 存在 selection bias, 不可作为绝对精度评价")
    for r in gate.get("reasons", []): print(f"  {'❌' if 'FAIL' in str(r) else '⚠️'} {r}")
    for pk, pr in gate.get("pair_results", {}).items():
        flags = []
        if pr.get("low_overlap_warning"): flags.append("LOW_OVERLAP_SUPPORT")
        flag_str = f" [{', '.join(flags)}]" if flags else ""
        print(f"  {pk}: support={pr.get('support_passed')}, median={pr.get('median_passed')}, p95={pr.get('p95_passed')}, coverage={pr.get('coverage_passed')}{flag_str}")

    print("\n=== 输出文件 ===")
    for key, path in result["paths"].items():
        status = "✅" if path and os.path.isfile(path) else "❌ (skipped)"
        print(f"  {key}: {path or 'N/A'} {status}")
    for label, v in result.get("ply_verification", {}).items():
        print(f"  PLY verify [{label}]: exists={v['file_exists']}, size={v['file_size_bytes']}, readback={v['readback_points']}")

    # ── 增强 fusion_result.json ──
    result_path = os.path.join(args.output, "fusion_result.json")
    result_serializable = {
        "schema": "cr5_fusion_result_v2",
        "paths": result["paths"], "per_camera_stats": result["per_camera_stats"],
        "fused_stats": result["fused_stats"],
        "raw_pairwise_metrics": result["raw_pairwise_metrics"],
        "common_overlap_metrics": result["common_overlap_metrics"],
        "gate": gate, "ply_verification": result.get("ply_verification", {}),
        "rgb_output_suppressed": result.get("rgb_output_suppressed", False),
        "config_effective": result["config_effective"],
        "config_sources": config_sources,
        "provenance": {
            "git_sha": env_info["git_sha"],
            "git_branch": env_info["git_branch"],
            "config_path": os.path.abspath(args.config) if args.config else "N/A",
            "config_sha256": _sha256_file(args.config),
            "rig_path": os.path.abspath(args.rig),
            "rig_sha256": _sha256_file(args.rig),
            "stable_v1_json_sha256": rig.get("source_calibration", {}).get("sha256", "N/A"),
            "registration_evidence_path": os.path.abspath(ev_path) if evidence else "N/A",
            "registration_evidence_sha256": _sha256_file(ev_path) if evidence else "N/A",
            "group_manifest_sha256": _sha256_file(os.path.join(layout.group_dir, "group_manifest.json")),
            "input_files": input_files,
            "provenance_dir": prov_dir,
            "environment": env_info,
        },
        "dataset": os.path.abspath(args.dataset), "group_id": args.group_id,
        "manifest_available": manifest is not None,
        "registration_evidence_available": evidence is not None,
    }
    with open(result_path, "w") as f:
        json.dump(result_serializable, f, indent=2, default=str)

    print(f"\n📁 Provenance: {prov_dir}")
    if not gate["passed"]:
        print(f"\n❌ {gate_name} FAILED"); sys.exit(EXIT_GATE_FAIL)
    print(f"\n✅ 融合完成, {gate_name} PASS"); sys.exit(EXIT_OK)


if __name__ == "__main__":
    main()
