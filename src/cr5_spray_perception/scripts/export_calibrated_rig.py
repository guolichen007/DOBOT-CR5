#!/usr/bin/env python3
"""
CR5 Reconstruction — Stable V1 外参桥接脚本.

从 Stable V1 camera_extrinsics.json 生成 calibrated_rig.yaml.

本脚本只能做格式转换和元数据补充，禁止:
  - 优化、平均、ICP 或修改外参数值
  - 导入 gazebo_msgs / tf2_ros / model_states

用法:
  rosrun cr5_spray_perception export_calibrated_rig.py \
    --input ~/cr5_data/calibration/camera_extrinsics.json \
    --output ~/cr5_data/reconstruction/calibrated_rig.yaml
"""

import os, sys, argparse, json, yaml, logging
import numpy as np

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("export_calibrated_rig")

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.extrinsics import (
    load_stable_v1_extrinsics, build_calibrated_rig, sha256_file,
    validate_stable_v1_json,
)
from cr5_spray_perception.reconstruction.contracts import validate_calibrated_rig_yaml


def main():
    parser = argparse.ArgumentParser(
        description="Stable V1 外参 → calibrated_rig.yaml 桥接")
    parser.add_argument("--input", "-i", required=True,
                        help="Stable V1 camera_extrinsics.json 路径")
    parser.add_argument("--output", "-o", required=True,
                        help="输出 calibrated_rig.yaml 路径")
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        logger.error("输入文件不存在: %s", args.input)
        sys.exit(1)

    # Step 1: 校验 Stable V1 JSON
    logger.info("Step 1: 校验 Stable V1 JSON schema...")
    with open(args.input, "r") as f:
        raw_data = json.load(f)
    passed, errors = validate_stable_v1_json(raw_data)
    if not passed:
        logger.error("Stable V1 JSON 校验失败:")
        for e in errors:
            logger.error("  %s", e)
        sys.exit(1)
    logger.info("  ✅ solver=%s version=%s status=%s",
                raw_data["solver"], raw_data["version"], raw_data["status"])

    # Step 2: 加载外参矩阵
    logger.info("Step 2: 加载外参矩阵...")
    cameras = load_stable_v1_extrinsics(args.input)
    for cam, T in cameras.items():
        t = T[:3, 3]
        logger.info("  %s: T=[%.4f, %.4f, %.4f]", cam, t[0], t[1], t[2])

    # Step 3: 构建 calibrated_rig
    logger.info("Step 3: 构建 calibrated_rig YAML...")
    rig = build_calibrated_rig(args.input)

    # Step 4: 校验输出 schema
    logger.info("Step 4: 校验输出 schema...")
    passed, errors = validate_calibrated_rig_yaml(rig)
    if not passed:
        logger.error("calibrated_rig YAML 校验失败:")
        for e in errors:
            logger.error("  %s", e)
        sys.exit(1)
    logger.info("  ✅ schema_version=%s status=%s", rig["schema_version"], rig["status"])

    # Step 5: 写入 YAML
    logger.info("Step 5: 写入 %s ...", args.output)
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)

    # NumPy 数组转 list 以便 YAML 序列化
    def numpy_to_list(obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, dict):
            return {k: numpy_to_list(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [numpy_to_list(v) for v in obj]
        return obj

    rig_serializable = numpy_to_list(rig)

    # 自定义 YAML 输出格式
    class MatrixListDumper(yaml.Dumper):
        pass

    def list_representer(dumper, data):
        # 对于矩阵行列表 (4 个 4 元素列表), 使用 flow style
        if len(data) == 4 and all(isinstance(row, list) and len(row) == 4 for row in data):
            return dumper.represent_sequence(
                'tag:yaml.org,2002:seq', data, flow_style=True)
        return dumper.represent_sequence('tag:yaml.org,2002:seq', data)

    MatrixListDumper.add_representer(list, list_representer)

    with open(args.output, "w") as f:
        yaml.dump(rig_serializable, f, Dumper=MatrixListDumper,
                  default_flow_style=False, allow_unicode=True, sort_keys=False, width=120)

    sha = sha256_file(args.output)
    logger.info("✅ 完成!")
    logger.info("  INPUT:  %s", os.path.abspath(args.input))
    logger.info("  OUTPUT: %s", os.path.abspath(args.output))
    logger.info("  SHA256: %s", sha)

    # 输出摘要
    print("\n=== 外参桥接摘要 ===")
    print(f"schema_version: {rig['schema_version']}")
    print(f"status:         {rig['status']}")
    print(f"rig_frame:      {rig['rig_frame']}")
    print(f"source:         {rig['source_calibration']['file']}")
    print(f"source_sha256:  {rig['source_calibration']['sha256']}")
    print()
    for cam_name in ["cam_front_left", "cam_front_right", "cam_rear"]:
        cam = rig["cameras"][cam_name]
        T = np.array(cam["T_rig_camera"])
        t = T[:3, 3]
        print(f"{cam_name}:")
        print(f"  optical_frame: {cam['optical_frame']}")
        print(f"  T_rig_camera:  t=[{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]")


if __name__ == "__main__":
    main()
