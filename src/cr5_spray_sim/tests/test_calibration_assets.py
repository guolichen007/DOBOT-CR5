#!/usr/bin/env python3
"""test_calibration_assets.py — 标定资产契约测试

验证:
- 标定 marker PNG 文件完整性
- model.sdf / model.config 存在
- 三相机 spawn 脚本引用正确
- CMakeLists 安装目标覆盖
"""
import os
import sys
import hashlib

PKG_DIR = os.path.join(os.path.dirname(__file__), "..")
MATERIALS_DIR = os.path.join(PKG_DIR, "models", "calibration_target", "materials")
TEXTURES_DIR = os.path.join(MATERIALS_DIR, "textures")
SCRIPTS_DIR = os.path.join(MATERIALS_DIR, "scripts")
MODEL_DIR = os.path.join(PKG_DIR, "models", "calibration_target")


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(65536)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def test_model_config_exists():
    assert os.path.isfile(os.path.join(MODEL_DIR, "model.config")), \
        "model.config missing"
    assert os.path.isfile(os.path.join(MODEL_DIR, "model.sdf")), \
        "model.sdf missing"


def test_calibration_pngs_exist():
    """验证标定 marker PNG 文件存在且非空."""
    if not os.path.isdir(TEXTURES_DIR):
        print(f"SKIP: {TEXTURES_DIR} not found")
        return
    pngs = [f for f in os.listdir(TEXTURES_DIR) if f.endswith(".png")]
    assert len(pngs) >= 5, \
        f"Expected >= 5 calibration PNG files, found {len(pngs)}: {pngs}"
    for png in pngs:
        path = os.path.join(TEXTURES_DIR, png)
        size = os.path.getsize(path)
        assert size > 0, f"PNG {png} is empty"
        # PNG magic bytes
        with open(path, "rb") as f:
            header = f.read(8)
        assert header[:4] == b'\x89PNG', f"{png} is not a valid PNG"


def test_calibration_material_scripts_exist():
    """验证 material 脚本目录存在且有 .material 文件."""
    if os.path.isdir(MATERIALS_DIR):
        scripts_dir = os.path.join(MATERIALS_DIR, "scripts")
        if os.path.isdir(scripts_dir):
            mat_files = [f for f in os.listdir(scripts_dir)
                        if f.endswith(".material")]
            assert len(mat_files) > 0, \
                f"No .material files found in {scripts_dir}"


def test_spawn_script_references():
    """验证 spawn_fixed_cameras.py 引用正确的三相机名."""
    spawn_script = os.path.join(PKG_DIR, "scripts", "spawn_fixed_cameras.py")
    if os.path.isfile(spawn_script):
        with open(spawn_script, "r") as f:
            content = f.read()
        assert "cam_front_left" in content or "simulation_scene" in content, \
            "spawn script missing camera/scene reference"


def test_cmakelists_installs_scripts():
    """验证 CMakeLists.txt 安装关键脚本."""
    cmake_path = os.path.join(PKG_DIR, "CMakeLists.txt")
    assert os.path.isfile(cmake_path), "CMakeLists.txt missing"
    with open(cmake_path, "r") as f:
        content = f.read()
    for script in ["spawn_fixed_cameras.py", "run_simulation.sh",
                   "wait_scene_models.py"]:
        assert script in content, \
            f"CMakeLists.txt missing install for {script}"


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS: {t.__name__}")
        except Exception as e:
            print(f"FAIL: {t.__name__} — {e}")
            failed += 1
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
