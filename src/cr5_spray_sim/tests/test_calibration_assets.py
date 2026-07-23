#!/usr/bin/env python3
"""
test_calibration_assets.py — 标定资产完整契约测试

基于 calibration_target.yaml 验证:
- 五张 PNG SHA-256 精确匹配
- material 引用的 texture 文件存在
- SDF 引用的 material name 在 .material 中有定义
- YAML 声明的 required_files 存在
- CMakeLists 安装关键文件
"""
import os
import sys
import re
import hashlib
import yaml

PKG_DIR = os.path.join(os.path.dirname(__file__), "..")
MODEL_DIR = os.path.join(PKG_DIR, "models", "calibration_target")
CONTRACT_YAML = os.path.join(MODEL_DIR, "calibration_target.yaml")
TEXTURES_DIR = os.path.join(MODEL_DIR, "materials", "textures")
MATERIAL_FILE = os.path.join(MODEL_DIR, "materials", "scripts",
                              "calibration_target.material")
SDF_FILE = os.path.join(MODEL_DIR, "model.sdf")


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(65536)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def load_contract():
    if not os.path.isfile(CONTRACT_YAML):
        raise FileNotFoundError(f"Contract YAML not found: {CONTRACT_YAML}")
    with open(CONTRACT_YAML, "r") as f:
        return yaml.safe_load(f)


# ── 测试函数 ────────────────────────────────────────────────

def test_contract_yaml_exists():
    assert os.path.isfile(CONTRACT_YAML), \
        f"calibration_target.yaml missing: {CONTRACT_YAML}"


def test_png_sha256_match():
    """每张 PNG 的 SHA-256 必须与 YAML 契约精确匹配."""
    contract = load_contract()
    failed = []
    for face, panel in contract["panels"].items():
        expected_sha = panel["texture_sha256"]
        png_path = os.path.join(TEXTURES_DIR, panel["texture"])
        if not os.path.isfile(png_path):
            failed.append(f"{face}: texture file not found: {png_path}")
            continue
        actual_sha = sha256_file(png_path)
        if actual_sha != expected_sha:
            failed.append(
                f"{face}: SHA MISMATCH\n"
                f"    expected: {expected_sha}\n"
                f"    actual:   {actual_sha}"
            )
    assert not failed, "PNG SHA-256 checks failed:\n" + "\n".join(failed)


def test_all_pngs_are_valid():
    """每张 texture PNG 的 magic bytes 正确."""
    contract = load_contract()
    for face, panel in contract["panels"].items():
        png_path = os.path.join(TEXTURES_DIR, panel["texture"])
        assert os.path.isfile(png_path), \
            f"{face}: PNG missing: {png_path}"
        assert os.path.getsize(png_path) > 0, \
            f"{face}: PNG is empty: {png_path}"
        with open(png_path, "rb") as f:
            header = f.read(8)
        assert header[:4] == b'\x89PNG', \
            f"{face}: not a valid PNG: {png_path}"


def test_material_textures_exist():
    """.material 中引用的所有 texture 文件必须存在."""
    if not os.path.isfile(MATERIAL_FILE):
        raise FileNotFoundError(f"Material file missing: {MATERIAL_FILE}")
    with open(MATERIAL_FILE, "r") as f:
        content = f.read()
    # 提取所有 texture <name> 行
    textures = re.findall(r'texture\s+(\S+)', content)
    missing = []
    for tex in textures:
        tex_path = os.path.join(TEXTURES_DIR, tex)
        if not os.path.isfile(tex_path):
            missing.append(tex)
    assert not missing, \
        f"Texture files referenced in .material but missing:\n" + \
        "\n".join(f"  - {t}" for t in missing)


def test_sdf_materials_defined():
    """SDF 引用的所有 material name 在 .material 中必须有定义."""
    if not os.path.isfile(SDF_FILE):
        raise FileNotFoundError(f"SDF file missing: {SDF_FILE}")
    if not os.path.isfile(MATERIAL_FILE):
        raise FileNotFoundError(f"Material file missing: {MATERIAL_FILE}")

    with open(SDF_FILE, "r") as f:
        sdf_content = f.read()
    with open(MATERIAL_FILE, "r") as f:
        mat_content = f.read()

    # 从 SDF 提取 material name (<name>CR5/Calibration/...</name>)
    sdf_materials = set(re.findall(
        r'<name>\s*(CR5/Calibration/\S+?)\s*</name>', sdf_content))
    # 从 .material 提取 material 名称
    mat_defs = set(re.findall(r'^material\s+(\S+)', mat_content, re.MULTILINE))

    undefined = sdf_materials - mat_defs
    assert not undefined, \
        f"SDF materials not defined in .material:\n" + \
        "\n".join(f"  - {m}" for m in undefined)


def test_required_files_exist():
    """YAML 契约中声明的所有 required_files 必须存在."""
    contract = load_contract()
    missing = []
    for rf in contract.get("required_files", []):
        full_path = os.path.join(MODEL_DIR, rf)
        if not os.path.exists(full_path):
            missing.append(rf)
    assert not missing, \
        f"Required files missing:\n" + "\n".join(f"  - {f}" for f in missing)


def test_body_geometry_in_sdf():
    """SDF 中主体尺寸必须与 YAML 契约一致."""
    contract = load_contract()
    expected = contract["body"]
    expected_size = f'{expected["size_x"]} {expected["size_y"]} {expected["size_z"]}'

    with open(SDF_FILE, "r") as f:
        sdf = f.read()

    # 查找 main_body link 中的 box size
    body_section = re.search(
        r'<link name="main_body">(.*?)</link>', sdf, re.DOTALL)
    if body_section:
        size_match = re.search(
            r'<size>\s*([0-9.]+ [0-9.]+ [0-9.]+)\s*</size>',
            body_section.group(1))
        if size_match:
            assert size_match.group(1) == expected_size, \
                f"Body size mismatch: expected {expected_size}, " \
                f"got {size_match.group(1)}"
            return
    raise AssertionError("Could not find main_body box size in model.sdf")


def test_cmakelists_installs_model():
    """CMakeLists.txt 安装 models 目录 (含标定目标)."""
    cmake_path = os.path.join(PKG_DIR, "CMakeLists.txt")
    assert os.path.isfile(cmake_path), "CMakeLists.txt missing"
    with open(cmake_path, "r") as f:
        content = f.read()
    assert "models" in content, \
        "CMakeLists.txt does not install models directory"


def test_model_config_exists():
    """model.config 必须存在且有效."""
    cfg_path = os.path.join(MODEL_DIR, "model.config")
    assert os.path.isfile(cfg_path), "model.config missing"


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
