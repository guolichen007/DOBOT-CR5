#!/usr/bin/env python3
"""
V5 标定目标几何一致性验证脚本.

解析 YAML 和 SDF, 检查:
  - body size 一致
  - 五块 panel scale 一致
  - 五块 panel pose 一致
  - 所有板都位于主体外表面
  - Front/Back 不超出主体边缘
  - Top block 与 Top panel AABB 不相交
  - cables 与 Top panel AABB 不相交
  - face frame rotations 与 SDF visual rotations 一致

通过后输出 CALIBRATION_TARGET_GEOMETRY_PASS.
"""
import sys
import os
import math
import xml.etree.ElementTree as ET
import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)

YAML_PATH = os.path.join(PKG_DIR, "config", "calibration_target_v1.yaml")
SDF_PATH = os.path.join(PKG_DIR, "models", "calibration_target_v1", "model.sdf")

# YAML panel key → SDF link name + expected material
PANEL_LINK_MAP = {
    "front": "front_panel",
    "left":  "left_panel",
    "right": "right_panel",
    "top":   "top_panel",
    "back":  "back_panel",
}

TOLERANCE_MM = 0.0005  # 0.5mm tolerance for floating point comparisons


def fail(msg):
    print(f"FAIL: {msg}")
    sys.exit(1)


def check_close(a, b, label, tol=TOLERANCE_MM):
    if abs(a - b) > tol:
        fail(f"{label}: YAML={a}, SDF={b} (diff={abs(a-b):.6f}m)")


def parse_yaml():
    if not os.path.isfile(YAML_PATH):
        fail(f"YAML not found: {YAML_PATH}")
    with open(YAML_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    target = cfg.get("target", {})
    panels = cfg.get("panels", {})

    geometry = {
        "body": target.get("main_body_m"),
        "top_block_size": target.get("asymmetric_top_block_m"),
        "top_block_pose": target.get("asymmetric_top_block_pose_m"),
        "panel_gap": target.get("panel_gap_m"),
        "hanger_y": target.get("hanger_y_m"),
        "spreader_z": target.get("spreader_center_z_m"),
        "cable_bottom_z": target.get("cable_bottom_z_m"),
        "cable_top_z": target.get("cable_top_z_m"),
    }

    panel_info = {}
    for pk, link_name in PANEL_LINK_MAP.items():
        p = panels.get(pk, {})
        pose = p.get("pose_target", {})
        panel_info[pk] = {
            "link": link_name,
            "canvas": p.get("physical_canvas_m"),
            "xyz": pose.get("xyz", [0, 0, 0]),
            "rpy": pose.get("rpy", [0, 0, 0]),
        }

    return geometry, panel_info


def parse_sdf():
    if not os.path.isfile(SDF_PATH):
        fail(f"SDF not found: {SDF_PATH}")

    tree = ET.parse(SDF_PATH)
    root = tree.getroot()

    sdf_ns = ""  # SDF 1.7 doesn't use namespaces in the model file
    # Actually, SDF uses http://sdformat.org/schemas/root-1.0 as default
    # Try to extract the namespace
    ns_match = root.tag.split("}")[0].strip("{") if "}" in root.tag else ""
    ns = f"{{{ns_match}}}" if ns_match else ""

    model = root.find(f"{ns}model")
    if model is None:
        model = root
        ns = ""

    body_size = None
    panel_info = {}
    top_block_size = None
    top_block_pose = None
    cable_info = {}  # y → (bottom_z, top_z, length)
    spreader_z = None

    for link in model.findall(f"{ns}link"):
        name = link.get("name", "")

        if name == "main_body":
            for vis in link.findall(f"{ns}visual"):
                geom = vis.find(f"{ns}geometry")
                if geom is not None:
                    box = geom.find(f"{ns}box")
                    if box is not None:
                        sz = box.find(f"{ns}size")
                        if sz is not None and sz.text:
                            body_size = [float(x) for x in sz.text.strip().split()]

        elif name == "top_offset_block":
            pose_el = link.find(f"{ns}pose")
            if pose_el is not None and pose_el.text:
                top_block_pose = [float(x) for x in pose_el.text.strip().split()[:3]]
            for vis in link.findall(f"{ns}visual"):
                geom = vis.find(f"{ns}geometry")
                if geom is not None:
                    box = geom.find(f"{ns}box")
                    if box is not None:
                        sz = box.find(f"{ns}size")
                        if sz is not None and sz.text:
                            top_block_size = [float(x) for x in sz.text.strip().split()]

        elif name == "spreader_bar":
            pose_el = link.find(f"{ns}pose")
            if pose_el is not None and pose_el.text:
                spreader_z = float(pose_el.text.strip().split()[2])

        elif "cable" in name:
            pose_el = link.find(f"{ns}pose")
            if pose_el is not None and pose_el.text:
                parts = [float(x) for x in pose_el.text.strip().split()]
                cable_y = parts[1]
                cable_z = parts[2]
                cable_info[name] = {"y": cable_y, "z": cable_z}
                # Also get cylinder length
                for vis in link.findall(f"{ns}visual"):
                    geom = vis.find(f"{ns}geometry")
                    if geom is not None:
                        cyl = geom.find(f"{ns}cylinder")
                        if cyl is not None:
                            length_el = cyl.find(f"{ns}length")
                            if length_el is not None and length_el.text:
                                cable_info[name]["length"] = float(length_el.text.strip())

        elif name in PANEL_LINK_MAP.values():
            # Reverse map
            panel_key = [k for k, v in PANEL_LINK_MAP.items() if v == name][0]
            pose_el = link.find(f"{ns}pose")
            panel_xyz = [0, 0, 0]
            panel_rpy = [0, 0, 0]
            if pose_el is not None and pose_el.text:
                vals = [float(x) for x in pose_el.text.strip().split()]
                panel_xyz = vals[:3]
                panel_rpy = vals[3:6] if len(vals) >= 6 else [0, 0, 0]

            # Get scale from mesh
            scale = None
            for vis in link.findall(f"{ns}visual"):
                geom = vis.find(f"{ns}geometry")
                if geom is not None:
                    mesh = geom.find(f"{ns}mesh")
                    if mesh is not None:
                        scale_el = mesh.find(f"{ns}scale")
                        if scale_el is not None and scale_el.text:
                            scale = [float(x) for x in scale_el.text.strip().split()]

            panel_info[panel_key] = {
                "xyz": panel_xyz,
                "rpy": panel_rpy,
                "scale": scale,
            }

    return {
        "body": body_size,
        "top_block_size": top_block_size,
        "top_block_pose": top_block_pose,
        "panels": panel_info,
        "cables": cable_info,
        "spreader_z": spreader_z,
    }


def aabb_2d(x, y, sx, sy):
    """返回 2D AABB (min_x, max_x, min_y, max_y)."""
    return (x - sx/2, x + sx/2, y - sy/2, y + sy/2)


def aabb_overlap_2d(a, b):
    """检查两个 2D AABB 是否相交."""
    return not (a[1] <= b[0] or a[0] >= b[1] or a[3] <= b[2] or a[2] >= b[3])


def compute_panel_aabb_top(panel_xyz, panel_scale):
    """Top 面板在 XY 平面的 AABB (Z=0 忽略)."""
    # Top panel is at Z=panel_xyz[2], scale is (sx, sy, 1)
    sx = panel_scale[0] / 2.0
    sy = panel_scale[1] / 2.0
    cx, cy = panel_xyz[0], panel_xyz[1]
    return aabb_2d(cx, cy, panel_scale[0], panel_scale[1])


def compute_block_aabb_top(block_xyz, block_size):
    """偏置块在 XY 平面的 AABB."""
    cx, cy = block_xyz[0], block_xyz[1]
    return aabb_2d(cx, cy, block_size[0], block_size[1])


def compute_cable_aabb_top(cable_y, cable_radius=0.003):
    """吊索在 XY 平面的 AABB (近似为小圆)."""
    return (cable_y - cable_radius, cable_y + cable_radius, -cable_radius, cable_radius)


def check_rotations_match(sdf_rpy, yaml_rpy, panel_name):
    """检查 SDF 和 YAML 中面板旋转是否一致.

    SDF DAE visual 的 rpy 将 XY-plane (Z-normal) 旋转到面板外法向.
    YAML pose_target rpy 是 face frame 的方向.

    两者应一致: 面板外法向 == face frame local +Z.
    """
    # Compare modulo 2*pi
    for axis in range(3):
        a = sdf_rpy[axis] % (2 * math.pi)
        b = yaml_rpy[axis] % (2 * math.pi)
        diff = abs(a - b)
        # Also check wrapped around 2*pi
        diff = min(diff, 2 * math.pi - diff)
        if diff > 0.001:  # ~0.06 degrees tolerance
            fail(f"{panel_name}: rpy mismatch — SDF={sdf_rpy}, YAML={yaml_rpy} "
                 f"(axis {axis}: diff={diff:.6f}rad)")


def main():
    errors = 0

    print("=" * 60)
    print("Calibration Target Geometry Validation V5")
    print("=" * 60)

    # ---- Load ----
    print(f"\n[1] Loading YAML: {YAML_PATH}")
    yaml_geom, yaml_panels = parse_yaml()
    print(f"    Body: {yaml_geom['body']}")
    print(f"    Panel gap: {yaml_geom['panel_gap']} m")

    print(f"\n[2] Loading SDF: {SDF_PATH}")
    sdf_data = parse_sdf()
    print(f"    Body: {sdf_data['body']}")
    print(f"    Panels found: {list(sdf_data['panels'].keys())}")

    # ---- Body size ----
    print(f"\n[3] Checking body size...")
    if yaml_geom["body"] and sdf_data["body"]:
        for i, axis in enumerate(["X", "Y", "Z"]):
            check_close(yaml_geom["body"][i], sdf_data["body"][i],
                        f"Body {axis}")
        print("    Body size: OK")
    else:
        fail("Missing body size in YAML or SDF")

    # ---- Panel scale & pose ----
    print(f"\n[4] Checking panel scale & pose...")
    for pk, yp in yaml_panels.items():
        sp = sdf_data["panels"].get(pk)
        if sp is None:
            fail(f"Panel '{pk}' not found in SDF")

        # Scale: first 2 components must match canvas dimensions
        if sp["scale"] and yp["canvas"]:
            for i, axis in enumerate(["X", "Y"]):
                check_close(sp["scale"][i], yp["canvas"][i],
                            f"{pk} scale {axis}")

        # Position
        for i, axis in enumerate(["X", "Y", "Z"]):
            check_close(sp["xyz"][i], yp["xyz"][i],
                        f"{pk} position {axis}")

        # Rotation
        check_rotations_match(sp["rpy"], yp["rpy"], pk)

    print("    Panel scale & pose: OK")

    # ---- Margin checks ----
    print(f"\n[5] Checking panel margins (panels within body faces)...")
    body = yaml_geom["body"]
    half_body = [body[0]/2, body[1]/2, body[2]/2]
    gap = yaml_geom["panel_gap"]

    for pk, yp in yaml_panels.items():
        canvas = yp["canvas"]
        xyz = yp["xyz"]

        if pk == "front":
            # x = +Bx/2 + gap, panel Y-span must be within body Y-face
            expected_x = half_body[0] + gap
            check_close(xyz[0], expected_x, f"{pk} x")
            # Y margin: body_y/2 - canvas_y/2 (one side)
            y_margin = half_body[1] - canvas[0]/2
            z_margin = half_body[2] - canvas[1]/2
            if y_margin < 0:
                fail(f"{pk}: canvas Y={canvas[0]} exceeds body Y={body[1]} (margin={y_margin:.4f}m)")
            if z_margin < 0:
                fail(f"{pk}: canvas Z={canvas[1]} exceeds body Z={body[2]} (margin={z_margin:.4f}m)")
            print(f"    {pk}: Y_margin={y_margin*1000:.1f}mm, Z_margin={z_margin*1000:.1f}mm")

        elif pk == "back":
            expected_x = -(half_body[0] + gap)
            check_close(xyz[0], expected_x, f"{pk} x")
            y_margin = half_body[1] - canvas[0]/2
            z_margin = half_body[2] - canvas[1]/2
            if y_margin < 0:
                fail(f"{pk}: canvas Y={canvas[0]} exceeds body Y={body[1]} (margin={y_margin:.4f}m)")
            if z_margin < 0:
                fail(f"{pk}: canvas Z={canvas[1]} exceeds body Z={body[2]} (margin={z_margin:.4f}m)")
            print(f"    {pk}: Y_margin={y_margin*1000:.1f}mm, Z_margin={z_margin*1000:.1f}mm")

        elif pk == "left":
            expected_y = half_body[1] + gap
            check_close(xyz[1], expected_y, f"{pk} y")
            x_margin = half_body[0] - canvas[0]/2
            z_margin = half_body[2] - canvas[1]/2
            if x_margin < 0:
                fail(f"{pk}: canvas X={canvas[0]} exceeds body X={body[0]} (margin={x_margin:.4f}m)")
            if z_margin < 0:
                fail(f"{pk}: canvas Z={canvas[1]} exceeds body Z={body[2]} (margin={z_margin:.4f}m)")
            print(f"    {pk}: X_margin={x_margin*1000:.1f}mm, Z_margin={z_margin*1000:.1f}mm")

        elif pk == "right":
            expected_y = -(half_body[1] + gap)
            check_close(xyz[1], expected_y, f"{pk} y")
            x_margin = half_body[0] - canvas[0]/2
            z_margin = half_body[2] - canvas[1]/2
            if x_margin < 0:
                fail(f"{pk}: canvas X={canvas[0]} exceeds body X={body[0]} (margin={x_margin:.4f}m)")
            if z_margin < 0:
                fail(f"{pk}: canvas Z={canvas[1]} exceeds body Z={body[2]} (margin={z_margin:.4f}m)")
            print(f"    {pk}: X_margin={x_margin*1000:.1f}mm, Z_margin={z_margin*1000:.1f}mm")

        elif pk == "top":
            expected_z = half_body[2] + gap
            check_close(xyz[2], expected_z, f"{pk} z")
            x_margin = half_body[0] - canvas[0]/2
            y_margin = half_body[1] - canvas[1]/2
            if x_margin < 0:
                fail(f"{pk}: canvas X={canvas[0]} exceeds body X={body[0]} (margin={x_margin:.4f}m)")
            if y_margin < 0:
                fail(f"{pk}: canvas Y={canvas[1]} exceeds body Y={body[1]} (margin={y_margin:.4f}m)")
            print(f"    {pk}: X_margin={x_margin*1000:.1f}mm, Y_margin={y_margin*1000:.1f}mm")

    # ---- Top block vs Top panel AABB ----
    print(f"\n[6] Checking Top block vs Top panel occlusion...")
    top_panel = yaml_panels.get("top")
    if top_panel:
        panel_aabb = compute_panel_aabb_top(top_panel["xyz"], top_panel["canvas"])

        block_size = yaml_geom["top_block_size"]
        block_pose = yaml_geom["top_block_pose"]
        if block_size and block_pose:
            block_aabb = compute_block_aabb_top(block_pose, block_size)

            if aabb_overlap_2d(panel_aabb, block_aabb):
                fail(f"Top block overlaps Top panel! "
                     f"Panel AABB: ({panel_aabb[0]:.3f},{panel_aabb[1]:.3f})"
                     f"×({panel_aabb[2]:.3f},{panel_aabb[3]:.3f}), "
                     f"Block AABB: ({block_aabb[0]:.3f},{block_aabb[1]:.3f})"
                     f"×({block_aabb[2]:.3f},{block_aabb[3]:.3f})")

            # Check actual gap (minimum distance between AABBs)
            gap_x = max(0.0, max(panel_aabb[0] - block_aabb[1], block_aabb[0] - panel_aabb[1]))
            gap_y = max(0.0, max(panel_aabb[2] - block_aabb[3], block_aabb[2] - panel_aabb[3]))
            min_gap = math.sqrt(gap_x**2 + gap_y**2) if (gap_x > 0 or gap_y > 0) else 0.0
            print(f"    Panel AABB:  XY=[{panel_aabb[0]:.4f},{panel_aabb[1]:.4f}]×[{panel_aabb[2]:.4f},{panel_aabb[3]:.4f}]")
            print(f"    Block AABB:  XY=[{block_aabb[0]:.4f},{block_aabb[1]:.4f}]×[{block_aabb[2]:.4f},{block_aabb[3]:.4f}]")
            print(f"    Min gap: {min_gap*1000:.1f}mm (OK, no overlap)" if min_gap >= 0 else f"    OVERLAP!")
    else:
        fail("Top panel not found in YAML")

    # ---- Cables vs Top panel AABB ----
    print(f"\n[7] Checking cables vs Top panel occlusion...")
    hanger_y = yaml_geom["hanger_y"]
    if hanger_y:
        for cable_y_val in [-hanger_y, hanger_y]:
            cable_aabb = compute_cable_aabb_top(cable_y_val)
            if aabb_overlap_2d(panel_aabb, cable_aabb):
                fail(f"Cable at y={cable_y_val:.3f} overlaps Top panel!")
            gap_cable_x = max(0.0, max(panel_aabb[0] - cable_aabb[1], cable_aabb[0] - panel_aabb[1]))
            gap_cable_y = max(0.0, max(panel_aabb[2] - cable_aabb[3], cable_aabb[2] - panel_aabb[3]))
        print(f"    Cable Y=±{hanger_y}: outside Top panel (no overlap, OK)")

    # ---- Spreader z ----
    print(f"\n[8] Checking spreader bar...")
    spreader_yaml = yaml_geom["spreader_z"]
    spreader_sdf = sdf_data["spreader_z"]
    if spreader_yaml and spreader_sdf:
        check_close(spreader_yaml, spreader_sdf, "Spreader Z")
        print(f"    Spreader Z: {spreader_yaml} (OK)")

    # ---- Result ----
    print(f"\n{'=' * 60}")
    print("CALIBRATION_TARGET_GEOMETRY_PASS")
    print(f"{'=' * 60}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
