"""
CR5 Reconstruction — 数据集目录布局解析.

提供统一的路径解析函数, 支持三种输入格式:
  1. run 根目录:     <run_root>/groups/group_XXXX/<camera>/
  2. groups 根目录:   <groups_root>/group_XXXX/<camera>/
  3. 直接 group 目录: <any_path>/group_XXXX/<camera>/

同样支持 views 格式:
  1. <run_root>/views/view_XXXX/<camera>/
  2. <views_root>/view_XXXX/<camera>/
  3. <any_path>/view_XXXX/<camera>/

用法:
  from cr5_spray_perception.reconstruction.dataset_layout import resolve_capture_group
  layout = resolve_capture_group("/path/to/dataset", group_id=0)
  # layout.group_dir  → 实际 group 目录
  # layout.run_root   → run 根目录 (如有)
  # layout.layout_type → "groups" | "views"
"""

import os, json, logging
from dataclasses import dataclass, field
from typing import Optional, List

logger = logging.getLogger(__name__)


@dataclass
class DatasetLayout:
    """数据集目录布局解析结果."""
    group_dir: str                          # 实际 group/view 目录
    group_id: int                           # group ID
    layout_type: str                        # "groups" | "views"
    run_root: str = ""                      # run 根目录 (如可确定)
    groups_root: str = ""                   # groups/views 根目录
    manifest_path: str = ""                 # group_manifest.json 路径
    camera_dirs: dict = field(default_factory=dict)  # {cam_name: cam_dir}

    @property
    def is_valid(self) -> bool:
        return os.path.isdir(self.group_dir)

    @property
    def has_manifest(self) -> bool:
        return os.path.isfile(self.manifest_path)


def _find_group_dir(base_path: str, group_id: int, prefix: str) -> Optional[str]:
    """在 base_path 下查找 group_XXXX 目录.

    Args:
        base_path: 搜索根目录.
        group_id: group ID.
        prefix: "group" 或 "view".

    Returns:
        找到的目录路径, 或 None.
    """
    group_name = f"{prefix}_{group_id:04d}"
    candidate = os.path.join(base_path, group_name)
    if os.path.isdir(candidate):
        return candidate

    # 尝试宽松匹配 (group_0, group_00 等)
    if os.path.isdir(base_path):
        for entry in sorted(os.listdir(base_path)):
            full = os.path.join(base_path, entry)
            if not os.path.isdir(full):
                continue
            # 精确匹配 group_X 或 view_X
            if entry == group_name:
                return full
    return None


def resolve_capture_group(dataset_path: str, group_id: int = 0) -> DatasetLayout:
    """统一解析数据集路径, 支持 run/groups/group 三种输入.

    解析优先级:
      1. dataset_path 本身是 group_XXXX 目录 → 直接使用
      2. dataset_path/groups/group_XXXX → run 根目录模式
      3. dataset_path/group_XXXX → groups 根目录模式
      4. dataset_path/views/view_XXXX → views 模式 (同 2)
      5. dataset_path/view_XXXX → views 根目录模式 (同 3)

    Args:
        dataset_path: 用户传入的数据集路径.
        group_id: group ID (默认 0).

    Returns:
        DatasetLayout 实例.

    Raises:
        FileNotFoundError: 无法解析到有效的 group 目录.
    """
    dataset_path = os.path.expanduser(os.path.abspath(dataset_path))

    if not os.path.exists(dataset_path):
        raise FileNotFoundError(f"数据集路径不存在: {dataset_path}")

    # 尝试 1: dataset_path 本身是 group_XXXX 或 view_XXXX
    basename = os.path.basename(dataset_path.rstrip("/"))
    for prefix in ["group", "view"]:
        if basename.startswith(f"{prefix}_"):
            # 直接是 group 目录
            try:
                gid = int(basename.split("_")[1])
            except (ValueError, IndexError):
                gid = group_id
            manifest = os.path.join(dataset_path, "group_manifest.json")
            # 扫描相机子目录
            camera_dirs = _scan_camera_dirs(dataset_path)
            return DatasetLayout(
                group_dir=dataset_path,
                group_id=gid,
                layout_type="groups" if prefix == "group" else "views",
                run_root=os.path.dirname(os.path.dirname(dataset_path))
                    if os.path.basename(os.path.dirname(dataset_path)) in ("groups", "views")
                    else "",
                groups_root=os.path.dirname(dataset_path)
                    if os.path.basename(os.path.dirname(dataset_path)) in ("groups", "views")
                    else dataset_path,
                manifest_path=manifest,
                camera_dirs=camera_dirs,
            )

    # 尝试 2: dataset_path 包含 groups/ 或 views/ 子目录 (run root 模式)
    for prefix in ["groups", "views"]:
        sub_dir = os.path.join(dataset_path, prefix)
        if os.path.isdir(sub_dir):
            gname_prefix = "group" if prefix == "groups" else "view"
            group_dir = _find_group_dir(sub_dir, group_id, gname_prefix)
            if group_dir is not None:
                manifest = os.path.join(group_dir, "group_manifest.json")
                camera_dirs = _scan_camera_dirs(group_dir)
                return DatasetLayout(
                    group_dir=group_dir,
                    group_id=group_id,
                    layout_type=prefix,
                    run_root=dataset_path,
                    groups_root=sub_dir,
                    manifest_path=manifest,
                    camera_dirs=camera_dirs,
                )

            # 尝试列出 groups 目录中的内容
            entries = sorted([
                d for d in os.listdir(sub_dir)
                if os.path.isdir(os.path.join(sub_dir, d))
            ])
            if entries:
                logger.warning(
                    "在 %s 中未找到 %s_%04d, 可用: %s",
                    sub_dir, gname_prefix, group_id, entries[:10])

    # 尝试 3: dataset_path 下直接有 group_XXXX (groups root 模式)
    for gname_prefix in ["group", "view"]:
        prefix = "groups" if gname_prefix == "group" else "views"
        group_dir = _find_group_dir(dataset_path, group_id, gname_prefix)
        if group_dir is not None:
            manifest = os.path.join(group_dir, "group_manifest.json")
            camera_dirs = _scan_camera_dirs(group_dir)
            return DatasetLayout(
                group_dir=group_dir,
                group_id=group_id,
                layout_type=prefix,
                run_root="",
                groups_root=dataset_path,
                manifest_path=manifest,
                camera_dirs=camera_dirs,
            )

    # 全部失败
    tried = [
        f"  {dataset_path}/groups/group_{group_id:04d}",
        f"  {dataset_path}/group_{group_id:04d}",
        f"  {dataset_path}/views/view_{group_id:04d}",
        f"  {dataset_path}/view_{group_id:04d}",
    ]
    raise FileNotFoundError(
        f"无法解析数据集路径 '{dataset_path}' 到 group_{group_id:04d}.\n"
        f"尝试了:\n" + "\n".join(tried) +
        f"\n请确认目录结构为以下之一:"
        f"\n  1) <run>/groups/group_{group_id:04d}/<camera>/"
        f"\n  2) <groups>/group_{group_id:04d}/<camera>/"
        f"\n  3) <any>/group_{group_id:04d}/<camera>/"
    )


def _scan_camera_dirs(group_dir: str) -> dict:
    """扫描 group 目录下的相机子目录.

    Args:
        group_dir: group 目录路径.

    Returns:
        {cam_name: cam_dir_path} 字典.
    """
    camera_dirs = {}
    if not os.path.isdir(group_dir):
        return camera_dirs
    for entry in sorted(os.listdir(group_dir)):
        full = os.path.join(group_dir, entry)
        if os.path.isdir(full) and entry.startswith("cam_"):
            camera_dirs[entry] = full
    return camera_dirs


def load_group_manifest(group_dir: str) -> Optional[dict]:
    """加载 group_manifest.json.

    Args:
        group_dir: group 目录路径.

    Returns:
        manifest dict, 或 None (文件不存在或解析失败).
    """
    manifest_path = os.path.join(group_dir, "group_manifest.json")
    if not os.path.isfile(manifest_path):
        return None
    try:
        with open(manifest_path, "r") as f:
            return json.load(f)
    except (json.JSONDecodeError, IOError) as e:
        logger.warning("无法解析 group_manifest.json: %s", e)
        return None


def list_available_groups(run_or_groups_dir: str, layout_type: str = "groups") -> List[int]:
    """列出可用的 group ID.

    Args:
        run_or_groups_dir: run 根目录或 groups 根目录.
        layout_type: "groups" 或 "views".

    Returns:
        排序的 group ID 列表.
    """
    prefix = "group" if layout_type == "groups" else "view"
    sub_dir_name = layout_type

    # 尝试 run root 模式
    candidate = os.path.join(run_or_groups_dir, sub_dir_name)
    if not os.path.isdir(candidate):
        # 可能是 groups root 模式
        candidate = run_or_groups_dir

    if not os.path.isdir(candidate):
        return []

    ids = []
    for entry in sorted(os.listdir(candidate)):
        if entry.startswith(f"{prefix}_"):
            try:
                gid = int(entry.split("_")[1])
                ids.append(gid)
            except (ValueError, IndexError):
                continue
    return sorted(ids)
