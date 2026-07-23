#!/usr/bin/env python3
"""
DEPRECATED: 此兼容层的权威实现已迁移到 cr5_spray_perception.

请使用:
    from cr5_spray_perception import aruco_compat

本文件仅作为转发壳保留，后续版本将移除。
"""
import warnings
warnings.warn(
    "cr5_spray_sim.aruco_compat is deprecated, use cr5_spray_perception.aruco_compat",
    DeprecationWarning,
    stacklevel=2,
)

# 转发所有符号到权威实现
from cr5_spray_perception.aruco_compat import *  # noqa: F401 F403
