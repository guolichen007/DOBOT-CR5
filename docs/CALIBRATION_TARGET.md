# 标定目标详解

## 坐标轴

- +X: 朝向 Front ChArUco 面板
- +Y: 朝向 Left 面板
- +Z: 向上

## 主体

0.34 (X) × 0.28 (Y) × 0.24 (Z) m

## 面板

| 面 | 图案 | Board | Square (mm) | Marker (mm) | Tag (mm) | ID |
|----|------|-------|-------------|-------------|----------|-----|
| Front | ChArUco | 8×6 | 27 | 20 | — | 100–123 |
| Left | ChArUco | 6×5 | 22 | 16 | — | 200–214 |
| Right | AprilTag | 2×2 | — | — | 70 | 4–7 |
| Top | AprilTag | 1×1 | — | — | 120 | 8 |
| Back | ChArUco | 8×6 | 27 | 20 | — | 300–323 |

## Panel 位置 (PANEL_GAP=0.001m)

| 面 | XYZ | RPY |
|----|-----|-----|
| Front | (0.171, 0, 0) | (0, +π/2, 0) |
| Left | (0, 0.141, 0) | (-π/2, 0, 0) |
| Right | (0, -0.141, 0) | (+π/2, 0, 0) |
| Top | (0, 0, 0.121) | (0, 0, 0) |
| Back | (-0.171, 0, 0) | (0, -π/2, 0) |

## 文件

| 文件 | SHA-256 |
|------|---------|
| charuco_front.png | 88bd0f19... |
| charuco_left.png | 4d658ccf... |
| charuco_back.png | 0301d4d9... |
| apriltag_right.png | 0cb8caf9... |
| apriltag_top.png | f7b68467... |

## 修改规则

- 禁止非等比缩放 PNG
- 禁止改变 Marker/Tag 真实尺寸
- 修改后必须重新运行 validate_calibration_target_geometry.py
