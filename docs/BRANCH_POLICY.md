# 分支策略

## 永久分支

| 分支 | 职责 |
|------|------|
| `main` | 已验证、可构建、可运行的稳定工程基线 |
| `develop/multi-camera-calibration` | 下一阶段三相机外参与三维重建开发 |

## 短期任务分支

命名规范:
- `feature/<clear-topic>` — 新功能
- `fix/<clear-topic>` — 修复
- `chore/<clear-topic>` — 工程整理

完成后必须合并并删除远端分支。

## 禁止

- 长期存在的 fix/...-v1、feature/...-v337 风格分支
- 直接在 main 上做大改动 (小修复除外)
- git push --force origin main (除非人工授权)
