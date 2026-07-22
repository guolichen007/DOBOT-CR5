# 仿真操作手册

## 构建

```bash
cd ~/cr5_ros1_ws
catkin_make
source devel/setup.bash
```

## 启动

```bash
# GUI 模式 (调试)
bash src/cr5_spray_sim/scripts/run_simulation.sh \
  --gui --object=calibration_target --profile=quality

# Headless 模式 (批量)
bash src/cr5_spray_sim/scripts/run_simulation.sh \
  --headless --object=calibration_target --profile=quality --strict
```

## 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| --gui | flag | 显示 Gazebo GUI |
| --headless | flag | 无头模式 |
| --object | calibration_target, motor_housing_cylinder, rectangular_housing | 工作对象 |
| --profile | vm, quality | 相机分辨率 |
| --strict | flag | 任何检查失败则退出 |
| --isolated | flag | 独立模式 |

## 第二终端

```bash
source /tmp/cr5_spray_simulation.env
rostopic list
rosrun tf tf_echo world calibration_target_frame
```

## 停止

在启动终端按 Ctrl+C，自动清理进程和会话文件。
