# V3.3.2 初始姿态故障记录

> 分支：`fix/cr5-spray-tool-v33-stable-painting`
> HEAD：`0570504`
> 状态：launcher fixed, initial-pose NOT fixed, GUI acceptance FAILED

## 实测数据

```
world → Link6:              [-0.530, -0.141, -0.293]
world → spray_nozzle_frame: [-0.422, -0.141, -0.230]
world → object_frame:       [ 0.560,  0.000,  0.980]

cr5_robot model pose: (0, 0, 0)
joint_state_controller: running
arm_controller: running
```

## 预期值（六轴全零）

```
world → Link6:              [ 0.000, -0.246,  1.047]
world → spray_nozzle_frame: [ 0.000, -0.371,  1.047]
```

## 根因

1. Gazebo 未暂停，物理启动后 CR5 关节受重力下落
2. controller_manager 尚未就绪，无控制器保持姿态
3. arm_controller 启动时接管了已折叠的姿态
4. 未调用 set_model_configuration 设置初始关节角
