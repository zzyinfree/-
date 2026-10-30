# 无人机空地协同搜索打击预赛代码

本仓库保存当前预赛仿真的 ROS 源码、任务运行脚本，以及九格多模态模型相关的训练与评估脚本。飞行录像、日志、训练数据和模型权重未纳入 Git 仓库。

## 目录

- `ros_src/mission_pkg`：行为树任务、穿框、搜索打击、返航与降落控制。
- `ros_src/object_det`：视觉检测节点与配置。
- `ros_src/sensor_pkg`、`ros_src/common_msgs`：传感器和消息定义。
- `workspace_scripts`：仿真启动、重置、在线自然语言任务、摄像头、训练和评估脚本。

## 在比赛实例中运行

代码以比赛实例的 ROS Noetic、PX4、RflySim3D、FM9G4B-V 及原有环境为基础。**另一台电脑须先启动同一官方 Ubuntu 比赛镜像或其完整克隆**。普通电脑只克隆此仓库无法运行仿真：RflySim3D/PX4、ROS 扩展包、`object_det` 的 Ultralytics、检测权重及约 8.5 GB 的九格模型都来自比赛镜像，不在本仓库。不要把模型、凭据或录像直接推到这个公开仓库。

在新比赛实例的 root 终端执行（仓库名为 `-`，可以直接复制）：

```bash
git clone https://github.com/zzyinfree/-.git /root/cicc-source
bash /root/cicc-source/restore_on_instance.sh --check
bash /root/cicc-source/restore_on_instance.sh --install
```

`--check` 先报告缺少的镜像资源；`--install` 把 ROS 包和任务脚本放到比赛环境的固定目录并执行 `catkin_make`。请在启动仿真前确认脚本通过。如果新实例的官方镜像缺少模型或第三方依赖，应从合法的比赛环境来源恢复，不能通过这个源码仓库自动获取。

启动仿真：

```bash
/usr/bin/sudo /bin/bash /root/gpufree-data/cicc_workspace/competition_boot.sh
```

执行一条自然语言任务：

```bash
/usr/bin/sudo /bin/bash /root/gpufree-data/cicc_workspace/run_mission_online.sh --once --timeout 300
```

比赛时应输入组委会指定的提示词。此仓库中的试飞脚本只用于本地仿真验证。

## 最近验证

- 蓝球：`run_175_blue_fast_scan_20261030`，前视两帧确认、冲刺、下视 20 帧未见蓝球，返航降落。
- 红球：`run_177_red_right_return_20261030`，冲刺后使用右侧返航航点，完成降落。
- 紫球：`run_173_purple_recovery_center_20261030`，边界回拉与重搜索后返航，最终仿真落点距起飞中心约 7 cm。

上述 `SUCCESS` 表示任务流程结束；没有接入裁判计分信号，不能据此认定气球得分。
