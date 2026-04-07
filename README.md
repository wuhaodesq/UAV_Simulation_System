# UAV Simulation System

一个可扩展的无人机仿真飞行软件（Python 版），支持多种机型参数、控制器与任务轨迹。

## 功能

- 支持多种无人机机型（轻型四旋翼、摄影无人机、重载无人机）
- 简化 3D 动力学（位置、速度、加速度）
- 基于 PID 思想的航点跟随控制器
- 可配置仿真步长、总时长、目标航点
- 输出轨迹 CSV，便于二次分析与可视化
- 提供无人机外形（X 型四旋翼）俯视图叠加到仿真轨迹

## 快速开始

```bash
python3 -m uav_sim.main --model quad_light --duration 40 --dt 0.05 --output trajectory.csv --plot trajectory.png
```

运行后会在终端打印最终状态，并在 `trajectory.csv` 中保存每一时刻状态。

## 参数说明

- `--model`: 机型名称，可选 `quad_light` / `camera_pro` / `heavy_lift`
- `--duration`: 仿真总时长（秒）
- `--dt`: 时间步长（秒）
- `--output`: 轨迹输出文件路径
- `--plot`: 轨迹图像输出路径（可选，需要 matplotlib）

## 运行测试

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
```
