# UAV Simulation System

一个面向**无人机研发全流程**的模拟仿真软件（Python 版），用于在研发早期到验证阶段提前发现风险，降低试飞成本。

## 全流程能力

- **需求阶段仿真校核**：根据载荷、续航、抗风需求检查机型可行性
- **方案阶段飞行模拟**：多航点飞行、控制器闭环、简化 3D 动力学
- **验证阶段可靠性场景**：可注入风扰动，评估任务到达误差
- **可视化与资产沉淀**：输出轨迹 CSV、无人机外形叠加图、全流程 JSON 报告

## 快速开始

```bash
python3 -m uav_sim.main \
  --model camera_pro \
  --duration 35 --dt 0.1 \
  --payload 1.0 --endurance 30 --wind 7 \
  --output trajectory.csv \
  --plot trajectory.png \
  --report full_process_report.json
```

运行后将生成：

- `trajectory.csv`: 单次飞行轨迹
- `trajectory.png`: 轨迹与无人机外形图（可选，需 matplotlib）
- `full_process_report.json`: 研发全流程评估结果（需求通过、任务通过、总体结论）

## 参数说明

- `--model`: 机型名称，可选 `quad_light` / `camera_pro` / `heavy_lift`
- `--duration`: 单次飞行仿真总时长（秒）
- `--dt`: 单次飞行仿真步长（秒）
- `--payload`: 研发需求载荷（kg）
- `--endurance`: 研发需求续航（min）
- `--wind`: 研发需求最大风速（m/s）
- `--output`: 单次飞行轨迹 CSV 路径
- `--plot`: 单次飞行图像路径（可选，需要 matplotlib）
- `--report`: 全流程报告 JSON 路径

## 运行测试

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
```
