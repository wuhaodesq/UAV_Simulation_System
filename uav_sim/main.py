import argparse
import csv

from .controllers import WaypointController
from .models import DRONE_PRESETS
from .physics import Vec3
from .process import (
    RnDRequirements,
    load_config,
    run_full_process_simulation,
    save_markdown_summary,
    save_report,
)
from .simulator import UAVSimulator
from .visualization import render_trajectory_with_drone


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="UAV R&D full-process simulation platform")
    p.add_argument("--model", default="quad_light", choices=sorted(DRONE_PRESETS.keys()))
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--dt", type=float, default=0.1)
    p.add_argument("--output", default="trajectory.csv")
    p.add_argument("--plot", default="", help="输出仿真图像路径，例如 trajectory.png")
    p.add_argument("--payload", type=float, default=0.5, help="研发需求载荷(kg)")
    p.add_argument("--endurance", type=float, default=30.0, help="研发需求续航(min)")
    p.add_argument("--wind", type=float, default=6.0, help="研发需求最大风速(m/s)")
    p.add_argument("--report", default="full_process_report.json", help="全流程评估报告输出路径")
    p.add_argument("--summary", default="full_process_summary.md", help="全流程Markdown摘要输出路径")
    p.add_argument("--config", default="", help="全流程任务配置 JSON 文件路径")
    p.add_argument("--trials", type=int, default=30, help="鲁棒性蒙特卡洛仿真次数")
    return p.parse_args()


def run_single_flight(model_key: str, duration: float, dt: float, output_csv: str, plot: str) -> None:
    model = DRONE_PRESETS[model_key]
    controller = WaypointController(max_accel_mps2=model.max_accel_mps2)
    sim = UAVSimulator(model, controller)

    mission = [
        Vec3(10, 0, 8),
        Vec3(20, 20, 12),
        Vec3(5, 25, 10),
        Vec3(0, 0, 2),
    ]
    samples = sim.run(mission, duration=duration, dt=dt)

    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z", "vx", "vy", "vz"])
        for s in samples:
            writer.writerow([
                f"{s.t:.2f}",
                f"{s.state.x:.3f}",
                f"{s.state.y:.3f}",
                f"{s.state.z:.3f}",
                f"{s.state.vx:.3f}",
                f"{s.state.vy:.3f}",
                f"{s.state.vz:.3f}",
            ])

    if plot:
        try:
            image = render_trajectory_with_drone(samples, plot)
            print(f"trajectory image saved: {image}")
        except RuntimeError as exc:
            print(f"skip plot: {exc}")

    final = samples[-1].state
    print(
        f"single flight model={model.name}, final_pos=({final.x:.2f},{final.y:.2f},{final.z:.2f}), "
        f"samples={len(samples)}"
    )


def main() -> None:
    args = parse_args()

    run_single_flight(args.model, args.duration, args.dt, args.output, args.plot)

    req = RnDRequirements(payload_kg=args.payload, endurance_min=args.endurance, max_wind_mps=args.wind)
    config = load_config(args.config) if args.config else None
    report = run_full_process_simulation(args.model, req, config=config, robustness_trials=args.trials)
    save_report(report, args.report)
    save_markdown_summary(report, args.summary)

    print(
        f"full process report saved: {args.report}, summary saved: {args.summary}, "
        f"requirement_pass={report.requirement_pass}, robustness_pass_rate={report.robustness_pass_rate}, "
        f"safety_score={report.safety_score}, overall_pass={report.overall_pass}"
    )
    if report.requirement_notes:
        print("requirement notes:")
        for note in report.requirement_notes:
            print(f"- {note}")

    print("recommendations:")
    for rec in report.recommendations:
        print(f"- {rec}")


if __name__ == "__main__":
    main()
