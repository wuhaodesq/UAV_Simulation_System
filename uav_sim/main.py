import argparse
import csv

from .controllers import WaypointController
from .models import DRONE_PRESETS
from .physics import Vec3
from .simulator import UAVSimulator


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="UAV flight simulation")
    p.add_argument("--model", default="quad_light", choices=sorted(DRONE_PRESETS.keys()))
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--dt", type=float, default=0.1)
    p.add_argument("--output", default="trajectory.csv")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    model = DRONE_PRESETS[args.model]

    controller = WaypointController(max_accel_mps2=model.max_accel_mps2)
    sim = UAVSimulator(model, controller)

    mission = [
        Vec3(10, 0, 8),
        Vec3(20, 20, 12),
        Vec3(5, 25, 10),
        Vec3(0, 0, 2),
    ]
    samples = sim.run(mission, duration=args.duration, dt=args.dt)

    with open(args.output, "w", newline="", encoding="utf-8") as f:
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

    final = samples[-1].state
    print(
        f"model={model.name}, final_pos=({final.x:.2f},{final.y:.2f},{final.z:.2f}), "
        f"final_vel=({final.vx:.2f},{final.vy:.2f},{final.vz:.2f}), samples={len(samples)}"
    )


if __name__ == "__main__":
    main()
