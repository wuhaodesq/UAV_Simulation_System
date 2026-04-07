from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Tuple

from .simulator import Sample


@dataclass
class DroneShape:
    center: Tuple[float, float]
    arm_1: Tuple[Tuple[float, float], Tuple[float, float]]
    arm_2: Tuple[Tuple[float, float], Tuple[float, float]]
    rotor_radius: float


def compute_heading(vx: float, vy: float) -> float:
    if abs(vx) < 1e-6 and abs(vy) < 1e-6:
        return 0.0
    return math.atan2(vy, vx)


def build_drone_shape(x: float, y: float, heading: float, body_size: float = 1.2) -> DroneShape:
    half = body_size / 2.0
    c = math.cos(heading)
    s = math.sin(heading)

    # X 型四旋翼外形
    p1 = (x + half * c, y + half * s)
    p2 = (x - half * c, y - half * s)

    c2 = math.cos(heading + math.pi / 2)
    s2 = math.sin(heading + math.pi / 2)
    p3 = (x + half * c2, y + half * s2)
    p4 = (x - half * c2, y - half * s2)

    return DroneShape(center=(x, y), arm_1=(p1, p2), arm_2=(p3, p4), rotor_radius=body_size * 0.15)


def render_trajectory_with_drone(samples: Iterable[Sample], output_image: str = "trajectory.png") -> str:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
    except ImportError as exc:
        raise RuntimeError("matplotlib is required for visualization") from exc

    data: List[Sample] = list(samples)
    if not data:
        raise ValueError("samples cannot be empty")

    xs = [s.state.x for s in data]
    ys = [s.state.y for s in data]

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(xs, ys, color="#2979ff", linewidth=2, label="Flight Trajectory")
    ax.scatter(xs[0], ys[0], color="green", s=60, label="Start")
    ax.scatter(xs[-1], ys[-1], color="red", s=60, label="End")

    end = data[-1].state
    heading = compute_heading(end.vx, end.vy)
    shape = build_drone_shape(end.x, end.y, heading)

    ax.plot(
        [shape.arm_1[0][0], shape.arm_1[1][0]],
        [shape.arm_1[0][1], shape.arm_1[1][1]],
        color="black",
        linewidth=2,
    )
    ax.plot(
        [shape.arm_2[0][0], shape.arm_2[1][0]],
        [shape.arm_2[0][1], shape.arm_2[1][1]],
        color="black",
        linewidth=2,
    )

    for rotor in [shape.arm_1[0], shape.arm_1[1], shape.arm_2[0], shape.arm_2[1]]:
        ax.add_patch(Circle(rotor, shape.rotor_radius, fill=False, color="gray", linewidth=1.5))

    ax.set_title("UAV Trajectory Simulation (Top View)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_image, dpi=150)
    plt.close(fig)
    return output_image
