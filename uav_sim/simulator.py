from dataclasses import dataclass
from typing import Iterable, List

from .controllers import WaypointController
from .models import DroneModel
from .physics import SimplePhysics, UAVState, Vec3


@dataclass
class Sample:
    t: float
    state: UAVState


class UAVSimulator:
    def __init__(self, model: DroneModel, controller: WaypointController):
        self.model = model
        self.controller = controller
        self.physics = SimplePhysics(model.drag_coeff, model.max_speed_mps)

    def run(self, waypoints: Iterable[Vec3], duration: float, dt: float) -> List[Sample]:
        if dt <= 0:
            raise ValueError("dt must be positive")
        if duration <= 0:
            raise ValueError("duration must be positive")

        wp = list(waypoints)
        if not wp:
            raise ValueError("At least one waypoint is required")

        idx = 0
        state = UAVState()
        t = 0.0
        samples: List[Sample] = []

        while t <= duration:
            target = wp[idx]
            accel = self.controller.command(state, target)
            state = self.physics.step(state, accel, dt)

            if (
                abs(target.x - state.x) < 0.8
                and abs(target.y - state.y) < 0.8
                and abs(target.z - state.z) < 0.8
                and idx < len(wp) - 1
            ):
                idx += 1

            samples.append(Sample(t=t, state=UAVState(**vars(state))))
            t += dt

        return samples
