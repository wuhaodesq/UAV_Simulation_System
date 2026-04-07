from dataclasses import dataclass
import math


@dataclass
class UAVState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


@dataclass
class Vec3:
    x: float
    y: float
    z: float


class SimplePhysics:
    def __init__(self, drag_coeff: float, max_speed_mps: float):
        self.drag_coeff = drag_coeff
        self.max_speed_mps = max_speed_mps

    def step(self, state: UAVState, accel_cmd: Vec3, dt: float) -> UAVState:
        ax = accel_cmd.x - self.drag_coeff * state.vx
        ay = accel_cmd.y - self.drag_coeff * state.vy
        az = accel_cmd.z - self.drag_coeff * state.vz

        state.vx += ax * dt
        state.vy += ay * dt
        state.vz += az * dt

        speed = math.sqrt(state.vx ** 2 + state.vy ** 2 + state.vz ** 2)
        if speed > self.max_speed_mps and speed > 0:
            scale = self.max_speed_mps / speed
            state.vx *= scale
            state.vy *= scale
            state.vz *= scale

        state.x += state.vx * dt
        state.y += state.vy * dt
        state.z = max(0.0, state.z + state.vz * dt)
        if state.z <= 0.0 and state.vz < 0:
            state.vz = 0.0

        return state
