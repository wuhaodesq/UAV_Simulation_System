from dataclasses import dataclass
from .physics import UAVState, Vec3


@dataclass
class WaypointController:
    kp_pos: float = 1.2
    kd_vel: float = 0.7
    max_accel_mps2: float = 6.0

    def command(self, state: UAVState, waypoint: Vec3) -> Vec3:
        ax = self.kp_pos * (waypoint.x - state.x) - self.kd_vel * state.vx
        ay = self.kp_pos * (waypoint.y - state.y) - self.kd_vel * state.vy
        az = self.kp_pos * (waypoint.z - state.z) - self.kd_vel * state.vz

        ax = max(-self.max_accel_mps2, min(self.max_accel_mps2, ax))
        ay = max(-self.max_accel_mps2, min(self.max_accel_mps2, ay))
        az = max(-self.max_accel_mps2, min(self.max_accel_mps2, az))
        return Vec3(ax, ay, az)
