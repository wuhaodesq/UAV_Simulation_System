from dataclasses import dataclass


@dataclass(frozen=True)
class DroneModel:
    name: str
    mass_kg: float
    max_speed_mps: float
    max_accel_mps2: float
    drag_coeff: float


DRONE_PRESETS = {
    "quad_light": DroneModel(
        name="Light Quad",
        mass_kg=1.2,
        max_speed_mps=16.0,
        max_accel_mps2=8.0,
        drag_coeff=0.08,
    ),
    "camera_pro": DroneModel(
        name="Camera Pro",
        mass_kg=2.4,
        max_speed_mps=14.0,
        max_accel_mps2=6.0,
        drag_coeff=0.1,
    ),
    "heavy_lift": DroneModel(
        name="Heavy Lift",
        mass_kg=6.5,
        max_speed_mps=10.0,
        max_accel_mps2=4.0,
        drag_coeff=0.15,
    ),
}
