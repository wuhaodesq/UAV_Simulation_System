from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from typing import Dict, List

from .controllers import WaypointController
from .models import DRONE_PRESETS, DroneModel
from .physics import UAVState, Vec3
from .simulator import Sample, UAVSimulator


@dataclass(frozen=True)
class RnDRequirements:
    payload_kg: float
    endurance_min: float
    max_wind_mps: float


@dataclass(frozen=True)
class SimulationMission:
    name: str
    waypoints: List[Vec3]
    duration: float
    dt: float
    wind_mps: float


@dataclass
class MissionResult:
    mission_name: str
    pass_reach_goal: bool
    final_error_m: float
    max_speed_mps: float


@dataclass
class FullProcessReport:
    selected_model: str
    requirement_pass: bool
    requirement_notes: List[str]
    mission_results: List[MissionResult]
    overall_pass: bool

    def to_dict(self) -> Dict:
        return {
            "selected_model": self.selected_model,
            "requirement_pass": self.requirement_pass,
            "requirement_notes": self.requirement_notes,
            "mission_results": [asdict(m) for m in self.mission_results],
            "overall_pass": self.overall_pass,
        }


def _evaluate_requirements(model: DroneModel, req: RnDRequirements) -> List[str]:
    notes: List[str] = []
    if model.payload_capacity_kg < req.payload_kg:
        notes.append(f"payload insufficient: {model.payload_capacity_kg} < {req.payload_kg}")
    if model.endurance_min < req.endurance_min:
        notes.append(f"endurance insufficient: {model.endurance_min} < {req.endurance_min}")
    if model.max_speed_mps < req.max_wind_mps:
        notes.append(f"wind resistance weak: max_speed {model.max_speed_mps} < wind {req.max_wind_mps}")
    return notes


def _constant_wind_disturbance(wind_mps: float):
    gain = min(2.0, wind_mps * 0.12)

    def fn(_t: float, _state: UAVState) -> Vec3:
        return Vec3(gain, 0.0, 0.0)

    return fn


def _mission_result(mission: SimulationMission, samples: List[Sample]) -> MissionResult:
    end = samples[-1].state
    goal = mission.waypoints[-1]
    error = math.sqrt((end.x - goal.x) ** 2 + (end.y - goal.y) ** 2 + (end.z - goal.z) ** 2)
    max_speed = max(math.sqrt(s.state.vx ** 2 + s.state.vy ** 2 + s.state.vz ** 2) for s in samples)
    return MissionResult(
        mission_name=mission.name,
        pass_reach_goal=error < 3.0,
        final_error_m=round(error, 3),
        max_speed_mps=round(max_speed, 3),
    )


def run_full_process_simulation(model_key: str, req: RnDRequirements) -> FullProcessReport:
    model = DRONE_PRESETS[model_key]
    requirement_notes = _evaluate_requirements(model, req)
    requirement_pass = len(requirement_notes) == 0

    controller = WaypointController(max_accel_mps2=model.max_accel_mps2)
    sim = UAVSimulator(model, controller)

    missions = [
        SimulationMission(
            name="prototype_functional",
            waypoints=[Vec3(8, 0, 5), Vec3(12, 8, 8), Vec3(0, 0, 2)],
            duration=30,
            dt=0.1,
            wind_mps=min(req.max_wind_mps, 4.0),
        ),
        SimulationMission(
            name="verification_reliability",
            waypoints=[Vec3(15, 0, 10), Vec3(15, 15, 10), Vec3(0, 0, 2)],
            duration=40,
            dt=0.1,
            wind_mps=req.max_wind_mps,
        ),
    ]

    mission_results: List[MissionResult] = []
    for mission in missions:
        disturbance = _constant_wind_disturbance(mission.wind_mps)
        samples = sim.run(
            mission.waypoints,
            duration=mission.duration,
            dt=mission.dt,
            disturbance_fn=disturbance,
        )
        mission_results.append(_mission_result(mission, samples))

    overall_pass = requirement_pass and all(m.pass_reach_goal for m in mission_results)
    return FullProcessReport(
        selected_model=model.name,
        requirement_pass=requirement_pass,
        requirement_notes=requirement_notes,
        mission_results=mission_results,
        overall_pass=overall_pass,
    )


def save_report(report: FullProcessReport, output_path: str) -> None:
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report.to_dict(), f, ensure_ascii=False, indent=2)
