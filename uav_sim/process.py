from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import random
from typing import Dict, List, Optional

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


@dataclass(frozen=True)
class FullProcessConfig:
    missions: List[SimulationMission]


@dataclass
class MissionResult:
    mission_name: str
    pass_reach_goal: bool
    final_error_m: float
    max_speed_mps: float
    wind_mps: float


@dataclass
class FullProcessReport:
    selected_model: str
    requirement_pass: bool
    requirement_notes: List[str]
    mission_results: List[MissionResult]
    robustness_pass_rate: float
    safety_score: float
    recommendations: List[str]
    overall_pass: bool

    def to_dict(self) -> Dict:
        return {
            "selected_model": self.selected_model,
            "requirement_pass": self.requirement_pass,
            "requirement_notes": self.requirement_notes,
            "mission_results": [asdict(m) for m in self.mission_results],
            "robustness_pass_rate": self.robustness_pass_rate,
            "safety_score": self.safety_score,
            "recommendations": self.recommendations,
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
        wind_mps=mission.wind_mps,
    )


def _default_config(req: RnDRequirements) -> FullProcessConfig:
    return FullProcessConfig(
        missions=[
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
    )


def _calc_safety_score(req_pass: bool, mission_results: List[MissionResult], robustness_pass_rate: float) -> float:
    base = 100.0
    if not req_pass:
        base -= 35
    for r in mission_results:
        if not r.pass_reach_goal:
            base -= 20
        base -= min(15.0, r.final_error_m * 2.0)
        if r.wind_mps > 8:
            base -= 5
    base -= (1.0 - robustness_pass_rate) * 25.0
    return round(max(0.0, min(100.0, base)), 2)


def _build_recommendations(report: FullProcessReport) -> List[str]:
    rec: List[str] = []
    if not report.requirement_pass:
        rec.append("建议提升动力系统或更换机型，以满足载荷/续航/抗风需求。")
    if any(not m.pass_reach_goal for m in report.mission_results):
        rec.append("建议重新整定控制器参数（kp/kd）并扩大仿真回归场景。")
    if report.robustness_pass_rate < 0.8:
        rec.append("鲁棒性通过率偏低，建议增加冗余控制与风场补偿策略。")
    if report.safety_score < 70:
        rec.append("建议在进入实飞前增加失效注入与蒙特卡洛扰动测试。")
    if not rec:
        rec.append("当前方案可进入下一研发阶段（硬件在环或半实物仿真）。")
    return rec


def _run_monte_carlo_robustness(
    sim: UAVSimulator,
    mission: SimulationMission,
    trials: int,
    seed: int = 7,
) -> float:
    rng = random.Random(seed)
    pass_cnt = 0
    for _ in range(max(1, trials)):
        # 在标称风速附近随机扰动
        wind = max(0.0, mission.wind_mps * (0.7 + 0.6 * rng.random()))
        samples = sim.run(
            mission.waypoints,
            duration=mission.duration,
            dt=mission.dt,
            disturbance_fn=_constant_wind_disturbance(wind),
        )
        result = _mission_result(mission, samples)
        if result.pass_reach_goal:
            pass_cnt += 1
    return round(pass_cnt / max(1, trials), 3)


def run_full_process_simulation(
    model_key: str,
    req: RnDRequirements,
    config: Optional[FullProcessConfig] = None,
    robustness_trials: int = 30,
) -> FullProcessReport:
    model = DRONE_PRESETS[model_key]
    requirement_notes = _evaluate_requirements(model, req)
    requirement_pass = len(requirement_notes) == 0

    controller = WaypointController(max_accel_mps2=model.max_accel_mps2)
    sim = UAVSimulator(model, controller)
    cfg = config or _default_config(req)

    mission_results: List[MissionResult] = []
    for mission in cfg.missions:
        disturbance = _constant_wind_disturbance(mission.wind_mps)
        samples = sim.run(
            mission.waypoints,
            duration=mission.duration,
            dt=mission.dt,
            disturbance_fn=disturbance,
        )
        mission_results.append(_mission_result(mission, samples))

    robustness_target = cfg.missions[-1]
    robustness_pass_rate = _run_monte_carlo_robustness(sim, robustness_target, robustness_trials)
    safety_score = _calc_safety_score(requirement_pass, mission_results, robustness_pass_rate)
    overall_pass = (
        requirement_pass
        and all(m.pass_reach_goal for m in mission_results)
        and robustness_pass_rate >= 0.8
        and safety_score >= 70
    )
    report = FullProcessReport(
        selected_model=model.name,
        requirement_pass=requirement_pass,
        requirement_notes=requirement_notes,
        mission_results=mission_results,
        robustness_pass_rate=robustness_pass_rate,
        safety_score=safety_score,
        recommendations=[],
        overall_pass=overall_pass,
    )
    report.recommendations = _build_recommendations(report)
    return report


def save_report(report: FullProcessReport, output_path: str) -> None:
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report.to_dict(), f, ensure_ascii=False, indent=2)


def save_markdown_summary(report: FullProcessReport, output_path: str) -> None:
    lines = [
        "# UAV 全流程仿真摘要",
        "",
        f"- 机型: {report.selected_model}",
        f"- 需求通过: {report.requirement_pass}",
        f"- 鲁棒性通过率: {report.robustness_pass_rate}",
        f"- 安全评分: {report.safety_score}",
        f"- 总体结论: {report.overall_pass}",
        "",
        "## 任务结果",
    ]
    for m in report.mission_results:
        lines.append(
            f"- {m.mission_name}: pass={m.pass_reach_goal}, error={m.final_error_m}m, max_speed={m.max_speed_mps}m/s, wind={m.wind_mps}m/s"
        )
    lines.append("")
    lines.append("## 建议")
    for r in report.recommendations:
        lines.append(f"- {r}")

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def load_config(config_path: str) -> FullProcessConfig:
    with open(config_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    missions: List[SimulationMission] = []
    for item in data.get("missions", []):
        waypoints = [Vec3(*p) for p in item["waypoints"]]
        missions.append(
            SimulationMission(
                name=item["name"],
                waypoints=waypoints,
                duration=float(item.get("duration", 30)),
                dt=float(item.get("dt", 0.1)),
                wind_mps=float(item.get("wind_mps", 0.0)),
            )
        )

    if not missions:
        raise ValueError("config missions cannot be empty")
    return FullProcessConfig(missions=missions)
