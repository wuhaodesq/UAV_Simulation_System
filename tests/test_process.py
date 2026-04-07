import csv
import json
import os
import tempfile
import unittest

from uav_sim.process import (
    RnDRequirements,
    load_config,
    run_full_process_simulation,
    save_markdown_summary,
    save_metrics_csv,
)


class FullProcessTest(unittest.TestCase):
    def test_full_process_generates_results(self):
        report = run_full_process_simulation(
            model_key="camera_pro",
            req=RnDRequirements(payload_kg=1.0, endurance_min=30.0, max_wind_mps=6.0),
            robustness_trials=10,
            random_seed=7,
        )
        self.assertEqual(report.selected_model, "Camera Pro")
        self.assertGreaterEqual(len(report.mission_results), 2)
        self.assertGreaterEqual(report.safety_score, 0)
        self.assertLessEqual(report.safety_score, 100)
        self.assertGreaterEqual(report.robustness_pass_rate, 0)
        self.assertLessEqual(report.robustness_pass_rate, 1)

    def test_seed_reproducible(self):
        req = RnDRequirements(payload_kg=1.0, endurance_min=30.0, max_wind_mps=6.0)
        r1 = run_full_process_simulation("camera_pro", req, robustness_trials=12, random_seed=42)
        r2 = run_full_process_simulation("camera_pro", req, robustness_trials=12, random_seed=42)
        self.assertEqual(r1.robustness_pass_rate, r2.robustness_pass_rate)

    def test_requirement_failure_detected(self):
        report = run_full_process_simulation(
            model_key="quad_light",
            req=RnDRequirements(payload_kg=2.0, endurance_min=60.0, max_wind_mps=20.0),
            robustness_trials=8,
            random_seed=7,
        )
        self.assertFalse(report.requirement_pass)
        self.assertGreaterEqual(len(report.requirement_notes), 1)
        self.assertGreaterEqual(len(report.recommendations), 1)

    def test_load_custom_config(self):
        config_data = {
            "missions": [
                {
                    "name": "custom_validation",
                    "waypoints": [[6, 0, 4], [0, 0, 2]],
                    "duration": 20,
                    "dt": 0.1,
                    "wind_mps": 5,
                }
            ]
        }
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump(config_data, f)
            path = f.name

        cfg = load_config(path)
        self.assertEqual(len(cfg.missions), 1)
        self.assertEqual(cfg.missions[0].name, "custom_validation")
        os.remove(path)

    def test_save_markdown_summary(self):
        report = run_full_process_simulation(
            model_key="camera_pro",
            req=RnDRequirements(payload_kg=1.0, endurance_min=30.0, max_wind_mps=6.0),
            robustness_trials=5,
            random_seed=1,
        )
        with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False) as f:
            path = f.name

        save_markdown_summary(report, path)
        with open(path, "r", encoding="utf-8") as f:
            content = f.read()
        self.assertIn("UAV 全流程仿真摘要", content)
        self.assertIn("安全评分", content)
        os.remove(path)

    def test_save_metrics_csv(self):
        report = run_full_process_simulation(
            model_key="camera_pro",
            req=RnDRequirements(payload_kg=1.0, endurance_min=30.0, max_wind_mps=6.0),
            robustness_trials=4,
            random_seed=3,
        )
        with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
            path = f.name

        save_metrics_csv(report, path)
        with open(path, "r", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        self.assertGreaterEqual(len(rows), 2)
        self.assertEqual(rows[0][0], "mission_name")
        os.remove(path)


if __name__ == "__main__":
    unittest.main()
