import json
import tempfile
import unittest

from uav_sim.process import RnDRequirements, load_config, run_full_process_simulation


class FullProcessTest(unittest.TestCase):
    def test_full_process_generates_results(self):
        report = run_full_process_simulation(
            model_key="camera_pro",
            req=RnDRequirements(payload_kg=1.0, endurance_min=30.0, max_wind_mps=6.0),
        )
        self.assertEqual(report.selected_model, "Camera Pro")
        self.assertGreaterEqual(len(report.mission_results), 2)
        self.assertGreaterEqual(report.safety_score, 0)
        self.assertLessEqual(report.safety_score, 100)

    def test_requirement_failure_detected(self):
        report = run_full_process_simulation(
            model_key="quad_light",
            req=RnDRequirements(payload_kg=2.0, endurance_min=60.0, max_wind_mps=20.0),
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


if __name__ == "__main__":
    unittest.main()
