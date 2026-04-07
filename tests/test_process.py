import unittest

from uav_sim.process import RnDRequirements, run_full_process_simulation


class FullProcessTest(unittest.TestCase):
    def test_full_process_generates_results(self):
        report = run_full_process_simulation(
            model_key="camera_pro",
            req=RnDRequirements(payload_kg=1.0, endurance_min=30.0, max_wind_mps=6.0),
        )
        self.assertEqual(report.selected_model, "Camera Pro")
        self.assertGreaterEqual(len(report.mission_results), 2)

    def test_requirement_failure_detected(self):
        report = run_full_process_simulation(
            model_key="quad_light",
            req=RnDRequirements(payload_kg=2.0, endurance_min=60.0, max_wind_mps=20.0),
        )
        self.assertFalse(report.requirement_pass)
        self.assertGreaterEqual(len(report.requirement_notes), 1)


if __name__ == "__main__":
    unittest.main()
