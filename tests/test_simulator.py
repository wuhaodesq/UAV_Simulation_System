import unittest

from uav_sim.controllers import WaypointController
from uav_sim.models import DRONE_PRESETS
from uav_sim.physics import Vec3
from uav_sim.simulator import UAVSimulator


class SimulatorTest(unittest.TestCase):
    def test_reaches_near_waypoint(self):
        model = DRONE_PRESETS["quad_light"]
        sim = UAVSimulator(model, WaypointController(max_accel_mps2=model.max_accel_mps2))
        samples = sim.run([Vec3(6, 0, 4)], duration=10.0, dt=0.05)

        end = samples[-1].state
        self.assertLess(abs(end.x - 6), 2.0)
        self.assertLess(abs(end.y - 0), 2.0)
        self.assertLess(abs(end.z - 4), 2.0)

    def test_invalid_args(self):
        model = DRONE_PRESETS["quad_light"]
        sim = UAVSimulator(model, WaypointController(max_accel_mps2=model.max_accel_mps2))

        with self.assertRaises(ValueError):
            sim.run([Vec3(1, 1, 1)], duration=0, dt=0.1)
        with self.assertRaises(ValueError):
            sim.run([Vec3(1, 1, 1)], duration=10, dt=0)


if __name__ == "__main__":
    unittest.main()
