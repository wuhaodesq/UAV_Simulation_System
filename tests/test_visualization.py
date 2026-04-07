import math
import unittest

from uav_sim.visualization import build_drone_shape, compute_heading


class VisualizationTest(unittest.TestCase):
    def test_heading_from_velocity(self):
        self.assertAlmostEqual(compute_heading(1.0, 0.0), 0.0)
        self.assertAlmostEqual(compute_heading(0.0, 1.0), math.pi / 2)

    def test_build_drone_shape(self):
        shape = build_drone_shape(10.0, 5.0, 0.0, body_size=2.0)
        self.assertEqual(shape.center, (10.0, 5.0))
        self.assertAlmostEqual(shape.arm_1[0][0], 11.0)
        self.assertAlmostEqual(shape.arm_1[1][0], 9.0)
        self.assertGreater(shape.rotor_radius, 0.0)


if __name__ == "__main__":
    unittest.main()
