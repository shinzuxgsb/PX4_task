import math
import unittest

from p_to_p_mission.geometry import (
    body_offset_to_map,
    normalize_angle_deg,
    target_error,
)


class GeometryTest(unittest.TestCase):
    def test_normalize_angle_deg(self):
        self.assertEqual(normalize_angle_deg(190.0), -170.0)
        self.assertEqual(normalize_angle_deg(-190.0), 170.0)

    def test_body_offset_zero_yaw(self):
        x_cm, y_cm = body_offset_to_map(10.0, 20.0, 0.0, 250.0, -100.0)
        self.assertEqual(x_cm, 260.0)
        self.assertEqual(y_cm, -80.0)

    def test_body_offset_ninety_degree_yaw(self):
        x_cm, y_cm = body_offset_to_map(0.0, 0.0, 90.0, 100.0, 0.0)
        self.assertTrue(math.isclose(x_cm, 0.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(y_cm, 100.0, abs_tol=1e-9))

    def test_target_error_wraps_yaw(self):
        xy, z, yaw = target_error(0, 0, 100, 179, 3, 4, 110, -179)
        self.assertEqual(xy, 5.0)
        self.assertEqual(z, 10.0)
        self.assertEqual(yaw, 2.0)


if __name__ == "__main__":
    unittest.main()
