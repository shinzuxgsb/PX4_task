import math
import unittest

from square_mission.geometry import (
    body_offset_to_world,
    clockwise_square_offsets,
    signed_polygon_area_cm2,
    square_waypoints,
    wrap_angle,
)


class SquareGeometryTest(unittest.TestCase):
    def test_default_square_is_clockwise_from_lower_left(self):
        points = list(square_waypoints(0.0, 0.0, 0.0, 120.0))
        points.append(points[0])
        self.assertEqual(len(points), 5)
        self.assertLess(signed_polygon_area_cm2(points), 0.0)

        lengths = [
            math.dist(points[index], points[index + 1]) * 100.0
            for index in range(4)
        ]
        for length in lengths:
            self.assertAlmostEqual(length, 120.0)

        self.assertEqual(clockwise_square_offsets(120.0)[0], (120.0, 0.0))
        self.assertEqual(clockwise_square_offsets(120.0)[1], (120.0, -120.0))
        self.assertEqual(clockwise_square_offsets(120.0)[-1], (0.0, 0.0))

    def test_rotation_keeps_body_relative_square(self):
        waypoints = square_waypoints(1.0, 2.0, math.pi / 2.0, 120.0)
        self.assertAlmostEqual(waypoints[0][0], 1.0)
        self.assertAlmostEqual(waypoints[0][1], 3.2)
        self.assertAlmostEqual(waypoints[1][0], 2.2)
        self.assertAlmostEqual(waypoints[1][1], 3.2)
        self.assertAlmostEqual(waypoints[2][0], 2.2)
        self.assertAlmostEqual(waypoints[2][1], 2.0)
        self.assertAlmostEqual(waypoints[3][0], 1.0)
        self.assertAlmostEqual(waypoints[3][1], 2.0)

    def test_world_conversion(self):
        x, y = body_offset_to_world(1.0, 2.0, 0.0, 120.0, 60.0)
        self.assertAlmostEqual(x, 2.2)
        self.assertAlmostEqual(y, 2.6)

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(math.radians(350.0)), math.radians(-10.0))

    def test_invalid_side_rejected(self):
        with self.assertRaises(ValueError):
            clockwise_square_offsets(0.0)


if __name__ == "__main__":
    unittest.main()
