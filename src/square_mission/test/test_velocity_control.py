import math
import unittest

from square_mission.velocity_control import (
    ScalarSpeedPid,
    limit_xy_vector,
    target_velocity_world_cm_s,
    touchdown_ready,
)


class VelocityControlTest(unittest.TestCase):
    def controller(self, kd=0.2, kp=0.8):
        return ScalarSpeedPid(
            kp=kp,
            ki=0.0,
            kd=kd,
            max_speed_cm_s=24.0,
            deadzone_cm=0.6,
        )

    def test_far_target_is_capped_and_near_target_stops(self):
        controller = self.controller()
        self.assertEqual(controller.update(120.0, 0.02), 24.0)
        self.assertEqual(controller.update(0.5, 0.02), 0.0)

    def test_diagonal_velocity_respects_combined_limit(self):
        controller = self.controller()
        vx, vy, distance = target_velocity_world_cm_s(
            0.0, 0.0, 1.0, 1.0, controller, 0.02
        )
        self.assertAlmostEqual(distance, math.hypot(100.0, 100.0))
        self.assertLessEqual(math.hypot(vx, vy), 24.0 + 1e-9)
        self.assertAlmostEqual(abs(vx), abs(vy))

    def test_derivative_damps_approach_without_reversing(self):
        controller = self.controller(kd=0.2)
        controller.update(100.0, 0.02)
        controller.update(80.0, 0.02)
        slowing = controller.update(60.0, 0.02)
        self.assertGreaterEqual(slowing, 0.0)
        self.assertLess(slowing, 24.0)

    def test_vector_limit_scales_not_axis_clamps(self):
        x, y = limit_xy_vector(24.0, 24.0, 24.0)
        self.assertAlmostEqual(math.hypot(x, y), 24.0)
        self.assertAlmostEqual(x, y)

    def test_reset_clears_integrator_and_derivative_history(self):
        controller = self.controller()
        controller.update(100.0, 0.02)
        controller.reset()
        self.assertTrue(controller.first_call)
        self.assertEqual(controller.integral_cm_s, 0.0)

    def test_touchdown_requires_low_height_and_vertical_rate(self):
        self.assertFalse(touchdown_ready(5.1, 1.0, 5.0, 6.0))
        self.assertFalse(touchdown_ready(1.0, 6.1, 5.0, 6.0))
        self.assertTrue(touchdown_ready(5.0, 6.0, 5.0, 6.0))
        self.assertTrue(touchdown_ready(1.0, -1.0, 5.0, 6.0))
        self.assertTrue(touchdown_ready(0.0, 2.9, 0.0, 3.0))
        self.assertFalse(touchdown_ready(0.1, 2.9, 0.0, 3.0))
        self.assertFalse(touchdown_ready(math.nan, 0.0, 5.0, 6.0))
        self.assertFalse(touchdown_ready(1.0, math.nan, 5.0, 6.0))


if __name__ == "__main__":
    unittest.main()
