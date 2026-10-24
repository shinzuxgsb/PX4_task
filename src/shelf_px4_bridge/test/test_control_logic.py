import math
import unittest

from shelf_px4_bridge.control_logic import (
    parse_bridge_status,
    sample_rate_hz,
    stability_stats,
    yaw_from_quaternion,
)


class ControlLogicTests(unittest.TestCase):
    def test_parse_bridge_status(self):
        status = parse_bridge_status(
            "链路=OK PX4模式=OFFBOARD armed=False ekf_ok=True 电池=15.7V/67%"
        )
        self.assertIsNotNone(status)
        self.assertEqual(status.mode, "OFFBOARD")
        self.assertFalse(status.armed)
        self.assertTrue(status.ekf_ok)
        self.assertTrue(status.link_ok)
        self.assertEqual(status.battery_pct, 67)

    def test_parse_bridge_status_rejects_incomplete_line(self):
        self.assertIsNone(parse_bridge_status("链路=OK"))

    def test_stability_stats_uses_first_sample_as_reference(self):
        samples = [
            (0.0, 1.00, 2.00, 0.10),
            (0.5, 1.03, 2.00, 0.12),
            (1.0, 1.04, 1.97, 0.08),
        ]
        drift_cm, yaw_span_deg, count = stability_stats(samples)
        self.assertEqual(count, 3)
        self.assertAlmostEqual(drift_cm, 5.0)
        self.assertLess(yaw_span_deg, 2.0)

    def test_yaw_from_quaternion_90_degrees(self):
        yaw = yaw_from_quaternion(
            0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)
        )
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_sample_rate(self):
        self.assertEqual(sample_rate_hz([0.0, 0.1, 0.2, 0.3]), 10.0)
        self.assertEqual(sample_rate_hz([1.0]), 0.0)


if __name__ == "__main__":
    unittest.main()
