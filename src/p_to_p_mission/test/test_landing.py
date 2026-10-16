import unittest

from p_to_p_mission.landing import next_descent_setpoint_cm


class LandingRampTest(unittest.TestCase):
    def test_target_descends_at_bounded_rate(self):
        self.assertAlmostEqual(next_descent_setpoint_cm(60, 60, 10, 0.1, 10, 5), 59)

    def test_target_does_not_lead_stationary_aircraft_beyond_limit(self):
        self.assertAlmostEqual(next_descent_setpoint_cm(55, 60, 10, 0.1, 10, 5), 55)

    def test_target_never_goes_below_measured_ground_baseline(self):
        self.assertAlmostEqual(next_descent_setpoint_cm(10.4, 11, 10, 0.1, 10, 5), 10)

    def test_delayed_timer_does_not_make_large_step(self):
        self.assertAlmostEqual(next_descent_setpoint_cm(60, 60, 10, 5, 10, 5), 57.5)

    def test_range_jump_up_does_not_command_climb(self):
        self.assertAlmostEqual(next_descent_setpoint_cm(55, 70, 10, 0.1, 10, 5), 55)


if __name__ == "__main__":
    unittest.main()
