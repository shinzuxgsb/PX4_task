import unittest

from gs import config
from gs.field import Field
from gs.planner import plan_sweep
from gs.state import GroundState


class GroundstationTests(unittest.TestCase):
    def setUp(self):
        self.runtime = config.load_runtime()
        self.field = Field(config.load_field())

    def test_sweep_route_keeps_reference_points(self):
        route = plan_sweep(self.field)
        scans = [leg for leg in route["legs"] if leg["type"] == "scan"]
        self.assertEqual(len(scans), 12)
        self.assertEqual(len(route["waypoints"]), 12)
        self.assertIn("A1", self.field.slots)
        self.assertIn("B6", self.field.slots)

    def test_px4_status_is_exposed_in_snapshot(self):
        state = GroundState(self.field, self.runtime)
        state.handle({
            "type": "px4_status",
            "state": "WAIT_GO",
            "mode": "OFFBOARD",
            "armed": False,
            "ekf_ok": True,
            "link_ok": True,
            "ready": 0,
            "battery_pct": 74,
            "takeoff_available": True,
        })
        snapshot = state.snapshot()
        self.assertEqual(snapshot["px4"]["state"], "WAIT_GO")
        self.assertTrue(snapshot["px4"]["takeoff_available"])
        self.assertEqual(snapshot["px4"]["battery_pct"], 74)

    def test_start_sends_px4_specific_command(self):
        state = GroundState(self.field, self.runtime)
        sent = []
        state.send = sent.append
        route = state.start_sweep()
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0]["type"], "cmd")
        self.assertEqual(sent[0]["cmd"], "px4_start")
        self.assertEqual(sent[0]["mode"], "sweep")
        self.assertEqual(sent[0]["route"]["waypoints"], route["waypoints"])


if __name__ == "__main__":
    unittest.main()
