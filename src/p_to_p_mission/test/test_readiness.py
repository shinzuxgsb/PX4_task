"""Exercise production callbacks without ROS, ports, or flight publishers."""

import ast
from collections import deque
from pathlib import Path
from types import SimpleNamespace as S
import time
import unittest

from p_to_p_mission.preflight import ground_height_baseline_cm


def callback(name):
    path = Path(__file__).parents[1] / 'p_to_p_mission' / 'mission_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    owner = next(node for node in tree.body
                 if isinstance(node, ast.ClassDef) and node.name == 'PointToPointMission')
    fn = next(node for node in owner.body
              if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = dict(time=time, ground_height_baseline_cm=ground_height_baseline_cm,
                     Int16=S, VehicleLandDetected=S, UInt8=S,
                     Trigger=S(Request=S, Response=S),
                     VehicleStatus=S(ARMING_STATE_DISARMED=1, ARMING_STATE_ARMED=2,
                                     NAVIGATION_STATE_OFFBOARD=14, NAVIGATION_STATE_AUTO_LAND=18))
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class MissionReadinessTest(unittest.TestCase):
    def node(self):
        node = S(state='IDLE', ACTIVE_STATES={'TAKEOFF'},
                 terminal_behavior='guided_land_handoff', vehicle_status=S(arming_state=1),
                 last_vehicle_status_monotonic=time.monotonic(), px4_status_timeout_sec=1.2,
                 recent_ground_heights=deque(), ground_height_max_cm=20.,
                 ground_height_cm=None, active_target=None, route_kick_remaining=0,
                 require_rc_offboard=True, flight_session_seen_armed=False,
                 current_height_cm=None, _sensors_fresh=lambda:True,
                 _update_pose=lambda:True)
        node._start_readiness = lambda: callback('_start_readiness')(node)
        return node

    def fill(self, node):
        for _ in range(6):
            callback('_height_callback')(node, S(data=10))

    def test_missing_status_fails_and_does_not_collect_baseline(self):
        node = self.node()
        node.vehicle_status = None
        self.fill(node)
        reply = callback('_readiness_callback')(node, None, S())
        self.assertFalse(reply.success)
        self.assertIn('飞控状态', reply.message)
        self.assertEqual(len(node.recent_ground_heights), 0)

    def test_disarmed_status_collects_baseline_and_check_has_no_flight_effect(self):
        node = self.node()
        self.fill(node)
        reply = callback('_readiness_callback')(node, None, S())
        self.assertTrue(reply.success)
        self.assertIn('10.0cm', reply.message)
        self.assertEqual(node.state, 'IDLE')
        self.assertIsNone(node.active_target)
        self.assertIsNone(node.ground_height_cm)
        self.assertEqual(node.route_kick_remaining, 0)

    def test_armed_status_does_not_collect_ground_samples(self):
        node = self.node()
        node.vehicle_status.arming_state = 2
        self.fill(node)
        self.assertEqual(len(node.recent_ground_heights), 0)
        self.assertFalse(node._start_readiness()[0])

    def test_stale_status_does_not_collect_or_pass_readiness(self):
        node = self.node()
        self.fill(node)
        node.last_vehicle_status_monotonic -= 2.
        self.fill(node)
        self.assertEqual(len(node.recent_ground_heights), 6)
        self.assertFalse(node._start_readiness()[0])

    def test_prearm_baseline_remains_available_after_rc_arms(self):
        node = self.node()
        self.fill(node)
        node.vehicle_status.arming_state = 2
        self.assertTrue(node._start_readiness()[0])

    def test_sensor_timeout_blocks_readiness(self):
        node = self.node()
        self.fill(node)
        node._sensors_fresh = lambda:False
        self.assertFalse(node._start_readiness()[0])

    def test_active_or_landing_task_is_not_reported_startable(self):
        for state in ['TAKEOFF', 'FC_LANDING']:
            node = self.node()
            self.fill(node)
            node.state = state
            self.assertFalse(node._start_readiness()[0])

    def test_baseline_requires_five_samples(self):
        self.assertIsNone(ground_height_baseline_cm([], 100., 20.))
        self.assertIsNone(ground_height_baseline_cm([(99., 10.)]*4, 100., 20.))
        self.assertEqual(ground_height_baseline_cm([(99., 10.)]*5, 100., 20.), 10.)

    def test_baseline_rejects_old_future_unstable_or_invalid_measurements(self):
        for samples in [[(39., 10.)]*5, [(101., 10.)]*5,
                        [(99., 8.), (99., 12.)] + [(99., 10.)]*3]:
            self.assertIsNone(ground_height_baseline_cm(samples, 100., 20.))
        for height in [float('nan'), float('inf'), 0., -1., 21.]:
            self.assertIsNone(ground_height_baseline_cm([(99., height)]*5, 100., 20.))


if __name__ == '__main__':
    unittest.main()
