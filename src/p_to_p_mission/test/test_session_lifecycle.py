"""Inert callback simulations: no ROS nodes, publishers or aircraft commands."""
from collections import deque
from types import SimpleNamespace as S
import time
import unittest

from test_readiness import callback


class MissionSessionTest(unittest.TestCase):
    def node(self, state='TAKEOFF'):
        events = []
        node = S(state=state, ACTIVE_STATES={'TAKEOFF','GOTO_B','HOLD_B','DESCEND_B','FAULT_HOLD'},
                 vehicle_status=S(arming_state=2, nav_state=14),
                 last_vehicle_status_monotonic=time.monotonic(), px4_status_timeout_sec=1.2,
                 land_detected=None, last_land_detected_monotonic=0.,
                 flight_session_seen_armed=True, require_rc_offboard=True,
                 active_target=S(z=60.), a_pose=S(), b_target=S(), ground_height_cm=10.,
                 recent_ground_heights=deque([(time.monotonic(),10.)]*6),
                 ground_height_max_cm=20., route_kick_remaining=1,
                 last_descent_tick_monotonic=123., reached_since=123.,
                 _update_pose=lambda:True, _sensors_fresh=lambda:True,
                 terminal_behavior='guided_land_handoff', route_choice=3,
                 route_pub=S(publish=lambda msg:events.append('route')),
                 _publish_target=lambda:events.append('target'),
                 _task_guard_reason=lambda:None,
                 _target_reached_stably=lambda:False)
        def enter(state, detail):
            node.state = state
            node.reached_since = None
            events.append(state)
        node._enter_state = enter
        for name in ['_reset_if_landed_and_disarmed','_rc_offboard_active','_start_readiness']:
            fn = callback(name)
            setattr(node, name, lambda fn=fn:fn(node))
        return node, events

    def status(self, node, armed=2, mode=14):
        callback('_status_callback')(node, S(arming_state=armed, nav_state=mode))

    def landed(self, node, value=True):
        callback('_land_detected_callback')(node, S(landed=value))

    def test_pilot_exit_cancels_targets_and_route_kick(self):
        node, events = self.node()
        self.status(node, mode=2)
        self.assertEqual(node.state, 'PILOT_TAKEOVER')
        self.assertIsNone(node.active_target)
        self.assertEqual(node.route_kick_remaining, 0)
        callback('_timer_callback')(node)
        self.assertNotIn('target', events)
        self.assertNotIn('route', events)

    def test_guard_cancels_route_and_never_advances_to_next_stage(self):
        node, events = self.node('DESCEND_B')
        node.route_kick_remaining = 0
        node._task_guard_reason = lambda:'DESCEND_B timeout'
        node._fault_hold = lambda reason:node._enter_state('FAULT_HOLD', reason)
        node._target_reached_stably = lambda:True
        callback('_timer_callback')(node)
        self.assertEqual(node.state, 'FAULT_HOLD')
        self.assertEqual(events, ['FAULT_HOLD', 'target'])
        self.assertEqual(node.route_kick_remaining, 0)

    def test_switch_back_cannot_restart_or_abort_hold_old_mission(self):
        node, _ = self.node()
        self.status(node, mode=2)
        self.status(node, mode=14)
        self.assertFalse(node._start_readiness()[0])
        self.assertFalse(callback('_abort_hold_callback')(node, None, S()).success)
        self.assertEqual(node.state, 'PILOT_TAKEOVER')

    def test_expected_native_land_is_not_a_takeover(self):
        node, _ = self.node('FC_LANDING')
        self.status(node, mode=18)
        self.assertEqual(node.state, 'FC_LANDING')
        self.landed(node)
        self.assertEqual(node.state, 'FC_LANDING')  # Still armed.

    def test_disarm_without_landed_does_not_reset_or_publish(self):
        node, events = self.node('FC_LANDING')
        self.status(node, armed=1, mode=18)
        self.assertEqual(node.state, 'FC_LANDING')
        callback('_timer_callback')(node)
        self.assertNotIn('route', events)

    def test_cancelling_native_land_by_reentering_offboard_does_not_resume(self):
        node, _ = self.node('FC_LANDING')
        self.status(node, mode=18)
        self.status(node, mode=14)
        self.assertEqual(node.state, 'PILOT_TAKEOVER')
        self.assertIsNone(node.active_target)

    def test_reset_accepts_both_delivery_orders_and_clears_old_session(self):
        for land_first in [True, False]:
            node, _ = self.node('FC_LANDING')
            if land_first:
                self.landed(node)
                self.status(node, armed=1, mode=18)
            else:
                self.status(node, armed=1, mode=18)
                self.landed(node)
            self.assertEqual(node.state, 'IDLE')
            for field in ['active_target','a_pose','b_target','ground_height_cm']:
                self.assertIsNone(getattr(node, field))
            self.assertEqual(node.route_kick_remaining, 0)
            self.assertEqual(len(node.recent_ground_heights), 0)
            self.assertFalse(node._start_readiness()[0])  # New baseline required.
            for _ in range(6):
                callback('_height_callback')(node, S(data=10))
            self.assertTrue(node._start_readiness()[0])

    def test_stale_or_false_land_does_not_reset(self):
        node, _ = self.node('FC_LANDING')
        node.land_detected = S(landed=True)
        node.last_land_detected_monotonic = time.monotonic()-2.
        self.status(node, armed=1, mode=18)
        self.assertEqual(node.state, 'FC_LANDING')
        self.landed(node, False)
        self.assertEqual(node.state, 'FC_LANDING')

    def test_stale_disarmed_status_does_not_reset(self):
        node, _ = self.node('FC_LANDING')
        node.vehicle_status.arming_state = 1
        node.last_vehicle_status_monotonic = time.monotonic()-2.
        self.landed(node)
        self.assertEqual(node.state, 'FC_LANDING')

    def test_timer_never_sends_targets_on_stale_or_disarmed_status(self):
        for stale in [True, False]:
            node, events = self.node()
            if stale:
                node.last_vehicle_status_monotonic -= 2.
            else:
                node.vehicle_status.arming_state = 1
            callback('_timer_callback')(node)
            self.assertEqual(events, [])

    def test_normal_armed_offboard_still_forwards_route_and_target(self):
        node, events = self.node()
        callback('_timer_callback')(node)
        self.assertEqual(events, ['route', 'target'])

    def test_real_start_refuses_disarmed_even_with_valid_readiness(self):
        node, _ = self.node('IDLE')
        node.vehicle_status.arming_state = 1
        self.assertTrue(node._start_readiness()[0])
        reply = callback('_start_callback')(node, None, S())
        self.assertFalse(reply.success)
        self.assertEqual(node.state, 'IDLE')


if __name__ == '__main__':
    unittest.main()
