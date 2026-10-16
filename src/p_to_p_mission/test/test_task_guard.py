import unittest
from pathlib import Path
from p_to_p_mission.task_guard import envelope_reason, stage_reason


class TaskGuardTest(unittest.TestCase):
    def test_radius_uses_vector_norm_and_relative_origin(self):
        self.assertIsNone(envelope_reason(150, 200, 60, 100, 150, 150, 120))
        self.assertIsNotNone(envelope_reason(210, 260, 60, 100, 150, 150, 120))
        self.assertIsNone(envelope_reason(250, 150, 120, 100, 150, 150, 120))
        self.assertIsNotNone(envelope_reason(250.01, 150, 60, 100, 150, 150, 120))

    def test_height_and_invalid_observation(self):
        self.assertIsNotNone(envelope_reason(0, 0, 121, 0, 0, 150, 120))
        self.assertIsNotNone(envelope_reason(float('nan'), 0, 60, 0, 0, 150, 120))

    def test_each_motion_stage_times_out(self):
        for state, limit in [('TAKEOFF',20), ('GOTO_B',20), ('HOLD_B',20), ('DESCEND_B',30)]:
            self.assertIsNone(stage_reason(state, limit-.01, 20,20,20,30,'guided_land_handoff'))
            self.assertIsNotNone(stage_reason(state, limit, 20,20,20,30,'guided_land_handoff'))

    def test_hold_mode_and_latched_fault_do_not_repeat_timeout(self):
        for state in ['FAULT_HOLD', 'FC_LANDING', 'IDLE']:
            self.assertIsNone(stage_reason(state, 1000,20,20,20,30,'guided_land_handoff'))
        self.assertIsNone(stage_reason('HOLD_B',1000,20,20,20,30,'hold'))

    def test_guard_runs_before_normal_target_and_does_not_emit_land(self):
        text = (Path(__file__).parents[1]/'p_to_p_mission'/'mission_node.py').read_text(encoding='utf8')
        guarded = text.split('reason = self._task_guard_reason()')[1].split('if self.state == "TAKEOFF"')[0]
        self.assertIn('self._fault_hold(', guarded)
        self.assertNotIn('mission_complete_pub', guarded)
        self.assertIn('return', guarded)
