"""Local task limits using fresh observations, not a hardware geofence."""
import math


def envelope_reason(x, y, height, origin_x, origin_y, radius, max_height):
    if not all(math.isfinite(v) for v in (x, y, height, origin_x, origin_y)):
        return 'nonfinite task observation'
    if math.hypot(x-origin_x, y-origin_y) > radius:
        return 'task horizontal radius exceeded'
    if height > max_height:
        return 'task height ceiling exceeded'
    return None


def stage_reason(state, elapsed, takeoff, travel, hold, descent, terminal_behavior):
    limits = {'TAKEOFF': takeoff, 'GOTO_B': travel, 'DESCEND_B': descent}
    if terminal_behavior != 'hold':
        limits['HOLD_B'] = hold
    limit = limits.get(state)
    if limit is not None and elapsed >= limit:
        return f'{state} timeout ({limit:g}s)'
    return None
