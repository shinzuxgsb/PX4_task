"""Pure helpers for a monotonic, rate-bounded Offboard touchdown target."""


def next_descent_setpoint_cm(
    previous_cm: float,
    measured_cm: float,
    ground_cm: float,
    dt_sec: float,
    rate_cm_s: float,
    max_lead_cm: float,
) -> float:
    """Lower the target without stepping below ground or jumping upward on range noise.

    The target can lead the measured height by at most ``max_lead_cm`` under
    normal readings. If a reading jumps upward, the target freezes instead of
    chasing that jump with a climb command.
    """
    dt_sec = min(max(dt_sec, 0.0), 0.25)
    desired_cm = previous_cm - rate_cm_s * dt_sec
    lead_limited_cm = min(previous_cm, measured_cm - max_lead_cm)
    return max(ground_cm, desired_cm, lead_limited_cm)
