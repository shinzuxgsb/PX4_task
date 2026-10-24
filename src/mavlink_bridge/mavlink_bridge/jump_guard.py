"""Pure helpers for the SLAM odometry jump guard.

Kept free of ROS imports so the decision logic can be unit tested on the
ground without starting a node.
"""

from __future__ import annotations


def classify_step(
    jump_m: float,
    dt_s: float,
    hard_limit_m: float,
    min_jump_m: float,
    max_speed_mps: float,
) -> bool:
    """Return True when one odometry step should be rejected.

    Two independent checks:

    1. A single-frame hard displacement limit catches gross relocalizations
       (metres to hundreds of metres) even when the timestamp gap is long.
    2. A time-normalized speed limit catches a jump that is too fast for the
       airframe.  Normal flight with a long odometry gap is not rejected:
       0.5 m over 0.43 s is only 1.16 m/s.
    """
    if jump_m > hard_limit_m:
        return True
    if jump_m < min_jump_m:
        return False
    if dt_s <= 1e-3:
        # Same/backwards timestamp: a non-trivial displacement is suspicious.
        return True
    return jump_m / dt_s > max_speed_mps
