"""ROS-independent validation of the pre-arm ground range baseline."""

import math
import statistics


def ground_height_baseline_cm(samples, now, max_height_cm):
    """Return a stable recent baseline, or None; never invent missing samples."""
    heights = [height for stamp, height in samples
               if math.isfinite(stamp) and 0.0 <= now - stamp <= 60.0]
    if (len(heights) < 5
            or not all(math.isfinite(height) and 0.0 < height <= max_height_cm
                       for height in heights)
            or max(heights) - min(heights) > 3.0):
        return None
    return float(statistics.median(heights))
