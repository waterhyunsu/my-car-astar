"""Front-sector LaserScan checks, independent of ROS transport."""
import math


def front_distance(scan, half_angle):
    """Return nearest usable front ray, or None for unusable scan data.

    Positive infinity means no return within range; NaN, negative infinity,
    zero and out-of-range finite measurements are ignored.
    The scan frame must have +X pointing forward.
    """
    if (not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment)
            or scan.angle_increment == 0.0
            or not math.isfinite(scan.range_min)
            or not math.isfinite(scan.range_max)
            or not 0.0 <= scan.range_min < scan.range_max):
        return None
    distances = []
    for i, value in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(angle) > half_angle + 1e-9:
            continue
        if value == math.inf:
            distances.append(value)
        elif math.isfinite(value) and value > 0.0 and scan.range_min <= value <= scan.range_max:
            distances.append(value)
    return min(distances) if distances else None
