"""Deterministic waypoint patrol for visual exploration, not navigation."""
import math


def patrol_pose(points, speed, elapsed):
    """Return XYZ and yaw along a closed loop at a constant speed in m/s."""
    if speed <= 0 or elapsed < 0 or len(points) < 2:
        raise ValueError('Patrol needs two points, positive speed and nonnegative time')
    segments = [(a, b, math.dist(a, b)) for a, b in zip(points, points[1:] + points[:1])]
    length = sum(s[2] for s in segments)
    if length <= 0:
        raise ValueError('Patrol has no length')
    progress = (elapsed * speed) % length
    for a, b, distance in segments:
        if distance and progress < distance:
            ratio = progress / distance
            position = tuple(a[i] + (b[i]-a[i])*ratio for i in range(3))
            return position, math.degrees(math.atan2(b[1]-a[1], b[0]-a[0]))
        progress -= distance
    return points[0], 0.0


def warehouse_patrols():
    """Routes use rack aisles and front/back crossings; not a collision planner."""
    return [
        ([(-6,-9,2.5),(-6,14,2.5),(6,14,2.5),(6,-9,2.5)], .7),
        ([(6,14,3.3),(6,-9,3.3),(-6,-9,3.3),(-6,14,3.3)], .6),
        ([(-7.8,-9,.03),(-7.8,-4.7,.03),(-3.8,-4.7,.03),(-3.8,13,.03),
          (-2.3,13,.03),(-2.3,-4.7,.03),(-7.8,-4.7,.03)], .4),
        ([(7.8,-9,.03),(7.8,-4.7,.03),(4,-4.7,.03),(4,13,.03),
          (5,13,.03),(5,-4.7,.03),(7.8,-4.7,.03)], .3),
    ]
