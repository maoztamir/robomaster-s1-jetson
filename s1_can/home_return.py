"""Geometry and bounded velocities for a short supervised reverse return."""
import math


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def validate_return(start, home):
    if not all(math.isfinite(v) for v in (*start, *home)):
        raise ValueError('Invalid return pose')
    dx, dy = home[0]-start[0], home[1]-start[1]
    forward = math.cos(start[2])*dx + math.sin(start[2])*dy
    sideways = -math.sin(start[2])*dx + math.cos(start[2])*dy
    if math.hypot(dx, dy) > .4 or forward > .02 or abs(sideways) > .06:
        raise ValueError('Base must be within 40 cm, almost directly behind the robot')
    if abs(angle(home[2]-start[2])) > math.radians(14):
        raise ValueError('Base heading requires too much rotation for this short return')
    if math.hypot(dx, dy) > .03:
        reverse_heading = math.atan2(-dy, -dx)
        if abs(angle(reverse_heading-start[2])) > math.radians(14):
            raise ValueError('Return path requires too much rotation')


def return_velocity(pose, home):
    """Return reverse-only SI velocity, turn velocity and arrival status."""
    dx, dy = home[0]-pose[0], home[1]-pose[1]
    distance = math.hypot(dx, dy)
    desired_heading = home[2] if distance <= .03 else math.atan2(-dy, -dx)
    heading_error = angle(desired_heading-pose[2])
    if distance <= .03 and abs(heading_error) <= math.radians(4):
        return 0., 0., True
    if abs(heading_error) > math.radians(3):
        # Correct heading while stationary; don't introduce sideways drive.
        return 0., math.copysign(.25, heading_error), False
    return (-.08 if distance > .03 else 0.), 0., False
