"""Ideal planar navigation geometry; not a tyre/contact dynamics model."""
import math


def integrate(pose, velocity, dt):
    x, y, yaw = pose
    vx, vy, wz = velocity
    # Exact constant body-twist integration, including lateral motion.
    if abs(wz) < 1e-8:
        dx, dy = vx * dt, vy * dt
    else:
        a = wz * dt
        dx = (vx * math.sin(a) + vy * (math.cos(a) - 1)) / wz
        dy = (vx * (1 - math.cos(a)) + vy * math.sin(a)) / wz
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * dx - s * dy, y + s * dx + c * dy, math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))


def overlaps(pose, size, box):
    """Separating-axis test: oriented robot rectangle vs axis-aligned box."""
    x, y, yaw = pose
    bx, by, width, height = box[:4]
    c, s = math.cos(yaw), math.sin(yaw)
    axes = [(1., 0.), (0., 1.), (c, s), (-s, c)]
    for ax, ay in axes:
        distance = abs((x-bx)*ax + (y-by)*ay)
        robot_radius = size[0]/2 * abs(c*ax+s*ay) + size[1]/2 * abs(-s*ax+c*ay)
        box_radius = width/2 * abs(ax) + height/2 * abs(ay)
        if distance > robot_radius + box_radius:
            return False
    return True


def ray_distance(origin, angle, boxes, max_range):
    direction = math.cos(angle), math.sin(angle)
    nearest = max_range
    for bx, by, width, height, *_ in boxes:
        lo, hi = 0., nearest
        for p, d, low, high in zip(origin, direction, (bx-width/2, by-height/2), (bx+width/2, by+height/2)):
            if abs(d) < 1e-12:
                if not low <= p <= high:
                    hi = -1.
                    break
            else:
                a, b = (low-p)/d, (high-p)/d
                lo, hi = max(lo, min(a,b)), min(hi, max(a,b))
        if hi >= lo:
            nearest = min(nearest, lo)
    return nearest


def wheel_rates(velocity, radius=.05, lever=.20):
    vx, vy, wz = velocity
    # Physical ideal X-mecanum inverse kinematics, FR, FL, RL, RR.
    return ((vx+vy+lever*wz)/radius, (vx-vy-lever*wz)/radius,
            (vx+vy-lever*wz)/radius, (vx-vy+lever*wz)/radius)
