"""Map-free, forward-only patrol policy. Distances are relative to the scanner."""
import math
import random


class Patrol:
    def __init__(self, forward=0.15, turn=0.35, clearance=0.75, radius=0.4, seed=7):
        if not (0 < forward <= 1 and 0 < turn <= 1 and clearance > radius > 0):
            raise ValueError('Invalid patrol speeds or clearances')
        self.forward, self.turn = forward, turn
        self.clearance, self.radius = clearance, radius
        self.random = random.Random(seed)
        self.direction = 1
        self.turning = False
        self.until = 0.0
        self.next_turn = 0.0

    def command(self, ranges, angle_min, increment, range_min, range_max, now, yaw=0.0):
        if not (len(ranges) >= 36 and math.isfinite(increment) and increment > 0
                and math.isfinite(angle_min) and math.isfinite(yaw)
                and 0 <= range_min < range_max < math.inf
                and range_max > self.clearance + .2):
            return 0., 0., 'invalid scan'
        bins = [[] for _ in range(36)]
        for i, distance in enumerate(ranges):
            angle = (angle_min + i * increment + yaw + math.pi) % (2 * math.pi)
            bins[min(35, int(angle / (2 * math.pi) * 36))].append(distance)
        clear = []
        for values in bins:
            # RPLIDAR can report finite returns slightly beyond range_max.
            # Saturate distant returns; never promote NaN/negative/near-invalid
            # values into clear space. Clearance thresholds remain below max.
            valid = [min(v, range_max) for v in values
                     if (math.isfinite(v) and v >= range_min) or v == math.inf]
            if not values or len(valid) < 0.8 * len(values):
                return 0., 0., 'incomplete scan'
            clear.append(min(valid))
        if min(clear) < self.radius:
            return 0., 0., 'obstacle inside turning clearance'
        front = min(clear[13:23])
        if not self.next_turn:
            self.next_turn = now + self.random.uniform(25, 45)
        if not self.turning and (front < self.clearance or now >= self.next_turn):
            self.turning = True
            left, right = min(clear[23:30]), min(clear[6:13])
            self.direction = (1 if left > right else -1) if abs(left-right) > .15 else self.random.choice([-1, 1])
            self.until = now + self.random.uniform(1, 3)
        if self.turning:
            if now >= self.until and front > self.clearance + .2:
                self.turning = False
                self.next_turn = now + self.random.uniform(25, 45)
            else:
                return 0., self.direction * self.turn, 'turning'
        return self.forward, 0., 'forward'
