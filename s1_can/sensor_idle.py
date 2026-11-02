"""ROS-independent inactivity policy and expiring motion permission."""
import math


class SensorMotionGate:
    def __init__(self, enabled=False, timeout=.6):
        self.enabled = enabled
        self.timeout = timeout
        self.ready_at = None

    def update(self, ready, now):
        self.ready_at = now if ready else None

    def permits(self, now):
        return not self.enabled or (self.ready_at is not None and
                                    0 <= now - self.ready_at <= self.timeout)


class SensorIdlePolicy:
    def __init__(self, now, timeout=300.):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Idle timeout must be positive and finite')
        self.timeout = timeout
        self.last_activity = now
        self.hold = False
        self.active_goals = set()

    def activity(self, now):
        self.last_activity = now

    def command(self, values, now):
        if not all(math.isfinite(v) for v in values) or any(abs(v) > .001 for v in values):
            self.activity(now)

    def should_sleep(self, now):
        return (not self.hold and not self.active_goals and
                now - self.last_activity >= self.timeout)
