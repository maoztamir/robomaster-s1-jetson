"""Latched tracking gate using receipt time and source timestamp freshness."""
import math


class TrackingGuard:
    def __init__(self, timeout=1.5, low_quality_grace=.4):
        self.timeout = timeout
        self.low_quality_grace = low_quality_grace
        self.last = None
        self.low_quality_since = None
        self.ready = False
        self.stopped = False
        self.reason = 'waiting for first healthy tracking status'

    def update(self, healthy, age, now, explicitly_lost=False):
        if self.ready and self.last is not None and now - self.last > self.timeout:
            self.stopped = True
            self.reason = 'tracking status delivery timed out'
        timestamp_valid = math.isfinite(age) and -.1 <= age <= self.timeout
        if not timestamp_valid:
            if not math.isfinite(age):
                self.reason = 'tracking status has an invalid timestamp'
            elif age < -.1:
                self.reason = 'tracking status timestamp is in the future'
            else:
                self.reason = 'tracking status timestamp is stale'
            if self.ready:
                self.stopped = True
            self.last = None
            self.low_quality_since = None
        elif explicitly_lost:
            self.reason = 'stereo odometry explicitly reported lost'
            if self.ready:
                self.stopped = True
            self.last = None
            self.low_quality_since = None
        elif not healthy:
            self.reason = 'stereo odometry has too few inliers'
            if self.low_quality_since is None:
                self.low_quality_since = now
            elif self.ready and now - self.low_quality_since >= self.low_quality_grace:
                self.stopped = True
            if not self.ready:
                self.last = None
        elif not self.stopped:
            self.last = now
            self.low_quality_since = None
            self.ready = True
            self.reason = None

    def permits(self, now):
        if self.ready and (self.last is None or now - self.last > self.timeout):
            self.stopped = True
            if self.reason is None:
                self.reason = 'tracking status delivery timed out'
        return self.ready and not self.stopped
