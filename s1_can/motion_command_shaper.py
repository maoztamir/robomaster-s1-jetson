"""Bounded command pulses for a chassis that ignores small wheel commands.

The angular pulse magnitude is supported by the clockwise recording; opposite
direction and pulse response still require supervised hardware verification.
"""
import math


class CommandPulse:
    def __init__(self, gain, magnitude, average_limit, settle_seconds, duration=.10):
        if not all(math.isfinite(v) and v > 0 for v in
                   (gain, magnitude, average_limit, settle_seconds, duration)):
            raise ValueError('Pulse settings must be finite and positive')
        self.gain = gain
        self.magnitude = magnitude
        self.average_limit = average_limit
        self.settle_seconds = settle_seconds
        if duration > .25:
            raise ValueError('Pulse duration must not exceed 250 ms')
        self.duration = duration
        self.last_at = None
        self.sign = 0
        self.credit = 0.
        self.until = None
        self.next_at = 0.

    def active(self, now):
        return self.until is not None and now < self.until-1e-9

    def pause(self, now):
        """Preserve pending motion while another axis has control."""
        if not math.isfinite(now) or (self.last_at is not None and now < self.last_at):
            raise ValueError('Invalid pulse clock')
        self.last_at = now
        if self.until is not None:
            self.until = None
            self.next_at = max(self.next_at, now+self.settle_seconds)

    def output(self, now, requested):
        if not math.isfinite(now) or not math.isfinite(requested):
            raise ValueError('Non-finite pulse input')
        if self.last_at is not None and now < self.last_at:
            raise ValueError('Pulse clock moved backwards')
        dt = 0. if self.last_at is None else min(.10, now-self.last_at)
        self.last_at = now
        sign = (1 if requested > .001 else -1 if requested < -.001 else 0)
        if sign == 0 or sign != self.sign:
            was_active = self.until is not None
            self.until = None
            self.credit = 0.
            if was_active:
                self.next_at = max(self.next_at, now+self.settle_seconds)
            self.sign = sign
            return 0.
        # Do not accumulate commands across a stall in the caller.
        area = self.gain*self.magnitude*self.duration
        self.credit = min(area, self.credit+min(abs(requested), self.average_limit)*dt)
        if self.until is not None:
            if now < self.until-1e-9:
                return sign*self.magnitude
            self.until = None
            # At least one zero command separates successive pulses.
            self.next_at = max(self.next_at, now+self.settle_seconds)
            return 0.
        if now >= self.next_at and self.credit >= area-1e-12:
            self.credit = 0.
            self.until = now+self.duration
            return sign*self.magnitude
        return 0.


class MotionCommandShaper:
    def __init__(self, forward_gain, angular_gain, allow_reverse=False, turn_pulses=False):
        self.allow_reverse = allow_reverse
        self.forward_gain = forward_gain
        self.angular_gain = angular_gain
        self.turn_pulses = turn_pulses
        # Linear pulse peaks retain the existing 0.08 m/s command bound.
        self.linear = CommandPulse(forward_gain, .08/forward_gain, .08, .10, duration=.25)
        # Turning pulse peaks are larger than the old continuous turn limit.
        # 100 ms pulses are separated by >=350 ms of zero turning command.
        self.angular = CommandPulse(angular_gain, .65, .15, .35)

    def output(self, now, vx, vy, wz):
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            raise ValueError('Non-finite velocity')
        if abs(vy) > .001 or (vx < -.001 and not self.allow_reverse):
            raise ValueError('Unsupported reverse or lateral command')
        bounded_x = min(.08, max(-.08 if self.allow_reverse else 0., vx))
        # Positive intent below this floor otherwise takes longer than the
        # no-progress timeout to accumulate a useful 2 cm nominal pulse.
        pulse_request = math.copysign(max(.01,abs(bounded_x)),bounded_x) if abs(bounded_x)>.001 else 0.
        if not self.turn_pulses:
            if abs(bounded_x)>=.07:
                self.linear.output(now,0.)
                linear=bounded_x/self.forward_gain
            else:
                linear=self.linear.output(now,pulse_request)
            return linear,0.,min(.15,max(-.15,wz))/self.angular_gain
        if self.linear.active(now):
            self.angular.pause(now)
            return self.linear.output(now,pulse_request),0.,0.
        angular = self.angular.output(now, wz)
        # Correct heading while stationary, with clearance for the whole footprint.
        if angular:
            self.linear.pause(now)
            return 0., 0., angular
        if abs(bounded_x) >= .07:
            self.linear.output(now, 0.)
            return bounded_x/self.forward_gain, 0., 0.
        return self.linear.output(now, pulse_request), 0., 0.
