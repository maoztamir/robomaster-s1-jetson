"""Bounded conversion for the first forward-only hardware navigation trial."""
import math


def validate_selected_goal(start, target, frame):
    if frame != 'localization_map' or not all(math.isfinite(v) for v in target):
        raise RuntimeError('Goal must be a finite pose on this map')
    dx,dy=target[0]-start[0],target[1]-start[1]
    forward=math.cos(start[2])*dx+math.sin(start[2])*dy
    sideways=-math.sin(start[2])*dx+math.cos(start[2])*dy
    heading=math.atan2(math.sin(target[2]-start[2]),math.cos(target[2]-start[2]))
    if not .10<=forward<=.35 or abs(sideways)>.05 or math.hypot(dx,dy)>.35:
        raise RuntimeError('Choose a point 10–35 cm directly ahead for this first navigation mode')
    if abs(heading)>math.radians(15):
        raise RuntimeError('Keep the goal arrow facing the current forward direction')


class TrialConversion:
    def __init__(self, forward_gain, angular_gain, minimum_forward_command=0.):
        if not all(math.isfinite(v) and v > 0 for v in (forward_gain, angular_gain)):
            raise ValueError('Motion calibration gains must be positive and finite')
        self.forward_gain, self.angular_gain = forward_gain, angular_gain
        if not math.isfinite(minimum_forward_command) or not 0<=minimum_forward_command<=.08/forward_gain:
            raise ValueError('Minimum forward command must respect the forward speed bound')
        self.minimum_forward_command=minimum_forward_command

    def convert(self, vx, vy, wz):
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            raise ValueError('Non-finite Nav2 velocity')
        if vx < -.001 or abs(vy) > .001:
            raise ValueError('First trial permits forward motion and heading correction only')
        forward=min(.08,max(0.,vx))/self.forward_gain
        if self.minimum_forward_command:
            forward=max(self.minimum_forward_command,forward) if vx>.001 else 0.
        return (forward,
                0., min(.15, max(-.15, wz)) / self.angular_gain)


class TrialGuard:
    def __init__(self):
        self.scan_at = self.odom_at = self.joy_at = None
        self.front_clear = self.turn_clear = False
        self.l1 = True
        self.start_at = None
        self.start_pose = None
        self.fault = None

    def reason(self, now, pose, tf_age, rotating=False):
        if self.fault:
            return self.fault
        reason = None
        for name, stamp, limit in (('LiDAR', self.scan_at, .7), ('odometry', self.odom_at, .7),
                                   ('gamepad', self.joy_at, 1.)):
            if stamp is None or not 0 <= now-stamp <= limit:
                reason = name + ' is unavailable or stale'
                break
        if self.l1:
            reason = 'L1 stop control pressed or not yet verified released'
        elif not self.front_clear:
            reason = 'Insufficient clear LiDAR coverage ahead'
        elif rotating and not self.turn_clear:
            reason = 'Insufficient clearance for heading correction'
        elif not math.isfinite(tf_age) or not -.6 <= tf_age <= 1.:
            reason = 'Localization transform is stale'
        elif not all(math.isfinite(v) for v in pose):
            reason = 'Invalid localized pose'
        if self.start_at is not None:
            if now-self.start_at > 20.:
                reason = 'Trial duration limit reached'
            if self.start_pose is not None:
                a = self.start_pose
                if math.hypot(pose[0]-a[0], pose[1]-a[1]) > .45:
                    reason = 'Trial travel limit reached'
                yaw = math.atan2(math.sin(pose[2]-a[2]), math.cos(pose[2]-a[2]))
                if abs(yaw) > math.radians(15):
                    reason = 'Unexpected heading change exceeded 15 degrees'
            if reason:
                self.fault = reason  # Do not resume merely because data recovers.
        return reason
