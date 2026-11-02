"""One supervised short reverse return to the saved base. L1 stops motion."""
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
import yaml

from hardware_navigation_trial import Trial
from s1_can.home_pose import home_parameters
from s1_can.home_return import return_velocity, validate_return
from s1_can.motion_command_shaper import MotionCommandShaper


class ReturnHome(Trial):
    def __init__(self):
        super().__init__(False)
        root = Path(__file__).resolve().parents[1]
        data = yaml.safe_load((root/'maps/hardware/home-pose.yaml').read_text())
        params = home_parameters(root/'maps/hardware/home-pose.yaml', data['map_yaml'])
        self.home = (params['initial_pose.x'], params['initial_pose.y'], params['initial_pose.yaw'])
        self.initial_distance = None
        self.arrival_at = None
        self.arrived = False
        self.progress_pose = None
        self.progress_at = None
        self.command_shaper=MotionCommandShaper(self.conversion.forward_gain,
                                                self.conversion.angular_gain,allow_reverse=True)

    def scan(self, msg):
        super().scan(msg)
        rear = []
        for i, distance in enumerate(msg.ranges):
            a = msg.angle_min+i*msg.angle_increment-math.pi
            if (math.isfinite(distance) and max(.15, msg.range_min) <= distance <= min(8.,msg.range_max)
                    and abs(math.atan2(math.sin(a),math.cos(a))) <= math.radians(35)):
                rear.append(distance)
        self.guard.front_clear = len(rear) >= 20 and min(rear) > .35

    def zero(self):
        if self.commands_sent or self.lease is not None:
            super().zero()

    def tick(self):
        now = time.monotonic()
        if self.lease is not None and now-self.last_lease >= .4:
            try:
                if self.lease.poll() is not None:
                    raise BrokenPipeError('Manual control lease ended')
                self.lease.stdin.write('keep\n'); self.lease.stdin.flush(); self.last_lease=now
            except (OSError,ValueError) as exc:
                self.guard.fault=str(exc)
        if not self.armed:
            return
        try:
            pose, age, _ = self.pose()
            vx, wz, reached = return_velocity(pose,self.home)
            reason = self.guard.reason(now,pose,age,rotating=abs(wz)>.01)
            if reason:
                raise RuntimeError(reason.replace('coverage ahead','coverage behind'))
            distance=math.hypot(pose[0]-self.home[0],pose[1]-self.home[1])
            if distance > self.initial_distance+.025:
                raise RuntimeError('Return motion is moving away from the base')
            if reached:
                self.zero()
                if self.arrival_at is None: self.arrival_at=now
                if now-self.arrival_at >= .5:
                    self.arrived=True; self.armed=False
                return
            self.arrival_at=None
            previous=self.progress_pose
            yaw_progress=abs(math.atan2(math.sin(pose[2]-previous[2]),math.cos(pose[2]-previous[2])))
            if math.hypot(pose[0]-previous[0],pose[1]-previous[1])>.005 or yaw_progress>math.radians(1):
                self.progress_pose=pose; self.progress_at=now
            elif now-self.progress_at>4:
                raise RuntimeError('No localized motion progress; return stopped')
            msg=Twist()
            # Reuse the existing command scale, with reverse response checked by live pose.
            msg.linear.x,msg.linear.y,msg.angular.z=self.command_shaper.output(now,vx,0.,wz)
            self.pub.publish(msg); self.commands_sent+=1
        except (RuntimeError,ValueError,TransformException) as exc:
            self.guard.fault=str(exc); self.zero()

    def return_home(self):
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)
            try:
                pose,age,_=self.pose()
                reason=self.guard.reason(time.monotonic(),pose,age,rotating=True)
                if not reason: break
            except TransformException: reason='Localization unavailable'
        else: raise RuntimeError(reason)
        validate_return(pose,self.home)
        self.isolate_teleop()
        pose,age,_=self.pose()
        validate_return(pose,self.home)
        reason=self.guard.reason(time.monotonic(),pose,age,rotating=True)
        if reason: raise RuntimeError(reason)
        self.initial_distance=math.hypot(pose[0]-self.home[0],pose[1]-self.home[1])
        self.guard.start_pose=pose; self.guard.start_at=time.monotonic(); self.armed=True
        self.progress_pose=pose; self.progress_at=self.guard.start_at
        print(f'Return started: {self.initial_distance:.3f} m to base, reverse speed <=0.08 m/s. L1 stops.',flush=True)
        while rclpy.ok() and not self.arrived:
            rclpy.spin_once(self,timeout_sec=.05)
            if self.guard.fault: raise RuntimeError(self.guard.fault)
        if not self.arrived: raise RuntimeError('Return interrupted')
        pose,_,_=self.pose()
        print(f'Return completed: localized position error '
              f'{math.hypot(pose[0]-self.home[0],pose[1]-self.home[1]):.3f} m.',flush=True)


def main():
    rclpy.init(); node=ReturnHome()
    try:
        node.return_home()
    except (RuntimeError,ValueError,KeyboardInterrupt) as exc:
        print('Return stopped: '+str(exc),flush=True)
        raise SystemExit(1)
    finally:
        node.finish(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
