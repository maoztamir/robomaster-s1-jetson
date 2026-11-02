"""One 30cm forward Nav2 trial; --dry-run plans without commanding motion."""
import argparse
import json
import math
import os
from pathlib import Path
import selectors
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy, LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from s1_can.nav_trial_guard import TrialConversion, TrialGuard, validate_selected_goal


class Trial(Node):
    def __init__(self, dry_run, require_heading=True):
        super().__init__('hardware_navigation_trial')
        root = Path(__file__).resolve().parents[1]
        f = yaml.safe_load((root / 'maps/hardware/forward-motion-calibration-candidate.yaml').read_text())
        w = yaml.safe_load((root / 'maps/hardware/turn-motion-calibration-candidate.yaml').read_text())
        self.conversion = TrialConversion(f['candidate_metres_per_abstract_forward_command_second'],
            w['candidate_radians_per_abstract_angular_command_second'],
            minimum_forward_command=0. if require_heading else .45)
        self.guard = TrialGuard()
        self.last_motion_progress = None
        self.progress_at = None
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.planner = ActionClient(self, ComputePathToPose, '/hardware_navigation/compute_path_to_pose')
        self.controller = ActionClient(self, FollowPath, '/hardware_navigation/follow_path')
        self.pub = None if dry_run else self.create_publisher(Twist, '/cmd_vel', 10)
        self.armed = False
        self.goal = None
        self.latest_command = None
        self.command_at = None
        self.lease = None
        self.last_lease = 0.
        self.commands_sent = 0
        self.selected_goal = None
        self.requested_target = None
        self.motion_trace = []
        self.last_observed_pose = None
        self.outcome = 'not_started'
        self.require_heading = require_heading
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/localization/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(Joy, '/robomaster/joy', self.joy, qos_profile_sensor_data)
        self.create_subscription(Twist, '/hardware_navigation/cmd_vel_preview', self.velocity, 10)
        self.create_subscription(PoseStamped, '/hardware_navigation/trial_goal', self.select_goal, 10)
        self.create_timer(.05, self.tick)

    def age(self, stamp):
        return self.get_clock().now().nanoseconds*1e-9 - (stamp.sec+stamp.nanosec*1e-9)

    def pose(self):
        tf = self.buffer.lookup_transform('localization_map', 'laser', rclpy.time.Time())
        p,q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return (p.x,p.y,yaw), self.age(tf.header.stamp), q

    def scan(self, msg):
        self.guard.front_clear = self.guard.turn_clear = False
        if msg.header.frame_id != 'laser' or not -.2 <= self.age(msg.header.stamp) <= .7:
            self.guard.scan_at = None; return
        front=[]; all_ranges=[]
        for i,r in enumerate(msg.ranges):
            if not math.isfinite(r) or not max(.15,msg.range_min) <= r <= min(8.,msg.range_max):
                continue
            all_ranges.append(r)
            a=msg.angle_min+i*msg.angle_increment
            if abs(math.atan2(math.sin(a),math.cos(a))) <= math.radians(30):front.append(r)
        self.guard.scan_at=time.monotonic()
        self.guard.front_clear=len(front)>=20 and min(front)>.335
        self.guard.turn_clear=len(all_ranges)>=60 and min(all_ranges)>.27

    def odom(self, msg):
        q=msg.pose.pose.orientation; p=msg.pose.pose.position
        norm=sum(v*v for v in (q.x,q.y,q.z,q.w))
        if (all(math.isfinite(v) for v in (p.x,p.y,norm)) and .8<=norm<=1.2
                and -.2<=self.age(msg.header.stamp)<=.7):
            self.guard.odom_at=time.monotonic()
        else:self.guard.odom_at=None

    def joy(self, msg):
        if len(msg.buttons)<=6 or not -.2<=self.age(msg.header.stamp)<=1.:
            self.guard.joy_at=None; return
        self.guard.joy_at=time.monotonic();self.guard.l1=bool(msg.buttons[6])

    def select_goal(self, msg):
        if not self.armed:
            self.selected_goal=msg

    def velocity(self, msg):
        self.latest_command=(msg.linear.x,msg.linear.y,msg.angular.z)
        self.command_at=time.monotonic()

    def zero(self):
        if self.pub is not None and rclpy.ok():
            self.record_command((0.,0.,0.))
            self.pub.publish(Twist())

    def record_command(self, command):
        if self.guard.start_at is not None and len(self.motion_trace)<5000:
            self.motion_trace.append({'t':time.monotonic()-self.guard.start_at,
                'localized_pose':self.last_observed_pose,'requested_si':self.latest_command,
                'abstract_output':command,'fault':self.guard.fault})

    def save_trace(self):
        if self.guard.start_at is None:return
        folder=Path(__file__).resolve().parents[1]/'maps/hardware/trial-traces'
        folder.mkdir(parents=True,exist_ok=True)
        path=folder/f'navigation-{time.time_ns()}.json'
        target=self.requested_target
        data={'outcome':self.outcome,'command_profile':'continuous_conversion',
              'minimum_forward_command':self.conversion.minimum_forward_command,
              'require_final_heading':self.require_heading,
              'target_xy':None if target is None else [target.position.x,target.position.y],
              'trace':self.motion_trace}
        with path.open('x') as out:json.dump(data,out,indent=2)
        print('Command trace saved: '+str(path),flush=True)

    def tick(self):
        now=time.monotonic()
        if self.lease is not None and now-self.last_lease>=.4:
            try:
                if self.lease.poll() is not None:raise BrokenPipeError('Teleop isolation lease ended')
                self.lease.stdin.write('keep\n');self.lease.stdin.flush();self.last_lease=now
            except (OSError, ValueError) as exc:
                self.guard.fault=str(exc)
        if not self.armed:return
        try:
            pose,age,_=self.pose()
        except TransformException:
            self.guard.fault='Localization transform unavailable';self.zero();return
        self.last_observed_pose=pose
        rotating=self.latest_command is not None and abs(self.latest_command[2])>.001
        reason=self.guard.reason(now,pose,age,rotating)
        if reason:self.zero();return
        if self.command_at is None or now-self.command_at>.3:
            self.zero()
            if now-self.guard.start_at>2.:self.guard.fault='Nav2 velocity stream timed out'
            return
        if self.last_motion_progress is None:
            self.last_motion_progress=pose;self.progress_at=now
        previous=self.last_motion_progress
        yaw_change=abs(math.atan2(math.sin(pose[2]-previous[2]),math.cos(pose[2]-previous[2])))
        if math.hypot(pose[0]-previous[0],pose[1]-previous[1])>.005 or yaw_change>math.radians(1):
            self.last_motion_progress=pose;self.progress_at=now
        elif now-self.progress_at>5:
            self.guard.fault='No localized motion progress';self.zero();return
        try:converted=self.conversion.convert(*self.latest_command)
        except ValueError as exc:self.guard.fault=str(exc);self.zero();return
        msg=Twist();msg.linear.x,msg.linear.y,msg.angular.z=converted
        self.record_command(converted)
        self.pub.publish(msg);self.commands_sent+=1

    def wait(self, future, seconds):
        deadline=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)
            if self.guard.fault:raise RuntimeError(self.guard.fault)
            if future.done():return future.result()
        raise RuntimeError('Nav2 request timed out')

    def isolate_teleop(self):
        self.lease=subprocess.Popen(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5',
            os.environ.get('JETSON_SSH_TARGET', 'ubuntu@192.0.2.10'),'bash /home/ubuntu/robomaster-s1-jetson/scripts/pause_teleop_for_trial.sh'],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
        selector=selectors.DefaultSelector();selector.register(self.lease.stdout,selectors.EVENT_READ)
        try:
            if not selector.select(8.) or self.lease.stdout.readline().strip()!='PAUSED':
                raise RuntimeError('Could not isolate gamepad teleop')
        finally:selector.close()
        self.lease.stdin.write('keep\n');self.lease.stdin.flush();self.last_lease=time.monotonic()
        until=time.monotonic()+.5
        while time.monotonic()<until:rclpy.spin_once(self,timeout_sec=.05)

    def run(self, dry_run, rviz_goal=False):
        if rviz_goal:
            print('Ready: use 2D Goal Pose in hardware_navigation.rviz. Select a point '
                  '10–35 cm ahead; keep its arrow facing forward. L1 stops motion.',flush=True)
            while rclpy.ok() and self.selected_goal is None:
                rclpy.spin_once(self,timeout_sec=.1)
        if not self.planner.wait_for_server(timeout_sec=8.):raise RuntimeError('Nav2 planner unavailable')
        deadline=time.monotonic()+10.;reason='Waiting for fresh sensors'
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)
            try:
                pose,age,q=self.pose();reason=self.guard.reason(time.monotonic(),pose,age)
                if not reason:break
            except TransformException:reason='Localization transform unavailable'
        else:raise RuntimeError(reason)
        self.guard.start_pose=pose
        goal=ComputePathToPose.Goal();goal.goal.header.frame_id='localization_map'
        goal.goal.header.stamp=self.get_clock().now().to_msg()
        if rviz_goal:
            selected=self.selected_goal;p=selected.pose.position;s=selected.pose.orientation
            if not .8<=sum(v*v for v in (s.x,s.y,s.z,s.w))<=1.2:
                raise RuntimeError('Invalid goal heading')
            yaw=math.atan2(2*(s.w*s.z+s.x*s.y),1-2*(s.y*s.y+s.z*s.z))
            validate_selected_goal(pose,(p.x,p.y,yaw),selected.header.frame_id)
            goal.goal.pose=selected.pose
        else:
            goal.goal.pose.position.x=pose[0]+.3*math.cos(pose[2])
            goal.goal.pose.position.y=pose[1]+.3*math.sin(pose[2]);goal.goal.pose.orientation=q
        goal.planner_id='GridBased';goal.use_start=False
        self.requested_target=goal.goal.pose
        request=self.wait(self.planner.send_goal_async(goal),8.)
        if not request.accepted:raise RuntimeError('Planner rejected the target')
        result=self.wait(request.get_result_async(),10.).result
        if result.error_code or len(result.path.poses)<2:raise RuntimeError('No usable path to target')
        points=result.path.poses
        length=sum(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y)
                   for a,b in zip(points,points[1:]))
        if length>.45:raise RuntimeError('Path requires a detour; first trial must be short')
        print(f'Path checked: {length:.3f}m, {len(points)} poses. Target '
              f'({goal.goal.pose.position.x:.3f}, {goal.goal.pose.position.y:.3f}).',flush=True)
        if dry_run:
            print('DRY RUN passed. No motion goal or motor commands sent.',flush=True);return
        if not self.controller.wait_for_server(timeout_sec=5.):raise RuntimeError('Nav2 controller unavailable')
        self.isolate_teleop()
        pose,age,_=self.pose()
        reason=self.guard.reason(time.monotonic(),pose,age)
        if reason:raise RuntimeError(reason)
        self.latest_command=None;self.command_at=None
        self.guard.start_at=time.monotonic();self.guard.start_pose=pose;self.armed=True
        follow=FollowPath.Goal();follow.path=result.path;follow.controller_id='FollowPath'
        follow.goal_checker_id='goal_checker' if self.require_heading else 'position_goal_checker'
        follow.progress_checker_id='progress_checker'
        self.goal=self.wait(self.controller.send_goal_async(follow),5.)
        if not self.goal.accepted:raise RuntimeError('Controller rejected the trial')
        self.outcome='running'
        print('Arrival mode: '+('position and heading' if self.require_heading else
              'position only, within 5 cm; final heading is not corrected'),flush=True)
        if self.conversion.minimum_forward_command:
            print(f'Minimum moving forward command: {self.conversion.minimum_forward_command:g} abstract; '
                  'zero requests still stop immediately.',flush=True)
        print('Trial started. Press L1 to stop. Forward command limited to 0.08m/s; '
              'continuous turning limit 0.15rad/s. Command pulses disabled.',flush=True)
        outcome=self.wait(self.goal.get_result_async(),20.)
        self.armed=False;self.zero()
        if outcome.status!=GoalStatus.STATUS_SUCCEEDED or outcome.result.error_code:
            raise RuntimeError('Nav2 did not reach the target: '+str(outcome.result.error_code))
        end,_,_=self.pose()
        self.outcome='succeeded'
        print(f'Trial complete; localized travel '
              f'{math.hypot(end[0]-pose[0],end[1]-pose[1]):.3f}m.',flush=True)

    def finish(self):
        self.armed=False
        try:
            if self.goal is not None and self.goal.accepted and rclpy.ok():
                future=self.goal.cancel_goal_async()
                rclpy.spin_until_future_complete(self,future,timeout_sec=1.)
            for _ in range(3):self.zero();time.sleep(.05)
        finally:
            if self.lease is not None:
                if self.lease.stdin:self.lease.stdin.close()
                try:self.lease.wait(timeout=4.)
                except subprocess.TimeoutExpired:self.lease.terminate()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--rviz-goal',action='store_true',help='Wait for one nearby forward goal selected in RViz')
    parser.add_argument('--position-only',action='store_true',
                        help='Arrive within 5 cm without requiring the final heading')
    args=parser.parse_args()
    rclpy.init();node=Trial(args.dry_run,require_heading=not args.position_only)
    try:node.run(args.dry_run,args.rviz_goal)
    except (RuntimeError,KeyboardInterrupt) as exc:
        node.outcome='stopped: '+str(exc)
        node.armed=False;node.zero()
        print('Trial stopped: '+str(exc),flush=True)
        try:
            pose,age,_=node.pose()
            if node.requested_target is not None and -.6<=age<=1.:
                target=node.requested_target; q=target.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                heading=math.degrees(math.atan2(math.sin(yaw-pose[2]),math.cos(yaw-pose[2])))
                print(f'Remaining target error: '
                      f'{math.hypot(pose[0]-target.position.x,pose[1]-target.position.y):.3f} m, '
                      f'heading {heading:+.1f} degrees.',flush=True)
        except TransformException:
            pass
        raise SystemExit(1)
    finally:
        try:
            node.finish()
            try:node.save_trace()
            except OSError as exc:print('Could not save trace: '+str(exc),flush=True)
        finally:
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
