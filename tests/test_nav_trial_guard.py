import math
import unittest
from s1_can.nav_trial_guard import TrialConversion, TrialGuard, validate_selected_goal


class ConversionTests(unittest.TestCase):
    def test_position_only_minimum_command_preserves_stop_and_speed_bound(self):
        c=TrialConversion(.16,.72,minimum_forward_command=.45)
        self.assertEqual(c.convert(.029,0.,0.),(.45,0.,0.))
        self.assertEqual(c.convert(0.,0.,0.),(0.,0.,0.))
        self.assertEqual(c.convert(.0005,0.,0.),(0.,0.,0.))
        self.assertEqual(c.convert(.2,0.,0.),(.5,0.,0.))
        for floor in (-1.,.6,math.nan):
            with self.assertRaises(ValueError):TrialConversion(.16,.72,floor)

    def test_selected_goal_bounds_and_rotated_robot_frame(self):
        validate_selected_goal((0.,0.,math.pi/2),(0.,.3,math.pi/2),'localization_map')
        for goal,frame in (((0.,-.3,0.),'localization_map'),
                           ((.3,.1,0.),'localization_map'),
                           ((.4,0.,0.),'localization_map'),
                           ((.3,0.,math.pi/2),'localization_map'),
                           ((.3,0.,0.),'wrong_map')):
            with self.assertRaises(RuntimeError):
                validate_selected_goal((0.,0.,0.),goal,frame)
    def test_convert_and_bound_speed_without_changing_driver_scales(self):
        c=TrialConversion(.16,.72)
        self.assertEqual(c.convert(.08,0.,0.),(.5,0.,0.))
        self.assertEqual(c.convert(.2,0.,0.)[0],.5)
        self.assertAlmostEqual(c.convert(0.,0.,.15)[2],.15/.72)
        for cmd in ((-.1,0.,0.),(0.,.1,0.),(math.nan,0.,0.)):
            with self.assertRaises(ValueError):c.convert(*cmd)

    def ready(self):
        g=TrialGuard();g.scan_at=g.odom_at=g.joy_at=10.
        g.front_clear=g.turn_clear=True;g.l1=False
        return g

    def test_stale_odometry_latches_stop_and_does_not_resume(self):
        g=self.ready();g.start_at=10.;g.start_pose=(0.,0.,0.)
        self.assertIsNone(g.reason(10.1,(0.,0.,0.),.1))
        g.scan_at=g.joy_at=10.8
        self.assertEqual(g.reason(10.8,(0.,0.,0.),.1), 'odometry is unavailable or stale')
        g.scan_at=g.odom_at=g.joy_at=10.8
        self.assertIsNotNone(g.reason(10.8,(0.,0.,0.),.1))

    def test_operator_stop_obstacle_and_distance_limit(self):
        for trigger in ('l1','front_clear','distance','heading'):
            g=self.ready();g.start_at=10.;g.start_pose=(0.,0.,0.);pose=(0.,0.,0.)
            if trigger=='l1':g.l1=True
            elif trigger=='front_clear':g.front_clear=False
            elif trigger=='distance':pose=(.46,0.,0.)
            else:pose=(0.,0.,math.radians(16))
            self.assertIsNotNone(g.reason(10.1,pose,.1))
