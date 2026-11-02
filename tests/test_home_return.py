import math
import unittest
from s1_can.home_return import return_velocity, validate_return


class HomeReturnTests(unittest.TestCase):
    def test_only_nearby_base_behind_robot_is_allowed(self):
        validate_return((0.,0.,0.),(-.25,0.,0.))
        for home in ((.25,0.,0.),(-.5,0.,0.),(-.25,.1,0.),(-.25,0.,.3),(math.nan,0.,0.)):
            with self.assertRaises(ValueError): validate_return((0.,0.,0.),home)

    def test_reverse_speed_is_bounded_and_turn_is_stationary(self):
        self.assertEqual(return_velocity((0.,0.,0.),(-.25,0.,0.)),(-.08,0.,False))
        vx,wz,done=return_velocity((0.,0.,-.1),(-.25,0.,0.))
        self.assertEqual(vx,0.); self.assertGreater(wz,0.); self.assertLessEqual(wz,.25)
        self.assertFalse(done)

    def test_arrival_requires_position_and_heading(self):
        self.assertEqual(return_velocity((-.24,0.,0.),(-.25,0.,0.)),(0.,0.,True))
        vx,wz,done=return_velocity((-.24,0.,-.1),(-.25,0.,0.))
        self.assertEqual(vx,0.); self.assertGreater(wz,0.); self.assertFalse(done)

    def test_rotated_reverse_path(self):
        start=(0.,0.,-math.pi/2); home=(0.,.25,-math.pi/2)
        validate_return(start,home)
        self.assertEqual(return_velocity(start,home),(-.08,0.,False))
