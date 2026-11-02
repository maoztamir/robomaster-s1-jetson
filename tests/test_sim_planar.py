"""Geometric checks for the explicitly ideal navigation prototype."""
import math
import unittest
from simulation.planar import integrate, overlaps, ray_distance, wheel_rates

class PlanarTests(unittest.TestCase):
    def test_lateral_motion_rotates_with_body(self):
        x,y,_=integrate((0,0,math.pi/2),(0,1,0),1)
        self.assertAlmostEqual(x,-1)
        self.assertAlmostEqual(y,0)
    def test_circular_motion(self):
        x,y,yaw=integrate((0,0,0),(1,0,1),math.pi/2)
        self.assertAlmostEqual(x,1)
        self.assertAlmostEqual(y,1)
        self.assertAlmostEqual(yaw,math.pi/2)
    def test_collision_rotated_footprint(self):
        self.assertFalse(overlaps((0,0,0),(.4,.1),(.0,.16,.1,.1)))
        self.assertTrue(overlaps((0,0,math.pi/2),(.4,.1),(.0,.16,.1,.1)))
    def test_ray_nearest_and_parallel(self):
        self.assertAlmostEqual(ray_distance((0,0),0,[(2,0,1,1),(4,0,1,1)],12),1.5)
        self.assertEqual(ray_distance((0,0),math.pi/2,[(2,0,1,1)],12),12)
    def test_wheel_patterns(self):
        self.assertEqual(wheel_rates((.1,0,0)),(2,2,2,2))
        self.assertEqual(wheel_rates((0,.1,0)),(2,-2,2,-2))
        self.assertEqual(wheel_rates((0,0,.5)),(2,-2,-2,2))
if __name__ == '__main__':
    unittest.main()
