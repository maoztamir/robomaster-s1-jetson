import importlib.util
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('patrol', Path(__file__).resolve().parents[1] / 's1_can/patrol.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
Patrol = module.Patrol


class PatrolTests(unittest.TestCase):
    def command(self, policy, scan, now=1):
        return policy.command(scan, -math.pi, math.tau / 360, .15, 12., now)

    def test_open_space_moves(self):
        self.assertGreater(self.command(Patrol(), [math.inf] * 360)[0], 0)

    def test_invalid_scan_stops(self):
        for scan in ([], [math.nan] * 360, [0.] * 360, [math.inf] * 180):
            self.assertEqual(self.command(Patrol(), scan)[:2], (0., 0.))

    def test_front_obstacle_turns_without_forward_motion(self):
        scan = [2.] * 360
        scan[180] = .6
        x, z, _ = self.command(Patrol(), scan)
        self.assertEqual(x, 0.)
        self.assertNotEqual(z, 0.)

    def test_distant_returns_do_not_block_coverage(self):
        scan = [2.] * 360
        scan[10:20] = [12.45] * 10
        self.assertGreater(self.command(Patrol(), scan)[0], 0.)
        scan[180] = .6
        x, z, _ = self.command(Patrol(), scan)
        self.assertEqual(x, 0.)
        self.assertNotEqual(z, 0.)

    def test_invalid_sector_still_stops(self):
        for value in (math.nan, -math.inf, 0., .1):
            scan = [2.] * 360
            scan[10:20] = [value] * 10
            self.assertEqual(self.command(Patrol(), scan)[:2], (0., 0.))

    def test_sensor_range_must_cover_clearance(self):
        result = Patrol().command([math.inf] * 360, -math.pi,
                                  math.tau / 360, .15, .5, 1.)
        self.assertEqual(result[:2], (0., 0.))

    def test_close_obstacle_anywhere_prevents_turning(self):
        for i in (0, 90, 180, 270):
            scan = [2.] * 360
            scan[i] = .3
            self.assertEqual(self.command(Patrol(), scan)[:2], (0., 0.))

    def test_seeded_room_patrol(self):
        geometry_spec = importlib.util.spec_from_file_location('planar', Path(__file__).resolve().parents[1] / 'simulation/planar.py')
        geometry = importlib.util.module_from_spec(geometry_spec)
        geometry_spec.loader.exec_module(geometry)
        boxes = [(0, -3, 6, .1), (0, 3, 6, .1), (-3, 0, .1, 6), (3, 0, .1, 6), (0, 0, .6, 2)]
        for seed in range(5):
            policy = Patrol(seed=seed)
            pose, distance = (-2., 0., 0.), 0.
            for tick in range(2400):
                scan = [geometry.ray_distance(pose[:2], pose[2] - math.pi + i * math.tau / 360, boxes, 12.) for i in range(360)]
                x, z, _ = self.command(policy, scan, tick * .05 + 1)
                pose = geometry.integrate(pose, (x, 0., z), .05)
                distance += x * .05
                self.assertFalse(any(geometry.overlaps(pose, (.32, .24), box) for box in boxes))
            self.assertGreater(distance, 4.)


if __name__ == '__main__':
    unittest.main()
