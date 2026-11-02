import tempfile
import unittest
from pathlib import Path

try:
    import yaml
    from s1_can.home_pose import home_parameters, map_signature
except ImportError:
    yaml = None


@unittest.skipIf(yaml is None, 'PyYAML required')
class HomePoseTests(unittest.TestCase):
    def test_map_identity_and_pose_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = root / 'map.pgm'; image.write_bytes(b'P5\n1 1\n255\n\xff')
            grid = root / 'map.yaml'; grid.write_text('image: map.pgm\nresolution: 0.03\n')
            home = root / 'home.yaml'
            self.assertEqual(home_parameters(home, grid), {})
            data = {'frame_id': 'localization_map', 'map_signature': map_signature(grid),
                    'pose': {'x': 1., 'y': 2., 'yaw': .5}}
            home.write_text(yaml.safe_dump(data))
            self.assertEqual(home_parameters(home, grid)['initial_pose.x'], 1.)
            self.assertTrue(home_parameters(home, grid)['set_initial_pose'])
            data['pose']['yaw'] = float('nan')
            home.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError):
                home_parameters(home, grid)
            data['pose']['yaw'] = .5
            home.write_text(yaml.safe_dump(data))
            image.write_bytes(b'changed map')
            with self.assertRaises(ValueError):
                home_parameters(home, grid)
