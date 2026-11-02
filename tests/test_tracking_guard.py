import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('guard', Path(__file__).resolve().parents[1] / 's1_can/tracking_guard.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TrackingTests(unittest.TestCase):
    def test_waits_for_tracking(self):
        g = module.TrackingGuard()
        self.assertFalse(g.permits(0))
        g.update(False, 0., 0.)
        self.assertFalse(g.permits(0))
        g.update(True, .1, 1.)
        self.assertTrue(g.permits(1.))

    def test_loss_latches(self):
        g = module.TrackingGuard()
        g.update(True, .1, 0.)
        g.update(False, .1, .1, explicitly_lost=True)
        self.assertIn('explicitly reported lost', g.reason)
        g.update(True, .1, .2)
        self.assertFalse(g.permits(.2))

    def test_single_low_inlier_frame_does_not_latch(self):
        g = module.TrackingGuard()
        g.update(True, .1, 0.)
        g.update(False, .1, .1)
        self.assertTrue(g.permits(.1))
        self.assertIn('too few inliers', g.reason)
        g.update(True, .1, .2)
        self.assertTrue(g.permits(.2))

    def test_sustained_low_inliers_latch(self):
        g = module.TrackingGuard()
        g.update(True, .1, 0.)
        g.update(False, .1, .1)
        g.update(False, .1, .51)
        self.assertFalse(g.permits(.51))
        self.assertIn('too few inliers', g.reason)

    def test_silence_latches(self):
        g = module.TrackingGuard()
        g.update(True, .1, 0.)
        self.assertFalse(g.permits(2.))
        self.assertIn('timed out', g.reason)
        g.update(True, .1, 2.1)
        self.assertFalse(g.permits(2.1))

    def test_late_callback_cannot_hide_gap(self):
        g = module.TrackingGuard()
        g.update(True, .1, 0.)
        g.update(True, .1, 2.)
        self.assertFalse(g.permits(2.))

    def test_stale_or_invalid_stamps_cannot_arm(self):
        for age in (2., -.5, float('nan')):
            g = module.TrackingGuard()
            g.update(True, age, 0.)
            self.assertFalse(g.permits(0.))

    def test_reason_clears_when_healthy(self):
        g = module.TrackingGuard()
        g.update(True, .1, 0.)
        self.assertIsNone(g.reason)


if __name__ == '__main__':
    unittest.main()
