import unittest
from s1_can.sensor_idle import SensorIdlePolicy, SensorMotionGate


class IdleTests(unittest.TestCase):
    def test_idle_boundary_and_zero_commands(self):
        p = SensorIdlePolicy(0.)
        p.command((0., 0., 0.), 290.)
        self.assertFalse(p.should_sleep(299.))
        self.assertTrue(p.should_sleep(300.))

    def test_motion_request_resets_timer_even_when_motor_is_gated(self):
        p = SensorIdlePolicy(0.)
        self.assertTrue(p.should_sleep(301.))
        p.command((1., 0., 0.), 302.)
        self.assertFalse(p.should_sleep(302.))
        self.assertTrue(p.should_sleep(602.))

    def test_active_goal_and_stereo_hold_prevent_sleep(self):
        p = SensorIdlePolicy(0.)
        p.active_goals.add('goal')
        self.assertFalse(p.should_sleep(1000.))
        p.active_goals.clear()
        p.hold = True
        self.assertFalse(p.should_sleep(1000.))

    def test_guard_denies_on_start_sleep_and_dead_manager(self):
        gate = SensorMotionGate(enabled=True)
        self.assertFalse(gate.permits(1.))
        gate.update(True, 2.)
        self.assertTrue(gate.permits(2.3))
        self.assertFalse(gate.permits(2.7))
        gate.update(True, 3.)
        gate.update(False, 3.1)
        self.assertFalse(gate.permits(3.1))

    def test_disabled_feature_preserves_existing_driver(self):
        self.assertTrue(SensorMotionGate().permits(1000.))


if __name__ == '__main__':
    unittest.main()
