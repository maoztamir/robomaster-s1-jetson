import math
import unittest
from s1_can.motion_command_shaper import CommandPulse, MotionCommandShaper


class MotionShaperTests(unittest.TestCase):
    def test_heading_corrections_do_not_starve_small_forward_requests(self):
        shaper=MotionCommandShaper(.16,.72,turn_pulses=True)
        values=[shaper.output(i*.05,.003,0.,.15) for i in range(400)]
        self.assertGreater(sum(v[0] for v in values)*.16*.05,.08)
        self.assertTrue(any(v[2] for v in values))
        self.assertTrue(all(not (v[0] and v[2]) for v in values))
        self.assertLessEqual(max(v[0] for v in values)*.16,.08)

    def test_zero_forward_request_interrupts_a_pulse_immediately(self):
        shaper=MotionCommandShaper(.16,.72)
        for i in range(200):
            if shaper.output(i*.05,.02,0.,0.)[0]: break
        else:self.fail('No forward correction was emitted')
        self.assertEqual(shaper.output(i*.05+.01,0.,0.,0.),(0.,0.,0.))

    def test_default_turning_retains_continuous_limit_without_stronger_pulses(self):
        shaper=MotionCommandShaper(.16,.72)
        for i in range(100):
            self.assertAlmostEqual(shaper.output(i*.05,0.,0.,.5)[2],.15/.72)
        self.assertEqual(shaper.output(5.,0.,0.,0.),(0.,0.,0.))

    def test_angular_pulses_have_zero_gaps_and_bounded_average(self):
        pulse=CommandPulse(.7165776073,.65,.15,.35)
        values=[pulse.output(i*.05,.15) for i in range(400)]
        self.assertLessEqual(sum(values)*.7165776073*.05,20*.15+.05)
        lengths=[];run=0
        for value in values+[0.]:
            if value:run+=1
            elif run:lengths.append(run);run=0
        self.assertTrue(lengths)
        self.assertLessEqual(max(lengths),3)  # 20Hz sampling / float boundary.

    def test_zero_or_direction_change_stops_an_active_pulse(self):
        for request in (0.,-.15):
            pulse=CommandPulse(.72,.65,.15,.35)
            for i in range(20):
                if pulse.output(i*.05,.15):break
            self.assertEqual(pulse.output(i*.05+.01,request),0.)

    def test_long_pause_does_not_build_a_burst_and_invalid_clock_fails(self):
        pulse=CommandPulse(.72,.65,.15,.35)
        pulse.output(0.,.15)
        self.assertEqual(pulse.output(100.,.15),0.)
        with self.assertRaises(ValueError):pulse.output(99.,.15)
        with self.assertRaises(ValueError):pulse.output(101.,math.nan)

    def test_motion_limits_and_heading_pulses_are_stationary(self):
        shaper=MotionCommandShaper(.16,.72,turn_pulses=True)
        self.assertEqual(shaper.output(0.,.5,0.,0.),(.5,0.,0.))
        for bad in ((-.1,0.,0.),(0.,.1,0.),(math.nan,0.,0.)):
            with self.assertRaises(ValueError):shaper.output(0.,*bad)
        values=[shaper.output(i*.05,.08,0.,.15) for i in range(50)]
        self.assertTrue(any(v[2] for v in values))
        self.assertTrue(all(v[0]==0. for v in values if v[2]))
        reverse=MotionCommandShaper(.16,.72,allow_reverse=True)
        self.assertEqual(reverse.output(0.,-.08,0.,0.),(-.5,0.,0.))
