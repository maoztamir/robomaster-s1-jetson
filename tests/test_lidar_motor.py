import struct
import termios
import unittest
from unittest.mock import call, patch
from s1_can.lidar_motor import LidarMotorHold


class MotorHoldTests(unittest.TestCase):
    def test_hold_keeps_port_open_until_wake(self):
        with patch('s1_can.lidar_motor.os.open', return_value=7), \
                patch('s1_can.lidar_motor.os.close') as close, \
                patch('s1_can.lidar_motor.fcntl.ioctl') as ioctl:
            motor = LidarMotorHold('/dev/fake-lidar')
            motor.stop()
            close.assert_not_called()
            self.assertEqual(ioctl.call_args_list, [call(7, termios.TIOCEXCL),
                call(7, termios.TIOCMBIS, struct.pack('i', termios.TIOCM_DTR))])
            motor.stop()  # No second owner or second open.
            self.assertEqual(ioctl.call_count, 2)
            motor.release()
            ioctl.assert_called_with(7, termios.TIOCNXCL)
            close.assert_called_once_with(7)
            motor.release()
            self.assertEqual(close.call_count, 1)

    def test_control_failure_closes_port_and_does_not_mark_as_held(self):
        with patch('s1_can.lidar_motor.os.open', return_value=7), \
                patch('s1_can.lidar_motor.os.close') as close, \
                patch('s1_can.lidar_motor.fcntl.ioctl', side_effect=[None, OSError('failed'), None]):
            motor = LidarMotorHold('/dev/fake-lidar')
            with self.assertRaises(OSError):
                motor.stop()
            self.assertIsNone(motor.fd)
            close.assert_called_once_with(7)
