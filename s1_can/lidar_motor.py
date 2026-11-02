"""Hold the A1 USB adapter's DTR motor-stop line while its driver is asleep."""
import fcntl
import os
import struct
import termios


class LidarMotorHold:
    def __init__(self, port):
        self.port = port
        self.fd = None

    def stop(self):
        if self.fd is not None:
            return
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            # The ROS driver must have stopped before opening this port.
            # Exclusive access prevents a second driver from changing DTR.
            fcntl.ioctl(fd, termios.TIOCEXCL)
            fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack('i', termios.TIOCM_DTR))
        except BaseException:
            try:
                fcntl.ioctl(fd, termios.TIOCNXCL)
            finally:
                os.close(fd)
            raise
        self.fd = fd

    def release(self):
        if self.fd is None:
            return
        fd, self.fd = self.fd, None
        try:
            fcntl.ioctl(fd, termios.TIOCNXCL)
        finally:
            os.close(fd)
