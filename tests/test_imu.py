import math
import struct
import unittest
from unittest.mock import patch, MagicMock
from s1_can.imu import ImuDecoder, ImuReceiver
from s1_can.protocol import PacketEncoder
from s1_can.chassis import S1Chassis


class ImuTests(unittest.TestCase):
    def test_receiver_selects_standard_data_not_error_frames(self):
        sock = MagicMock()
        with patch('s1_can.imu.socket.socket', return_value=sock):
            receiver = ImuReceiver('can1')
        args = sock.setsockopt.call_args.args
        can_id, mask = struct.unpack('=II', args[2])
        self.assertEqual(can_id, 0x202)
        self.assertEqual(mask & 0x20000000, 0)  # CAN_ERR_FLAG must not be set
        self.assertEqual(mask & 0xC00007FF, 0xC00007FF)
        receiver.close()

    def packet(self, gyro=-1.):
        payload = bytes((0,7)) + struct.pack('<II6f',100,200,0,0,-1,0,0,gyro)
        return PacketEncoder().encode(3,9,0x48,8,payload)

    def test_fragmentation(self):
        p=self.packet(); d=ImuDecoder(); results=[]
        for i in range(0,len(p),8): results += d.feed(p[i:i+8])
        self.assertEqual(len(results),1)
        self.assertEqual(results[0][0],(100,200))
        self.assertEqual(results[0][1][-1],-1.)

    def test_crc_resync_and_nan(self):
        bad=bytearray(self.packet());bad[-1]^=1
        d=ImuDecoder()
        result=d.feed(b'garbage'+bad+self.packet(float('nan'))+self.packet())
        self.assertEqual(len(result),1)

    def test_subscription_uses_chassis_transport(self):
        class Transport:
            packets=[]
            def send(self,p): self.packets.append(p)
            def reconnect(self): return True
            def close(self): pass
        t=Transport(); robot=S1Chassis(transport=t,enable_imu=True)
        robot.start();robot.stop()
        subscriptions=[p for p in t.packets if p[9:11]==bytes((0x48,3))]
        self.assertEqual(len(subscriptions),1)
        self.assertEqual(subscriptions[0][11:-2],bytes.fromhex('09 07 03 00 01 f4 1d 1c dc 03 00 02 00 0a 00'))
        self.assertEqual(t.packets[-1][9:11],bytes((0x48,4)))


if __name__=='__main__': unittest.main()
