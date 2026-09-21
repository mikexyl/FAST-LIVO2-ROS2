"""Regression checks for acquisition timestamps and organized cloud layouts."""
from array import array
from pathlib import Path
import struct
import sys
import unittest

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from s3e_adapter import rebase_scan


class ScanTimingTest(unittest.TestCase):
    def make_cloud(self, big_endian=False):
        m = PointCloud2()
        m.header.stamp.sec, m.header.stamp.nanosec = 100, 20_000_000
        m.height, m.width, m.point_step, m.row_step = 2, 2, 22, 50
        m.is_bigendian = big_endian
        m.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n,o,t in [
            ('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('time',18,7)]]
        data = bytearray(100)
        for i,t in enumerate([0.02,-0.10,0.0,-0.05]):
            struct.pack_into(('>' if big_endian else '<')+'ffffHf', data,
                             i//2*50+i%2*22, float(i),2.,3.,10.+i,i,t)
        m.data = array('B', data)
        return m

    def check_rebase(self, big_endian):
        original = self.make_cloud(big_endian)
        out = rebase_scan(original)
        self.assertEqual((original.header.stamp.sec, original.header.stamp.nanosec), (100,20_000_000))
        self.assertEqual((out.height, out.width, out.row_step), (1,4,88))
        self.assertEqual(out.is_bigendian, big_endian)
        fmt = ('>' if big_endian else '<')+'ffffHf'
        a=[struct.unpack_from(fmt, original.data, i//2*50+i%2*22) for i in range(4)]
        b=[struct.unpack_from(fmt, out.data, i*22) for i in range(4)]
        self.assertEqual([p[4] for p in b], [1,3,2,0])
        self.assertTrue(np.all(np.diff([p[-1] for p in b])>=0))
        shift=(out.header.stamp.sec-100)+(out.header.stamp.nanosec-20_000_000)/1e9
        for point in b:
            source = a[point[4]]
            self.assertEqual(point[:-1], source[:-1])
            self.assertLess(abs(shift+point[-1]-source[-1]),1e-8)

    def test_padded_little_endian_cloud(self):
        self.check_rebase(False)

    def test_padded_big_endian_cloud(self):
        self.check_rebase(True)

    def test_missing_time_rejected(self):
        cloud=self.make_cloud();cloud.fields=cloud.fields[:-1]
        with self.assertRaisesRegex(ValueError,'float32 time'):
            rebase_scan(cloud)

    def test_wrong_time_units_rejected(self):
        cloud=self.make_cloud()
        struct.pack_into('<f',cloud.data,18,100000.0)
        with self.assertRaisesRegex(ValueError,'scan duration'):
            rebase_scan(cloud)


if __name__ == '__main__':
    unittest.main()
