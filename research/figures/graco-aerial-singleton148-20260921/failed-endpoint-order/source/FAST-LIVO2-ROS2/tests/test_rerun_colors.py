"""Check the PCL packed-color adaptation not provided by the native importer."""
import importlib.util
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location('s3e_rerun', Path(__file__).parents[1] / 'scripts/s3e_rerun.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ColorsTest(unittest.TestCase):
    def test_packed_rgb_with_padding_and_byte_order(self):
        for big in (False, True):
            data = bytearray(56)
            for offset, color in [(4, 0xFF0000), (16, 0x00FF00), (32, 0x0000FF), (44, 0x123456)]:
                struct.pack_into('>I' if big else '<I', data, offset, color)
            message = SimpleNamespace(height=2, width=2, row_step=28, point_step=12,
                                      is_bigendian=big, data=data,
                                      fields=[SimpleNamespace(name='rgb', offset=4)])
            self.assertEqual(module.pcl_colors(message).tolist(),
                             [0xFF0000FF, 0x00FF00FF, 0x0000FFFF, 0x123456FF])

    def test_no_color_field(self):
        self.assertIsNone(module.pcl_colors(SimpleNamespace(fields=[])))


if __name__ == '__main__':
    unittest.main()
