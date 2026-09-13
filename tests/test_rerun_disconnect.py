"""A closed viewer must not prevent finalization of the independent recording."""
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from mcap.reader import make_reader

spec = importlib.util.spec_from_file_location('s3e_rerun', Path(__file__).parents[1] / 'scripts/s3e_rerun.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class DisconnectTest(unittest.TestCase):
    def test_mcap_finishes_when_grpc_viewer_is_closed(self):
        def disconnected():
            raise RuntimeError('gRPC connection severed: viewer closed')

        with TemporaryDirectory() as directory:
            stream = module.McapStream(Path(directory))
            with patch.object(module.rr, 'get_data_recording', return_value=SimpleNamespace(flush=disconnected)):
                stream.close()
            with (Path(directory) / 'visualization.mcap').open('rb') as file:
                self.assertEqual(make_reader(file).get_header().profile, 'ros2')
            self.assertTrue(stream.file.closed)
            self.assertFalse(stream.worker.is_alive())
            self.assertIsNone(stream.error)
            self.assertIn('viewer closed', stream.viewer_disconnect)


if __name__ == '__main__':
    unittest.main()
