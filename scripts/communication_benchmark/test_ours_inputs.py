import json
from pathlib import Path
import tempfile
import threading
import unittest

import numpy as np
from ours import retained_odometry
from s3e_pipeline.online_io import atomic_json


class RetainedOdometryTests(unittest.TestCase):
    def test_native_updates_without_backend_report(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); trial=root/'trial'; (trial/'frontend').mkdir(parents=True)
            (root/'trials.json').write_text(json.dumps({'Alpha':str(trial)}))
            native=trial/'frontend/native_updates.jsonl'
            rows=[dict(stamp_ns=(i+1)*10**9,sensor_stamp_ns=i*10**9,
                       pose=[i,0,0,0,0,0,1]) for i in range(3)]
            native.write_text(''.join(json.dumps(row)+'\n' for row in rows))
            raw,path=retained_odometry(root,'Alpha')
            np.testing.assert_array_equal(raw[:,0],[1.,2.,3.])
            np.testing.assert_array_equal(raw[:,1],[0.,1.,2.])
            self.assertEqual(path,native)
            for bad_rows in [[],[rows[0],rows[0]],
                             [rows[0],dict(rows[1],pose=[float('nan'),0,0,0,0,0,1])]]:
                native.write_text(''.join(json.dumps(row)+'\n' for row in bad_rows))
                with self.assertRaises(ValueError):retained_odometry(root,'Alpha')

    def test_existing_report_keeps_frozen_input(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); (root/'report').mkdir()
            expected=np.array([[1,0,0,0,0,0,0,1],[2,1,0,0,0,0,0,1]])
            path=root/'report/Alpha-raw.tum'; np.savetxt(path,expected)
            raw,actual=retained_odometry(root,'Alpha')
            np.testing.assert_array_equal(raw,expected)
            self.assertEqual(actual,path)


class ConcurrentProgressTests(unittest.TestCase):
    def test_reader_never_observes_partial_progress(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'progress.json'; errors=[]; done=threading.Event()
            atomic_json(path,dict(revision=-1,values=[-1]*5000))
            def publish():
                try:
                    for i in range(100):atomic_json(path,dict(revision=i,values=[i]*5000))
                except Exception as error:errors.append(error)
                finally:done.set()
            writer=threading.Thread(target=publish);writer.start();reads=0
            try:
                while not done.is_set():
                    value=json.loads(path.read_text());reads+=1
                    self.assertEqual(value['values'],[value['revision']]*5000)
            finally:writer.join()
            self.assertFalse(errors);self.assertGreater(reads,0)
            self.assertEqual(json.loads(path.read_text())['revision'],99)


if __name__=='__main__':unittest.main()
