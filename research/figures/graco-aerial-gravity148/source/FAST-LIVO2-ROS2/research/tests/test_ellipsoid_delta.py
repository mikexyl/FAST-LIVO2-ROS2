from pathlib import Path
import sys
from types import SimpleNamespace as NS
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.ellipsoid_delta_writer import DeltaWriter,DeltaReader


def test_lossless_map_deltas_updates_removals_reentry_and_empty(tmp_path):
    rng=np.random.default_rng(32);all_values=rng.integers(0,2**32-1,(8,17),dtype=np.uint32)
    all_values[:,15]=np.arange(8);writer=DeltaWriter(tmp_path);reader=DeltaReader()
    for turn,ids in enumerate(([0,1,2],[0,2,3,4],[2,3],[0,2,3,4,5],[],[7])):
        values=all_values[ids].copy()
        if turn==2:values[0,0]^=1
        row=dict(world_frame='Alpha/odom_ellipselio')
        msg=NS(header=NS(frame_id=row['world_frame']),is_bigendian=False,height=1,point_step=68,
               row_step=len(values)*68,width=len(values),data=values.tobytes())
        assert not writer(row,dict(topic='/Alpha/research/ellipsoids'),msg)
        reader.apply(tmp_path/row['ellipsoid_delta']['file'],len(values))
        assert np.array_equal(reader.values,values)
    with pytest.raises(ValueError,match='world-frame'):
        msg.header.frame_id='Alpha/imu';writer(row,dict(topic='/Alpha/research/ellipsoids'),msg)
