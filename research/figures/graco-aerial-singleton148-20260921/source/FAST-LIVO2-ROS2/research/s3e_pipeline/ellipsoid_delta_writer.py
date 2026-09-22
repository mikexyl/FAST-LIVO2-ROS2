#!/usr/bin/env python3
"""Lossless persistent-map deltas beside standard raw-cloud/state MCAP exports."""
import argparse
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.mcap_writer import consume


class DeltaWriter:
    def __init__(self,output):
        self.output=Path(output);(self.output/'ellipsoids').mkdir(parents=True,exist_ok=True)
        self.files=[];self.ids=np.empty(0,dtype=np.uint32);self.values=np.empty((0,17),dtype=np.uint32)

    def __call__(self,row,entry,msg):
        if not entry['topic'].endswith('/ellipsoids'):return True
        if msg.header.frame_id!=row['world_frame']:raise ValueError('Lossless delta export requires native world-frame ellipsoids')
        if msg.is_bigendian or msg.height!=1 or msg.point_step!=68 or msg.row_step!=68*msg.width:
            raise ValueError('Unexpected native ellipsoid layout')
        # Compare all float geometry bitwise; retain IDs/types as uint32 words.
        values=np.frombuffer(msg.data,dtype='<u4').reshape(-1,17).copy();ids=values[:,15]
        if len(ids)>1 and np.any(ids[1:]<=ids[:-1]):raise ValueError('Map IDs must be strictly increasing')
        where=np.searchsorted(self.ids,ids);existing=where<len(self.ids)
        matched=np.flatnonzero(existing)
        existing[matched]=self.ids[where[matched]]==ids[matched]
        changed=~existing
        matched=np.flatnonzero(existing)
        changed[matched]=np.any(self.values[where[matched]]!=values[matched],axis=1)
        removed=np.setdiff1d(self.ids,ids,assume_unique=True)
        name=f'ellipsoids/{len(self.files):06d}.npz'
        np.savez_compressed(self.output/name,changed=values[changed],removed=removed)
        row['ellipsoid_delta']=dict(schema_version=1,file=name,changed=int(changed.sum()),removed=len(removed),
            current_count=len(ids),frame=msg.header.frame_id,encoding='lossless 17 little-endian uint32 words per native point, float geometry bit patterns preserved')
        self.files.append(name);self.ids=ids;self.values=values
        return False


class DeltaReader:
    def __init__(self):self.ids=np.empty(0,dtype=np.uint32);self.values=np.empty((0,17),dtype=np.uint32)
    def apply(self,path,expected_count):
        with np.load(path,allow_pickle=False) as f:changed=f['changed'];removed=f['removed']
        if changed.ndim!=2 or changed.shape[1]!=17 or changed.dtype!=np.uint32:raise ValueError('Invalid ellipsoid delta')
        old_keep=~np.isin(self.ids,np.concatenate([removed,changed[:,15]]))
        values=np.concatenate([self.values[old_keep],changed]);order=np.argsort(values[:,15])
        self.values=values[order];self.ids=self.values[:,15]
        if len(self.ids)!=expected_count or len(np.unique(self.ids))!=len(self.ids):raise ValueError('Ellipsoid delta chain mismatch')
        geometry=np.ascontiguousarray(self.values[:,:15]).view('<f4').astype(np.float64)
        return dict(centers=geometry[:,:3],axes=geometry[:,3:6],basis=geometry[:,6:].reshape(-1,3,3),
                    map_id=self.ids.copy(),primitive=self.values[:,16].copy())


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    consume(sys.stdin.buffer,args.output,DeltaWriter(args.output))
