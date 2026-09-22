import ctypes as ct
import json
from pathlib import Path
import sys
import numpy as np
import argparse
parser=argparse.ArgumentParser()
parser.add_argument('--reference',type=Path,required=True)
parser.add_argument('--library',type=Path,required=True)
parser.add_argument('--tiny-library',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from s3e_pipeline.ellipsoid_bev import unit_surface

rng=np.random.default_rng(17)
centers=rng.uniform(-12,12,(320,3));axes=rng.uniform(.1,1.,(320,3))
geometry=np.ascontiguousarray(np.column_stack([centers,axes,np.tile(np.eye(3).ravel(),(320,1))]))
sphere=np.ascontiguousarray(unit_surface(256))
ptr=lambda a:a.ctypes.data_as(ct.POINTER(ct.c_double))

def render(path):
    lib=ct.CDLL(str(path));pdouble=ct.POINTER(ct.c_double)
    lib.surface_create.argtypes=[ct.c_double,ct.c_double];lib.surface_create.restype=ct.c_void_p
    lib.surface_destroy.argtypes=[ct.c_void_p];lib.surface_reset.argtypes=[ct.c_void_p]
    lib.surface_add.argtypes=[ct.c_void_p,pdouble,ct.c_int,pdouble,ct.c_int]
    lib.surface_size.argtypes=[ct.c_void_p];lib.surface_copy.argtypes=[ct.c_void_p,pdouble,ct.c_int]
    lib.surface_error.restype=ct.c_char_p
    if hasattr(lib,'surface_capacity'):lib.surface_capacity.argtypes=[ct.c_void_p];lib.surface_capacity.restype=ct.c_uint
    handle=lib.surface_create(.25,40.);assert handle,lib.surface_error()
    def check(code):assert code>=0,lib.surface_error();return code
    try:
        check(lib.surface_reset(handle));states=[]
        # Repeated and novel voxels exercise exact count/sum preservation during rehash.
        batches=[geometry[i:i+16] for i in range(0,len(geometry),16)]+[geometry[:32]]
        for batch in batches:
            check(lib.surface_add(handle,ptr(batch),len(batch),ptr(sphere),len(sphere)))
            count=check(lib.surface_size(handle));output=np.empty((count,3))
            check(lib.surface_copy(handle,ptr(output),count))
            states.append(output[np.lexsort(output.T[::-1])])
        capacity=lib.surface_capacity(handle) if hasattr(lib,'surface_capacity') else None
        check(lib.surface_reset(handle));check(lib.surface_add(handle,ptr(geometry[:16]),16,ptr(sphere),len(sphere)))
        count=check(lib.surface_size(handle));reset=np.empty((count,3));check(lib.surface_copy(handle,ptr(reset),count))
        assert np.array_equal(reset[np.lexsort(reset.T[::-1])],states[0])
        return states,capacity
    finally:lib.surface_destroy(handle)

reference,_=render(args.reference)
grown,capacity=render(args.tiny_library)
normal,_=render(args.library)
assert capacity>16
assert all(np.array_equal(a,b) and np.array_equal(a,c) for a,b,c in zip(reference,grown,normal))
result=dict(passed=True,forced_initial_capacity=16,final_capacity=capacity,batches=len(reference),
            final_voxels=len(reference[-1]),bitwise_centroid_equality=True,reset_reuse_verified=True)
args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
