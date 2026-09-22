"""CUDA acceleration of the existing deterministic ellipsoid surface sampler."""
import ctypes as ct
from pathlib import Path
import numpy as np
from .ellipsoid_bev import unit_surface,validate_ellipsoids


class SurfaceSampler:
    def __init__(self,voxel=.25,max_range=80.):
        path=Path(__file__).resolve().parents[3]/'.ros2/ellipsoid-cuda/libellipsoid_surface.so'
        self.lib=ct.CDLL(str(path));self.voxel=voxel;self.max_range=max_range
        pointer=ct.POINTER(ct.c_double);lib=self.lib
        lib.surface_create.argtypes=[ct.c_double,ct.c_double];lib.surface_create.restype=ct.c_void_p
        lib.surface_destroy.argtypes=[ct.c_void_p];lib.surface_destroy.restype=None
        lib.surface_error.restype=ct.c_char_p
        lib.surface_reset.argtypes=[ct.c_void_p];lib.surface_size.argtypes=[ct.c_void_p]
        lib.surface_add.argtypes=[ct.c_void_p,pointer,ct.c_int,pointer,ct.c_int]
        lib.surface_copy.argtypes=[ct.c_void_p,pointer,ct.c_int]
        self.pointer=lib.surface_create(voxel,max_range)
        if not self.pointer:raise RuntimeError(lib.surface_error().decode())
    def check(self,result):
        if result<0:raise RuntimeError(self.lib.surface_error().decode())
        return result
    def close(self):
        if self.pointer:self.lib.surface_destroy(self.pointer);self.pointer=None
    def render(self,centers,axes,basis,spacing=.125):
        import small_gicp
        validate_ellipsoids(centers,axes,basis)
        required=np.maximum(32,np.ceil(4*np.pi*(axes.max(axis=1)/spacing)**2)).astype(int)
        counts=(2**np.ceil(np.log2(required))).astype(int)
        if counts.max()>8192:raise ValueError('Ellipsoid sample budget exceeded')
        geometry=np.ascontiguousarray(np.column_stack([centers,axes,basis.reshape(-1,9)]),dtype=np.float64)
        ptr=lambda x:x.ctypes.data_as(ct.POINTER(ct.c_double))
        self.check(self.lib.surface_reset(self.pointer));generated=0
        for count in np.unique(counts):
            ids=np.flatnonzero(counts==count);sphere=np.ascontiguousarray(unit_surface(int(count)))
            batch=max(1,250000//len(sphere))
            for start in range(0,len(ids),batch):
                rows=geometry[ids[start:start+batch]];generated+=len(rows)*len(sphere)
                self.check(self.lib.surface_add(self.pointer,ptr(rows),len(rows),ptr(sphere),len(sphere)))
        count=self.check(self.lib.surface_size(self.pointer));output=np.empty((count,3))
        self.check(self.lib.surface_copy(self.pointer,ptr(output),count))
        # Native sampler ordering and final voxel guard, with one centroid per
        # voxel. Quantization can move an exact boundary by <=0.1 micrometre.
        output=small_gicp.voxelgrid_sampling(output,self.voxel,num_threads=1).points()[:,:3].copy()
        return output,dict(surface_samples=generated,voxel_points=len(output),spacing_m=spacing,
                           voxel_m=self.voxel,device='CUDA',accumulator_resolution_m=1e-7)
