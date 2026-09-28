"""Direct geometric coverage; integrates analytic projected ellipsoid slices."""
import ctypes
import os
from pathlib import Path
import time
import numpy as np
from .ellipsoid_registration import validate
from .multilayer_bev import BANDS, fit_terrain

VERSION = 'ellipsoid-geometric-coverage-v1'


def native_library(device):
    if device not in ('cpu', 'cuda'): raise ValueError('Raster device must be cpu or cuda')
    root = Path(os.environ.get('S3E_ELLIPSOID_RASTER_DIR', '/workspace/.ros2/ellipsoid-raster'))
    lib = ctypes.CDLL(str(root / f'libellipsoid_raster_{device}.so'))
    fn = getattr(lib, f'raster_{device}')
    ptr = ctypes.c_void_p
    fn.argtypes = [ptr, ctypes.c_int, ptr, ptr, ptr, ctypes.c_int,
                   ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_int, ctypes.c_int, ptr]
    if device == 'cuda': fn.argtypes += [ctypes.POINTER(ctypes.c_size_t)]
    fn.restype = ctypes.c_int
    fn.variant='cpu_reference'
    if device=='cuda':
        try:
            variant=lib.raster_cuda_variant;variant.argtypes=[];variant.restype=ctypes.c_char_p
            fn.variant=variant().decode('ascii')
        except AttributeError:
            fn.variant='pixel_serial_v1'  # Frozen first-version library.
    return fn


def level_geometry(ellipsoids, ground):
    e = validate(ellipsoids); ground = np.asarray(ground, dtype=float)
    if ground.shape!=(4,4):raise ValueError('Invalid raster gravity transform')
    R = ground[:3, :3]
    if ( not np.isfinite(ground).all() or
            not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(R), 1) or not np.allclose(ground[3], [0,0,0,1])):
        raise ValueError('Invalid raster gravity transform')
    centers = e[:, :3] @ R.T + ground[:3, 3]
    basis = R @ e[:, 6:].reshape(-1, 3, 3)
    with np.errstate(over='ignore',under='ignore',invalid='ignore'):
        squared=e[:,3:6]**2
        shapes = (basis * squared[:,None,:]) @ basis.transpose(0,2,1)
    if not np.isfinite(shapes).all() or np.any(squared<=0):raise ValueError('Unrepresentable ellipsoid shape')
    return centers, shapes


def terrain_from_ellipsoids(ellipsoids, ground):
    centers, _ = level_geometry(ellipsoids, ground)
    try:
        coefficients, info, _, _ = fit_terrain(centers)
        info.update(available=True, geometry='native ellipsoid centers')
        return coefficients, info
    except ValueError as error:
        return None, dict(available=False, reason=str(error), ground_truth_used=False,
                          geometry='native ellipsoid centers')


def rasterize(ellipsoids, ground, radius, resolution=.5, threshold=.05,
              coefficients=None, device='cpu'):
    start = time.perf_counter()
    if not np.isfinite([radius,resolution,threshold]).all() or radius<=0 or resolution<=0 or not 0<=threshold<1:
        raise ValueError('Invalid raster region/resolution/threshold')
    centers, shapes = level_geometry(ellipsoids, ground)
    origin = int(np.floor(-radius/resolution)); size = int(np.ceil(radius/resolution))-origin
    if size>16384: raise ValueError('Raster exceeds supported image dimensions')
    primitives = np.ascontiguousarray(np.c_[centers, shapes.reshape(-1,9)], dtype=np.float64)
    bands = [[0,0,0,0,0,0]]; names = ['fullheight']
    if coefficients is not None:
        coefficients=np.asarray(coefficients,dtype=float)
        if coefficients.shape!=(3,) or not np.isfinite(coefficients).all():raise ValueError('Invalid terrain plane')
        bands += [[*coefficients,lo,1e300 if hi is None else hi,1] for _,_,lo,hi in BANDS]
        names += [b[0] for b in BANDS]
    bands=np.ascontiguousarray(bands,dtype=np.float64)
    tile=8; nt=(size+tile-1)//tile
    # A fixed primitive batch bounds the CSR tile lists independently of the
    # snapshot size. No N_ellipsoids x N_pixels temporary is constructed.
    batch_size=4096;values=np.zeros((len(names),size,size),dtype=np.float64)
    ptr=lambda a:a.ctypes.data_as(ctypes.c_void_p)
    call=native_library(device);raster_s=0.;peak_bytes=0;tile_references=0;peak_references=0
    for begin in range(0,max(1,len(primitives)),batch_size):
        batch=primitives[begin:begin+batch_size];local_shapes=shapes[begin:begin+batch_size]
        local_centers=centers[begin:begin+batch_size];bins=[[] for _ in range(nt*nt)]
        extents=np.sqrt(np.diagonal(local_shapes,axis1=1,axis2=2)[:,:2])
        # Clip before integer conversion, including shapes with very large axes.
        lower=np.floor(np.clip((local_centers[:,:2]-extents)/resolution-origin,-1,size)).astype(np.int64)
        upper=np.floor(np.clip((local_centers[:,:2]+extents)/resolution-origin,-1,size)).astype(np.int64)
        for i,(lo,hi) in enumerate(zip(lower,upper)):
            if np.any(hi<0) or np.any(lo>=size):continue
            lo=np.maximum(lo,0)//tile;hi=np.minimum(hi,size-1)//tile
            for x in range(int(lo[0]),int(hi[0])+1):
                for y in range(int(lo[1]),int(hi[1])+1):bins[x*nt+y].append(i)
        offsets=np.r_[0,np.cumsum([len(b) for b in bins])]
        if offsets[-1]>np.iinfo(np.int32).max:raise ValueError('Raster tile index overflow')
        offsets=np.ascontiguousarray(offsets,dtype=np.int32)
        ids=np.ascontiguousarray([i for b in bins for i in b],dtype=np.int32)
        local_values=np.empty_like(values);peak=ctypes.c_size_t(0)
        args=[ptr(batch),len(batch),ptr(offsets),ptr(ids),ptr(bands),len(names),radius,resolution,origin,size,tile,ptr(local_values)]
        if device=='cuda':args.append(ctypes.byref(peak))
        raster_start=time.perf_counter();status=call(*args);raster_s+=time.perf_counter()-raster_start
        if status:raise RuntimeError(f'{device} ellipsoid raster failed: {status}')
        values+=local_values;peak_bytes=max(peak_bytes,peak.value)
        tile_references+=len(ids);peak_references=max(peak_references,len(ids))
    if not np.isfinite(values).all() or np.any(values<0):raise ValueError('Invalid raster coverage')
    maximum=values.max(axis=(1,2),keepdims=True)
    density=np.divide(values,maximum,out=np.zeros_like(values),where=maximum>0)
    images=np.floor(np.where(density>threshold,density,0)*255+1e-10).astype(np.uint8)
    return dict(images=dict(zip(names,images)),coverage=dict(zip(names,values)),
        lower_bound=np.array([origin,origin],dtype=np.int32),resolution=resolution,
        ground=np.asarray(ground),diagnostics=dict(version=VERSION,device=device,ellipsoids=len(primitives),
        raster_s=raster_s,total_s=time.perf_counter()-start,execution_variant=call.variant,
        allocated_gpu_bytes=peak_bytes,primitive_batch_size=batch_size,peak_tile_references=peak_references,
        tile_references=tile_references,grid_shape=[size,size],layers=names,
        normalization='additive projected pixel coverage / layer maximum; normalized threshold',
        geometry='solid ellipsoid intersected with height slab and horizontal disk; no surface points'))
