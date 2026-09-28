"""Independent vertical-ray oracle for solid-ellipsoid projected coverage."""
import os
from pathlib import Path
import sys
import subprocess
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.ellipsoid_raster import rasterize,level_geometry

@pytest.fixture(scope='module',autouse=True)
def build_reference(tmp_path_factory):
    if 'S3E_ELLIPSOID_RASTER_DIR' not in os.environ:
        root=tmp_path_factory.mktemp('raster');project=Path(__file__).resolve().parents[2]
        subprocess.run(['bash',str(project/'scripts/build_ellipsoid_raster.sh'),str(root)],check=True)
        os.environ['S3E_ELLIPSOID_RASTER_DIR']=str(root)

def primitive(center=(0,0,0),axes=(1,1,1),basis=None):
    return np.r_[center,axes,(np.eye(3) if basis is None else basis).ravel()][None,:]

def ray_oracle(e,ground,radius,resolution,lo=None,hi=None,terrain=(0,0,0),samples=400):
    # Independent 3-D quadratic along vertical rays, with midpoint integration.
    centers,shapes=level_geometry(e,ground);Q=np.linalg.inv(shapes)
    origin=int(np.floor(-radius/resolution));size=int(np.ceil(radius/resolution))-origin
    image=np.zeros((size,size));u=(np.arange(samples)+.5)/samples
    for i in range(size):
        x=(origin+i+u)*resolution
        for j in range(size):
            y=(origin+j+u)*resolution
            xx,yy=np.meshgrid(x,y,indexing='ij');disk=xx**2+yy**2<=radius**2
            total=0
            for c,q in zip(centers,Q):
                dx=xx-c[0];dy=yy-c[1]
                b=q[2,0]*dx+q[2,1]*dy
                d=q[0,0]*dx**2+2*q[0,1]*dx*dy+q[1,1]*dy**2-1
                discriminant=b*b-q[2,2]*d
                valid=discriminant>=0
                if lo is not None:
                    half=np.sqrt(np.maximum(discriminant,0))/q[2,2]
                    middle=c[2]-b/q[2,2]-(terrain[0]*xx+terrain[1]*yy+terrain[2])
                    valid&=(middle+half>=lo)&(middle-half<=hi)
                total+=np.count_nonzero(valid&disk)
            image[i,j]=total/samples**2
    return image

@pytest.mark.parametrize('axes,angles,center',[
    ((1,1,1),(0,0,0),(0,0,0)),
    ((1.3,.7,.25),(.5,.3,.8),(.2,-.3,.8)),
    ((1.3,.002,.001),(.1,.5,.7),(.17,.11,0)),
    ((1,.7,.3),(.3,-.2,.8),(1.7,.4,.6))])
def test_coverage_oracle(axes,angles,center):
    e=primitive(center,axes,Rotation.from_euler('xyz',angles).as_matrix())
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',(.2,-.4,.1)).as_matrix()
    r=rasterize(e,T,2,.5,0,coefficients=np.array([.1,-.05,.2]))
    expected=ray_oracle(e,T,2,.5)
    assert np.max(np.abs(r['coverage']['fullheight']-expected))<.0015
    expected=ray_oracle(e,T,2,.5,-.5,2,(.1,-.05,.2))
    assert np.max(np.abs(r['coverage']['near']-expected))<.0015

def test_areas_overlap_and_band_boundary():
    e=primitive(center=(0,0,2))
    r=rasterize(e,np.eye(4),2,.25,0,coefficients=[0,0,0])
    assert r['coverage']['fullheight'].sum()*.25**2==pytest.approx(np.pi,abs=1e-6)
    # Both half-spheres project to the entire disk; a center-only split is wrong.
    assert np.allclose(r['coverage']['near'],r['coverage']['fullheight'],atol=1e-6)
    assert np.allclose(r['coverage']['low'],r['coverage']['fullheight'],atol=1e-6)
    doubled=rasterize(np.r_[e,e],np.eye(4),2,.25,0)
    assert np.allclose(doubled['coverage']['fullheight'],2*r['coverage']['fullheight'])
    assert np.array_equal(doubled['images']['fullheight'],r['images']['fullheight'])

def test_empty_invalid_and_thin():
    empty=rasterize(np.empty((0,15)),np.eye(4),2)
    assert not empty['images']['fullheight'].any()
    e=primitive(axes=(.002,.001,.0005))
    r=rasterize(e,np.eye(4),2,.5,0)
    assert r['coverage']['fullheight'].sum()*.5**2==pytest.approx(np.pi*.002*.001,rel=1e-5)
    bad=e.copy();bad[0,3]=0
    with pytest.raises(ValueError):rasterize(bad,np.eye(4),2)
    bad=e.copy();bad[0,6]=2
    with pytest.raises(ValueError):rasterize(bad,np.eye(4),2)
    T=np.eye(4);T[0,0]=2
    with pytest.raises(ValueError):rasterize(e,T,2)

@pytest.mark.skipif(os.environ.get('S3E_TEST_CUDA')!='1',reason='CUDA runtime explicitly enabled on 148')
def test_cpu_cuda_gray_agreement():
    rng=np.random.default_rng(43)
    e=np.concatenate([primitive(rng.uniform(-2,2,3),np.sort(rng.uniform(.002,.9,3))[::-1],
        Rotation.random(random_state=rng).as_matrix()) for _ in range(70)])
    args=(e,np.eye(4),3,.3,.05,[.1,-.1,.3])
    cpu=rasterize(*args,device='cpu');gpu=rasterize(*args,device='cuda')
    for name in cpu['images']:
        assert np.max(np.abs(cpu['images'][name].astype(int)-gpu['images'][name].astype(int)))<=1
        assert np.allclose(cpu['coverage'][name],gpu['coverage'][name],atol=1e-7)

@pytest.mark.skipif(os.environ.get('S3E_TEST_CUDA')!='1',reason='CUDA runtime explicitly enabled on 148')
def test_cuda_empty_and_multiple_scratch_and_primitive_batches():
    empty=rasterize(np.empty((0,15)),np.eye(4),2.1,.3,device='cuda')
    assert not empty['coverage']['fullheight'].any()
    # More than 4096 primitives; their tile references also cross the parallel
    # implementation's 16384-reference scratch boundary inside the first batch.
    e=np.repeat(primitive(axes=(.8,.6,.3)),4101,axis=0)
    cpu=rasterize(e,np.eye(4),4,.1,0,device='cpu')
    gpu=rasterize(e,np.eye(4),4,.1,0,device='cuda')
    assert gpu['diagnostics']['peak_tile_references']>16384
    assert np.max(np.abs(cpu['images']['fullheight'].astype(int)-gpu['images']['fullheight'].astype(int)))<=1
    assert np.allclose(cpu['coverage']['fullheight'],gpu['coverage']['fullheight'],rtol=1e-9,atol=1e-7)
