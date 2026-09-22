from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from s3e_pipeline.graco import ROBOTS, connectivity, mapping_config, validate_cloud


def test_six_robot_connection_requires_measured_chain_and_common_frame():
    edges=[dict(i=[a,0],j=[b,0]) for a,b in zip(ROBOTS,ROBOTS[1:])]
    poses=[dict(robot_id=r,component='robot1') for r in ROBOTS]
    assert connectivity(ROBOTS,edges,poses)['all_six_connected']
    assert not connectivity(ROBOTS,edges[:-1],poses)['all_six_connected']
    poses[-1]['component']='robot6'
    result=connectivity(ROBOTS,edges,poses)
    assert len(result['measured_components'])==1
    assert not result['common_frame_contract']


def cloud():
    dtype=np.dtype([('x','f4'),('y','f4'),('z','f4'),('intensity','f4'),('ring','u2'),('time','f4')])
    data=np.zeros(1600,dtype=dtype);data['ring']=np.arange(1600)%16;data['time']=np.linspace(-.0001,.1001,1600)
    return data,NS(fields=[NS(name=n,datatype=4 if n=='ring' else 7,count=1,offset=dtype.fields[n][1]) for n in dtype.names],
                   width=len(data),height=1,point_step=dtype.itemsize,row_step=len(data)*dtype.itemsize,
                   data=data, is_bigendian=False,header=NS(frame_id='velodyne'))


def test_native_velodyne_point_timing_is_required():
    data,msg=cloud()
    # bytes, as in ROS PointCloud2; include the small negative native offsets.
    msg.data=data.tobytes()
    assert validate_cloud(msg)['point_time_min_s']<0
    data['time']*=2;msg.data=data.tobytes()
    assert validate_cloud(msg)['point_time_max_s']>.2
    data['time']*=1000;msg.data=data.tobytes()
    with pytest.raises(ValueError,match='point time'):validate_cloud(msg)
    data['time']=0;msg.data=data.tobytes()
    with pytest.raises(ValueError,match='point time'):validate_cloud(msg)
    msg.fields=[f for f in msg.fields if f.name!='time']
    with pytest.raises(ValueError,match='native Velodyne'):validate_cloud(msg)


def test_graco_uses_dataset_calibration_and_records_analytics(tmp_path):
    import yaml
    T=np.eye(4);T[:3,3]=[.1,-.2,.3]
    (tmp_path/'imu-lidar.yaml').write_text(yaml.safe_dump({'T_Imu_Lidar':{'data':T.ravel().tolist()}}))
    (tmp_path/'imu.yaml').write_text(yaml.safe_dump(dict(accelerometer_noise_density=.01,gyroscope_noise_density=.001,
        accelerometer_random_walk=.0001,gyroscope_random_walk=.00001)))
    source=Path(__file__).resolve().parents[3]
    p=mapping_config(source,tmp_path,'robot6',dict(translation_m=1.,rotation_deg=10.,max_interval_s=2.))['/**']['ros__parameters']
    assert p['lidar']['t_imu_lidar']==[.1,-.2,.3]
    assert np.allclose(np.array(p['lidar']['r_imu_lidar']).reshape(3,3),np.eye(3))
    assert p['imu']['rate']==125 and p['imu']['acc_noise']==.01
    assert p['publish']['analytics'] and p['mapping']['namespace']=='robot6'
