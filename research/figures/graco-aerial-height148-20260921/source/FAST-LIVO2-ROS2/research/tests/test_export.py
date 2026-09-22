import io
import struct
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import canonical,read_json
from s3e_pipeline.mcap_writer import consume
from rosbags.typesys import Stores,get_typestore


def packet(stamp_delta=0,robot='Alpha'):
    ts=get_typestore(Stores.ROS2_HUMBLE);types=ts.types
    stamp=types['builtin_interfaces/msg/Time'](1_661_157_300,123456789+stamp_delta)
    header=types['std_msgs/msg/Header'](stamp,'Alpha/camera')
    message=types['sensor_msgs/msg/CompressedImage'](header,'png',np.array([1,2,3],dtype=np.uint8))
    data=bytes(ts.serialize_cdr(message,'sensor_msgs/msg/CompressedImage'))
    row=dict(robot_id=robot,frame_id=0,stamp_ns=1_661_157_300_123456789,
        messages=[dict(topic='/Alpha/research/image',type='sensor_msgs/msg/CompressedImage',size=len(data))])
    metadata=canonical(row)
    return struct.pack('<I',len(metadata))+metadata+data


def test_exact_nanosecond_association_and_explicit_completion(tmp_path):
    consume(io.BytesIO(packet()+struct.pack('<I',0)),tmp_path/'valid')
    result=read_json(tmp_path/'valid/manifest.json')
    assert result['last_stamp_ns']==1_661_157_300_123456789 and result['complete']
    with pytest.raises(ValueError,match='timestamp association'):
        consume(io.BytesIO(packet(1)+struct.pack('<I',0)),tmp_path/'mismatch')
    with pytest.raises(EOFError):consume(io.BytesIO(packet()),tmp_path/'truncated')
    assert not (tmp_path/'truncated/manifest.json').exists()
    with pytest.raises(ValueError,match='Cross-robot'):
        consume(io.BytesIO(packet(robot='Bob')+struct.pack('<I',0)),tmp_path/'namespace')


def test_frame_reader_yields_before_reading_all_chunks(tmp_path,monkeypatch):
    from s3e_pipeline import data,mcap_writer
    writer=mcap_writer.Writer
    monkeypatch.setattr(mcap_writer,'Writer',lambda *args,**kwargs:writer(*args,**dict(kwargs,chunk_size=1024)))
    ts=get_typestore(Stores.ROS2_HUMBLE);t=ts.types;packets=[]
    for key in range(40):
        stamp=t['builtin_interfaces/msg/Time'](1_661_157_300,key)
        h=t['std_msgs/msg/Header'](stamp,'Alpha/lidar')
        fields=[t['sensor_msgs/msg/PointField'](name,4*i,7,1) for i,name in enumerate(['x','y','z','intensity'])]
        cloud=t['sensor_msgs/msg/PointCloud2'](h,1,4,fields,False,16,64,np.zeros(64,dtype=np.uint8),True)
        image=t['sensor_msgs/msg/CompressedImage'](t['std_msgs/msg/Header'](stamp,'Alpha/camera'),'png',np.array([1,2,3],dtype=np.uint8))
        messages=[('cloud','sensor_msgs/msg/PointCloud2',cloud),('image','sensor_msgs/msg/CompressedImage',image)]
        serialized=[bytes(ts.serialize_cdr(msg,typename)) for _,typename,msg in messages]
        row=dict(robot_id='Alpha',frame_id=key,stamp_ns=stamp.sec*10**9+stamp.nanosec,
            messages=[dict(topic='/Alpha/research/'+name,type=typename,size=len(raw))
                for (name,typename,_),raw in zip(messages,serialized)])
        metadata=canonical(row);packets.append(struct.pack('<I',len(metadata))+metadata+b''.join(serialized))
    root=tmp_path/'streamed';consume(io.BytesIO(b''.join(packets)+struct.pack('<I',0)),root)
    streams=[];reader=data.NonSeekingReader
    def spy(stream):streams.append(stream);return reader(stream)
    monkeypatch.setattr(data,'NonSeekingReader',spy)
    frames=data.frames(root,verify=False);row,cloud,image=next(frames)
    assert row['frame_id']==0 and cloud.shape==(4,4) and image==b'\x01\x02\x03'
    assert streams[0].tell()<(root/'sensors.mcap').stat().st_size/2
    rest=list(frames);assert len(rest)==39 and rest[-1][0]['frame_id']==39


@pytest.mark.parametrize('explicit_image_free',[True,False])
def test_image_free_mcap_requires_explicit_metadata(tmp_path,explicit_image_free):
    from s3e_pipeline.data import frames
    ts=get_typestore(Stores.ROS2_HUMBLE);t=ts.types
    stamp=t['builtin_interfaces/msg/Time'](1661157300,123456789)
    h=t['std_msgs/msg/Header'](stamp,'Alpha/lidar')
    fields=[t['sensor_msgs/msg/PointField'](name,4*i,7,1) for i,name in enumerate(['x','y','z','intensity'])]
    points=np.array([[3,4,5,6],[7,8,9,10]],dtype='<f4')
    cloud=t['sensor_msgs/msg/PointCloud2'](h,1,2,fields,False,16,32,points.view(np.uint8).ravel(),True)
    raw=bytes(ts.serialize_cdr(cloud,'sensor_msgs/msg/PointCloud2'))
    row=dict(robot_id='Alpha',frame_id=0,stamp_ns=1661157300123456789,frontend='ellipselio',
        cloud_source='native full deskewed cloud',pose_source='native post-LiDAR update',
        messages=[dict(topic='/Alpha/research/cloud',type='sensor_msgs/msg/PointCloud2',size=len(raw))])
    if explicit_image_free:row['image_available']=False
    meta=canonical(row)
    consume(io.BytesIO(struct.pack('<I',len(meta))+meta+raw+struct.pack('<I',0)),tmp_path)
    if explicit_image_free:
        item,decoded,image=next(frames(tmp_path))
        assert np.array_equal(decoded,points) and image==b'' and item['stamp_ns']==row['stamp_ns']
        assert read_json(tmp_path/'manifest.json')['frontend']=='ellipselio'
    else:
        with pytest.raises(ValueError,match='explicit image-free'):next(frames(tmp_path))
