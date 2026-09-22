import json,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,Imu
bag=Path('/data/s3e/S3Ev2/S3E_Library_2/S3E_Library_2.db3')
c=sqlite3.connect(f'file:{bag}?mode=ro',uri=True);result={}
for topic,typ in [('/Bob/velodyne_points',PointCloud2),('/Bob/imu/data',Imu)]:
    tid=c.execute('select id from topics where name=?',(topic,)).fetchone()[0]
    n=c.execute('select count(*) from messages where topic_id=?',(tid,)).fetchone()[0]
    result[topic]={'messages':n}
    for side,order in [('first','asc'),('last','desc')]:
        t,data=c.execute(f'select timestamp,data from messages where topic_id=? order by timestamp {order} limit 1',(tid,)).fetchone()
        msg=deserialize_message(data,typ);h=msg.header.stamp
        result[topic][side+'_record_stamp_ns']=t
        result[topic][side+'_header_stamp_ns']=h.sec*10**9+h.nanosec
c.close()
out=Path('/workspace/.ros2/upstream-ellipselio-20260921/input-bounds.json');out.write_text(json.dumps(result,indent=2)+'\n');print(out.read_text())
