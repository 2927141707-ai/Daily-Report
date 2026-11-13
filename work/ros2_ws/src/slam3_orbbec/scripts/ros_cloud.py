import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def cloud_message(xyz,rgb,frame,stamp):
    msg=PointCloud2();msg.header.frame_id=frame;msg.header.stamp=stamp
    msg.height=1;msg.width=len(xyz);msg.is_bigendian=False;msg.is_dense=True
    msg.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z','rgb'))]
    msg.point_step=16;msg.row_step=16*len(xyz)
    data=np.zeros(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
    for i,k in enumerate(('x','y','z')):data[k]=xyz[:,i]
    colors=rgb.astype(np.uint32)
    data['rgb']=(colors[:,0]<<16)|(colors[:,1]<<8)|colors[:,2]
    msg.data=data.tobytes();return msg


def path_message(map_data,frame,stamp):
    path=Path();path.header.frame_id=frame;path.header.stamp=stamp
    for k in map_data['keyframes']:
        p=PoseStamped();p.header.frame_id=frame
        ns=round(k['timestamp']*1e9);p.header.stamp.sec=ns//1000000000;p.header.stamp.nanosec=ns%1000000000
        t=k['pose'];p.pose.position.x=float(t[0]);p.pose.position.y=float(t[1]);p.pose.position.z=float(t[2])
        p.pose.orientation.x=float(t[3]);p.pose.orientation.y=float(t[4]);p.pose.orientation.z=float(t[5]);p.pose.orientation.w=float(t[6])
        path.poses.append(p)
    return path
