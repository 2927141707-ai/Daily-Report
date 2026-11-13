#!/usr/bin/env python3
import json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import cv2
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
ROOT=Path(__file__).resolve().parents[2]
run=ROOT/'work/mapper_service_tests'/str(time.time_ns());run.mkdir(parents=True)
for name in ('rgb','depth'):(run/name).mkdir()
cv2.imwrite(str(run/'rgb/a.png'),np.full((24,32,3),[20,100,220],np.uint8))
cv2.imwrite(str(run/'depth/a.png'),np.full((24,32),1000,np.uint16))
intr=dict(width=32,height=24,fx=40,fy=40,cx=15.5,cy=11.5,depth_units_per_metre=1000,rectified=True)
k=dict(id=0,frame_id=0,key='a',timestamp=1.,rgb='rgb/a.png',depth='depth/a.png',pose=[0,0,0,0,0,0,1])
s=dict(schema=2,version=1,final=False,run_dir=str(run),active_map=0,frame_prefix='slam_map_',intrinsics=intr,maps=[dict(id=0,change_index=0,keyframes=[k])],writer=dict(written=1,dropped=0,failed=0))
(run/'intrinsics.json').write_text(json.dumps(intr))
exe=ROOT/'work/dense_ros2_ws/install/slam3_orbbec/lib/slam3_orbbec'
os.environ['ROS_LOG_DIR']=str(run/'logs')
rclpy.init();node=rclpy.create_node('mapper_test_fixture')
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub=node.create_publisher(String,'/orb_slam3/snapshot',qos)
def respond(req,res):res.success=True;res.message=json.dumps(s);return res
srv=node.create_service(Trigger,'/orb_slam3/get_snapshot',respond)
clouds=[]
def cloud(msg):
    data=np.frombuffer(msg.data,dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')]))
    clouds.append((msg.width,float(data['x'].mean()) if len(data) else None,msg.header.frame_id,int(data['rgb'][0]) if len(data) else None))
sub=node.create_subscription(PointCloud2,'/orb_slam3/dense_map',cloud,qos)
log=(run/'dense.log').open('w');p=subprocess.Popen([str(exe/'dense_node.py'),'--ros-args','-p',f'output_dir:={run}','-p','auto_save:=false','-p','stride:=1','-p','outlier_neighbors:=0'],stdout=log,stderr=subprocess.STDOUT)
loader=None

def wait_for(predicate,seconds=20):
    end=time.monotonic()+seconds
    while time.monotonic()<end and not predicate():rclpy.spin_once(node,timeout_sec=.05)
    assert predicate(),'ROS condition timed out; see '+str(run)
try:
    pub.publish(String(data=json.dumps(s)));wait_for(lambda:len(clouds)>0)
    assert clouds[-1][0]==768 and abs(clouds[-1][1])<1e-6
    assert clouds[-1][3]==(220<<16)|(100<<8)|20
    s['version']=2;k['pose'][0]=2.;s['maps'][0]['change_index']=1
    pub.publish(String(data=json.dumps(s)));wait_for(lambda:any(c[1] and c[1]>1.9 for c in clouds))
    client=node.create_client(Trigger,'/orb_slam3/save_map');assert client.wait_for_service(timeout_sec=5)
    future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=30)
    assert future.done() and future.result().success,str(future.result())
    bundle=Path(future.result().message);assert (bundle/'map_0.ply').exists()
    p.send_signal(signal.SIGINT);p.wait(timeout=15);assert p.returncode==0
    reload_log=(run/'reload.log').open('w');loader=subprocess.Popen([str(exe/'reload_map.py'),str(bundle)],stdout=reload_log,stderr=subprocess.STDOUT)
    before=len(clouds);wait_for(lambda:len(clouds)>before)
    assert clouds[-1][0]==768 and abs(clouds[-1][1]-2)<1e-6
    loader.send_signal(signal.SIGINT);loader.wait(timeout=10);assert loader.returncode==0
    report=dict(save_service=True,live_pointcloud=True,rgb_packing=True,corrected_pose_republished=True,reload=True,points=768,bundle=str(bundle),input='synthetic ROS snapshot fixture, not SLAM/camera')
    (run/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
finally:
    for child in (p,loader):
        if child and child.poll() is None:child.send_signal(signal.SIGINT);child.wait(timeout=20)
    log.close();node.destroy_node();rclpy.shutdown()
