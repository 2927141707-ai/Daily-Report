#!/usr/bin/env python3
"""Exercise the real ROS2 bridge/core on existing TUM data; NOT a camera test."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from std_srvs.srv import Trigger
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parents[2]
run=ROOT/'work/integration_runs'/str(time.time_ns());run.mkdir(parents=True)
exe=ROOT/'work/dense_ros2_ws/install/slam3_orbbec/lib/slam3_orbbec'
core=ROOT/'work/ORB_SLAM3_p06'
data=Path('/home/wmy/slam/datasets/rgbd_dataset_freiburg1_xyz')
rows=[s.split() for s in (core/'Examples/RGB-D/associations/fr1_xyz.txt').read_text().splitlines() if s and not s.startswith('#')]
os.environ['ROS_LOG_DIR']=str(run/'ros_logs')
procs=[];logs=[]
def start(name,command):
    log=(run/(name+'.log')).open('w');logs.append(log)
    if name=='tracker' and os.environ.get('SLAM_TEST_GDB'):
        command=['gdb','-batch','-ex','run','-ex','thread apply all bt','--args',*command]
    p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT);procs.append(p);return p

def stop(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:p.wait(timeout=100)
        except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=10);raise RuntimeError('Shutdown timed out')

rclpy.init();node=rclpy.create_node('camera',namespace='/camera')
for name,val in [('depth_registration',True),('enable_depth_scale',True),('enable_color_undistortion',False),('enable_depth_undistortion',False)]:node.declare_parameter(name,val)
rgbpub=node.create_publisher(Image,'/camera/color/image_raw',qos_profile_sensor_data)
depthpub=node.create_publisher(Image,'/camera/depth/image_raw',qos_profile_sensor_data)
infopub=node.create_publisher(CameraInfo,'/camera/color/camera_info',qos_profile_sensor_data)
received=[];states=[]
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
sub=node.create_subscription(PointCloud2,'/orb_slam3/dense_map',lambda m:received.append((m.width,m.header.frame_id)),qos)
state_sub=node.create_subscription(String,'/orb_slam3/state',lambda m:states.append(m.data),10)
launch_mode=os.environ.get('SLAM_TEST_LAUNCH')=='1'
if launch_mode:
    tracker=start('launch',['ros2','launch','slam3_orbbec','live.launch.py',
        'start_camera:=false','viewer:=false','record_dense:=true',f'output_dir:={run}'])
    dense=None
else:
    tracker=start('tracker',[str(exe/'rgbd_node'),'--ros-args','-p',f'vocabulary:={core}/Vocabulary/ORBvoc.txt','-p',f'output_dir:={run}','-p','viewer:=false','-p','record_dense:=true','-p','snapshot_period:=2.0'])
    dense=start('dense',[str(exe/'dense_node.py'),'--ros-args','-p',f'output_dir:={run}','-p','final_wait_seconds:=30.0'])
print(f'RUN={run}',flush=True)
try:
    deadline=time.monotonic()+4
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    for i,row in enumerate(rows[:200]):
        if tracker.poll() is not None:raise RuntimeError(f'Tracker exited ({tracker.returncode}); see tracker.log')
        color=cv2.imread(str(data/row[1]));raw=cv2.imread(str(data/row[3]),cv2.IMREAD_UNCHANGED)
        depth=np.rint(raw.astype(float)/5).astype(np.uint16) # TUM 5000 units/m -> mm
        stamp=node.get_clock().now().to_msg()
        info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='camera_color_optical_frame';info.width=640;info.height=480
        info.k=[517.3,0.,318.6,0.,516.5,255.3,0.,0.,1.]
        info.d=[0.2624,-0.9531,-0.0054,0.0026,1.1633];info.distortion_model='plumb_bob';infopub.publish(info)
        for array,enc,pub in [(color,'bgr8',rgbpub),(depth,'16UC1',depthpub)]:
            msg=Image();msg.header=info.header;msg.width=640;msg.height=480;msg.encoding=enc
            msg.is_bigendian=False;msg.step=640*(3 if enc=='bgr8' else 2);msg.data=array.tobytes();pub.publish(msg)
        end=time.monotonic()+.065
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
    client=node.create_client(Trigger,'/orb_slam3/save_map')
    if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Save service missing')
    future=client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node,future,timeout_sec=90)
    if not future.done() or not future.result().success:raise RuntimeError('Save failed: '+str(future.result()))
    bundle=Path(future.result().message.split(' (partial')[0]);meta=json.loads((bundle/'metadata.json').read_text())
    print('SERVICE_SAVED='+str(bundle),flush=True)
    assert any(n>0 for n,_ in received),'No live dense cloud'
    assert 'TRACKING_OK' in states,'No tracking OK state'
    stop(tracker);assert (run/'final_snapshot.json').exists(),'Missing final snapshot'
    if dense is not None:stop(dense)
    shutdown_log=(run/('launch.log' if launch_mode else 'dense.log')).read_text()
    assert 'FINAL MAP SAVED:' in shutdown_log,'Auto-save failed'
    if launch_mode:
        assert 'escalating' not in shutdown_log,'Launch required forced termination'
    loader=start('reload',[str(exe/'reload_map.py'),str(bundle)])
    before=len(received);deadline=time.monotonic()+6
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    stop(loader)
    expected=meta['maps'][str(meta['active_map'])]['points']
    assert any(n==expected for n,_ in received[before:]),'Reloaded cloud count mismatch'
    report={'input':'TUM freiburg1_xyz replay through ROS2; not live Gemini camera','sent_frames':200,
            'tracking_ok_messages':states.count('TRACKING_OK'),'cloud_messages':len(received),'service_bundle':str(bundle),
            'active_map_points':expected,'shutdown_final_snapshot':True,'automatic_final_save':True,'reload_point_count_matches':True,
            'simultaneous_launch_shutdown':launch_mode}
    (run/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
finally:
    for p in procs:
        if p.poll() is None:stop(p)
    node.destroy_node();rclpy.shutdown()
    for f in logs:f.close()
