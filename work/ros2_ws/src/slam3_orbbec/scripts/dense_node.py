#!/usr/bin/env python3
import json
from pathlib import Path
import threading
import time
import traceback
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path as PathMsg
from dense_mapping import Options, WorkQueue, rebuild, save_bundle, load_snapshot
from ros_cloud import cloud_message,path_message


class DenseNode(Node):
    def __init__(self):
        super().__init__('orb_slam3_dense')
        self.run=Path(self.declare_parameter('output_dir','slam3_run').value).resolve()
        defaults=Options()
        self.options=Options(**{k:self.declare_parameter(k,v).value for k,v in vars(defaults).items()})
        self.options.validate()
        self.final_wait=float(self.declare_parameter('final_wait_seconds',150.).value)
        self.auto_save=bool(self.declare_parameter('auto_save',True).value)
        self.group=ReentrantCallbackGroup()
        self.qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub=self.create_publisher(PointCloud2,'/orb_slam3/dense_map',self.qos)
        self.path_pub=self.create_publisher(PathMsg,'/orb_slam3/keyframe_path',self.qos)
        self.status_pub=self.create_publisher(String,'/orb_slam3/dense_status',self.qos)
        self.map_pubs={};self.last_maps=set();self.last_active=0
        self.client=self.create_client(Trigger,'/orb_slam3/get_snapshot',callback_group=self.group)
        self.sub=self.create_subscription(String,'/orb_slam3/snapshot',self.on_snapshot,self.qos,callback_group=self.group)
        self.srv=self.create_service(Trigger,'/orb_slam3/save_map',self.save,callback_group=self.group)
        self.queue=WorkQueue();self.saving=threading.Lock();self.worker=threading.Thread(target=self.work,name='dense-fusion')
        self.worker.start()
        self.get_logger().info(f'Dense mapper ready; outputs={self.run}; options={self.options}')

    def on_snapshot(self,msg):
        try:
            s=json.loads(msg.data)
            if Path(s['run_dir']).resolve()!=self.run:
                raise ValueError('Snapshot belongs to another recording session')
            self.queue.put_preview(s)
        except Exception as e:self.get_logger().error(str(e))

    def status(self,data):
        if rclpy.ok():self.status_pub.publish(String(data=json.dumps(data)))

    def publish(self,s,clouds):
        if not rclpy.ok():return
        stamp=self.get_clock().now().to_msg();empty=np.empty((0,3));active=s['active_map']
        for mid in self.last_maps-set(clouds):
            self.map_pubs[mid].publish(cloud_message(empty,empty,s['frame_prefix']+str(mid),stamp))
        for mid,c in clouds.items():
            if mid not in self.map_pubs:self.map_pubs[mid]=self.create_publisher(PointCloud2,f'/orb_slam3/dense_maps/map_{mid}',self.qos)
            self.map_pubs[mid].publish(cloud_message(c['xyz'],c['rgb'],s['frame_prefix']+str(mid),stamp))
        self.last_maps=set(clouds);self.last_active=active
        c=clouds.get(active,{'xyz':empty,'rgb':empty})
        frame=s['frame_prefix']+str(active)
        self.cloud_pub.publish(cloud_message(c['xyz'],c['rgb'],frame,stamp))
        m=next((m for m in s['maps'] if m['id']==active),{'keyframes':[]})
        self.path_pub.publish(path_message(m,frame,stamp))
        self.status({'state':'READY','version':s['version'],'active_map':active,'frame_id':frame,
                     'points':sum(len(c['xyz']) for c in clouds.values()),
                     'missing_keyframes':sum(len(c['missing']) for c in clouds.values()),
                     'preview_replaced':self.queue.replaced,'writer':s.get('writer',{})})

    def save(self,request,response):
        if not self.saving.acquire(blocking=False):
            response.success=False;response.message='Another save request is running';return response
        try:
            if not self.client.wait_for_service(timeout_sec=3.):raise RuntimeError('SLAM snapshot service unavailable')
            future=self.client.call_async(Trigger.Request());done=threading.Event();future.add_done_callback(lambda _:done.set())
            if not done.wait(30):raise RuntimeError('Snapshot service timed out; check tracking/disk writer')
            answer=future.result()
            if not answer.success:raise RuntimeError(answer.message)
            s=json.loads(answer.message)
            if Path(s['run_dir']).resolve()!=self.run:raise RuntimeError('Snapshot session mismatch')
            job={'snapshot':s,'event':threading.Event(),'path':None,'error':None}
            if not self.queue.put_save(job):raise RuntimeError('Save queue is full or shutting down')
            # Work runs in the single fusion thread. Executor threads remain available.
            job['event'].wait()
            if job['error']:raise RuntimeError(job['error'])
            response.success=True;response.message=str(job['path'])+job.get('warning','')
        except Exception as e:
            response.success=False;response.message=str(e)
        finally:self.saving.release()
        return response

    def work(self):
        while True:
            item=self.queue.get()
            if item is None:return
            kind,payload=item;s=payload['snapshot'] if kind=='save' else payload
            try:
                self.status({'state':'BUILDING','version':s['version'],'active_map':s['active_map']})
                clouds=rebuild(s,self.options)
                if kind=='save':
                    payload['path']=save_bundle(s,clouds,self.options)
                    missing=sum(len(c['missing']) for c in clouds.values())
                    if missing:payload['warning']=f' (partial coverage: {missing} missing RGB-D keyframes; see metadata.json)'
                    self.get_logger().info(f'Map saved: {payload["path"]}')
                self.publish(s,clouds)
            except Exception as e:
                if kind=='save':payload['error']=str(e)
                self.status({'state':'ERROR','version':s.get('version'),'error':str(e)})
                self.get_logger().error(f'Dense mapping: {e}')
            finally:
                if kind=='save':payload['event'].set()

    def finish(self):
        self.queue.stop();self.worker.join()
        if not self.auto_save:return True
        # ros2 launch sends SIGINT to both nodes. Wait for the tracking node to
        # drain the writer and emit its post-Shutdown snapshot before final fusion.
        final=self.run/'final_snapshot.json';deadline=time.monotonic()+self.final_wait
        print('Dense mapper: waiting for tracker final_snapshot.json',flush=True)
        while not final.exists() and time.monotonic()<deadline:time.sleep(.2)
        if not final.exists():
            print('FINAL SAVE FAILED: tracker did not produce a final snapshot; raw keyframes remain on disk',flush=True)
            return False
        try:
            s=load_snapshot(final)
            if not s['final']:raise ValueError('Expected final snapshot')
            output=save_bundle(s,rebuild(s,self.options),self.options)
            print(f'FINAL MAP SAVED: {output}',flush=True)
            return True
        except Exception as e:
            print(f'FINAL SAVE FAILED: {e}',flush=True)
            return False


def main():
    rclpy.init();node=DenseNode();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        executor.shutdown();saved=node.finish();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if not saved:raise SystemExit(1)


if __name__=='__main__':main()
