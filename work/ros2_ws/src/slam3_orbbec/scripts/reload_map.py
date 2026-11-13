#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path as PathMsg
from dense_mapping import read_ply,load_snapshot
from ros_cloud import cloud_message,path_message


def main():
    parser=argparse.ArgumentParser(description='Reload a saved point-cloud bundle into ROS2 (does not resume SLAM)')
    parser.add_argument('bundle',type=Path);parser.add_argument('--map',type=int)
    args,ros_args=parser.parse_known_args()
    meta=json.loads((args.bundle/'metadata.json').read_text());s=load_snapshot(args.bundle/'snapshot.json')
    mid=args.map if args.map is not None else meta['active_map']
    if str(mid) not in meta['maps']:parser.error('Requested map is not present in this bundle')
    entry=meta['maps'][str(mid)];xyz,rgb=read_ply(args.bundle/entry['ply'])
    rclpy.init(args=ros_args);node=Node('orb_slam3_map_loader')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub=node.create_publisher(PointCloud2,'/orb_slam3/dense_map',qos)
    path_pub=node.create_publisher(PathMsg,'/orb_slam3/keyframe_path',qos)
    def publish():
        stamp=node.get_clock().now().to_msg()
        pub.publish(cloud_message(xyz,rgb,entry['frame_id'],stamp))
        m=next(m for m in s['maps'] if m['id']==mid)
        path_pub.publish(path_message(m,entry['frame_id'],stamp))
    node.create_timer(2.,publish);publish()
    print(f'Loaded {len(xyz)} points; RViz Fixed Frame: {entry["frame_id"]}; Ctrl+C to stop',flush=True)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
