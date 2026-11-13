#!/usr/bin/env python3
"""ROS-independent bounded RGB-D reconstruction and versioned map persistence.
SPDX-License-Identifier: GPL-3.0-or-later
"""
import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import shutil
import threading
import time
import uuid

import cv2
import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class Options:
    voxel: float = 0.01
    stride: int = 2
    near: float = 0.3
    far: float = 4.0
    edge_threshold: float = 0.05
    outlier_radius: float = 0.04
    outlier_neighbors: int = 3
    max_voxels: int = 600000
    max_keyframes: int = 10000

    def validate(self):
        if not np.isfinite([self.voxel,self.near,self.far,self.edge_threshold,self.outlier_radius]).all():
            raise ValueError('Nonfinite mapping options')
        if self.voxel<=0 or self.stride<1 or not 0<self.near<self.far or self.edge_threshold<0 or self.outlier_radius<0 or self.outlier_neighbors<0 or self.max_voxels<1 or self.max_keyframes<1:
            raise ValueError('Invalid mapping options')


def rotation(q):
    q=np.asarray(q,dtype=float); n=np.linalg.norm(q)
    if not np.isfinite(n) or n<1e-9: raise ValueError('Invalid quaternion')
    x,y,z,w=q/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def safe_path(root, relative):
    root=Path(root).resolve(); path=(root/relative).resolve()
    if not path.is_relative_to(root): raise ValueError('Image path outside recording directory')
    return path


def frame_cloud(root, k, intr, options):
    rgb_path=safe_path(root,k['rgb']); depth_path=safe_path(root,k['depth'])
    if not rgb_path.is_file() or not depth_path.is_file(): return None
    color=cv2.imread(str(rgb_path),cv2.IMREAD_COLOR)
    depth=cv2.imread(str(depth_path),cv2.IMREAD_UNCHANGED)
    shape=(int(intr['height']),int(intr['width']))
    if color is None or depth is None or depth.dtype!=np.uint16 or depth.shape!=shape or color.shape[:2]!=shape:
        raise ValueError(f'Corrupt/incompatible RGB-D keyframe {k["key"]}')
    z=depth.astype(np.float32)/intr['depth_units_per_metre']
    valid=(z>=options.near)&(z<=options.far)&np.isfinite(z)
    # Reject depth discontinuities/flying pixels using a local 3x3 range test.
    if options.edge_threshold:
        low=cv2.erode(z,np.ones((3,3),np.uint8))
        high=cv2.dilate(z,np.ones((3,3),np.uint8))
        valid &= (high-low)<=options.edge_threshold
    v,u=np.mgrid[0:shape[0]:options.stride,0:shape[1]:options.stride]
    z=z[::options.stride,::options.stride]; valid=valid[::options.stride,::options.stride]
    xyz=np.column_stack(((u[valid]-intr['cx'])*z[valid]/intr['fx'],
                         (v[valid]-intr['cy'])*z[valid]/intr['fy'],z[valid])).astype(float)
    rgb=color[::options.stride,::options.stride][valid][:,::-1].astype(float)
    p=np.asarray(k['pose'],dtype=float)
    if p.shape!=(7,) or not np.isfinite(p).all(): raise ValueError('Invalid Twc pose')
    return xyz@rotation(p[3:]).T+p[:3],rgb


def aggregate(xyz,rgb,voxel,counts):
    keys,inv=np.unique(np.floor(xyz/voxel).astype(np.int64),axis=0,return_inverse=True)
    n=np.bincount(inv,weights=counts,minlength=len(keys))
    p=np.column_stack([np.bincount(inv,weights=xyz[:,i]*counts,minlength=len(keys))/n for i in range(3)])
    c=np.column_stack([np.bincount(inv,weights=rgb[:,i]*counts,minlength=len(keys))/n for i in range(3)])
    return p,c,n


def rebuild(snapshot, options=Options()):
    options.validate()
    if snapshot.get('schema')!=2: raise ValueError('Unsupported snapshot schema')
    intr=snapshot['intrinsics']
    if not intr.get('rectified') or any(not np.isfinite(intr[k]) or intr[k]<=0 for k in ('fx','fy','width','height','depth_units_per_metre')):
        raise ValueError('Invalid rectified intrinsics')
    if intr['width']*intr['height']>4000000: raise ValueError('Image dimensions exceed reconstruction memory limit')
    if sum(len(m['keyframes']) for m in snapshot['maps'])>options.max_keyframes:
        raise ValueError('Snapshot exceeds max_keyframes; split the scan or raise the explicit limit')
    result={}; total=0; seen=set()
    for m in snapshot['maps']:
        if m['id'] in result: raise ValueError('Duplicate map id')
        xyz=np.empty((0,3)); rgb=np.empty((0,3)); counts=np.empty(0)
        used=[]; missing=[]; no_depth=[]
        # Rebuild from immutable camera frames every revision, never append old
        # world points: moved, culled and merged keyframes therefore update correctly.
        for k in m['keyframes']:
            if k['key'] in seen: raise ValueError('Duplicate keyframe across maps; reject inconsistent snapshot')
            seen.add(k['key'])
            cloud=frame_cloud(snapshot['run_dir'],k,intr,options)
            if cloud is None: missing.append(k['key']);continue
            p,c=cloud
            if not len(p): no_depth.append(k['key']);continue
            xyz,rgb,counts=aggregate(np.concatenate((xyz,p)),np.concatenate((rgb,c)),options.voxel,
                                     np.concatenate((counts,np.ones(len(p)))))
            if total+len(xyz)>options.max_voxels:
                raise ValueError('max_voxels exceeded; use a larger voxel or smaller scan (no silent truncation)')
            used.append(k['key'])
        before=len(xyz)
        if len(xyz) and options.outlier_radius>0 and options.outlier_neighbors>0:
            tree=cKDTree(xyz)
            keep=np.concatenate([tree.query_ball_point(xyz[i:i+8192],options.outlier_radius,return_length=True)
                                  for i in range(0,len(xyz),8192)])>options.outlier_neighbors
            xyz=xyz[keep];rgb=rgb[keep]
        total+=len(xyz)
        result[m['id']]={'xyz':xyz.astype(np.float32),'rgb':np.clip(np.rint(rgb),0,255).astype(np.uint8),
                         'used':used,'missing':missing,'no_depth':no_depth,'before_outlier':before,
                         'change_index':m['change_index']}
    return result


PLY_DTYPE=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')])


def write_ply(path,xyz,rgb):
    data=np.empty(len(xyz),dtype=PLY_DTYPE)
    for i,k in enumerate(('x','y','z')): data[k]=xyz[:,i]
    for i,k in enumerate(('red','green','blue')): data[k]=rgb[:,i]
    with Path(path).open('wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\ncomment units metres; camera-to-world keyframe poses\n'
                 f'element vertex {len(data)}\nproperty float x\nproperty float y\nproperty float z\n'
                 'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode())
        data.tofile(f)


def read_ply(path):
    with Path(path).open('rb') as f:
        lines=[]
        for _ in range(100):
            line=f.readline().decode('ascii').strip();lines.append(line)
            if line=='end_header': break
        else: raise ValueError('Invalid PLY header')
        props=[l for l in lines if l.startswith('property ')]
        if 'format binary_little_endian 1.0' not in lines or props!=['property float x','property float y','property float z','property uchar red','property uchar green','property uchar blue']:
            raise ValueError('Expected XYZRGB binary PLY generated by this mapper')
        n=int(next(l.split()[2] for l in lines if l.startswith('element vertex ')))
        if not 0<=n<=10000000: raise ValueError('Invalid point count')
        a=np.fromfile(f,dtype=PLY_DTYPE,count=n)
        if len(a)!=n: raise ValueError('Truncated PLY')
        return np.column_stack([a[k] for k in ('x','y','z')]),np.column_stack([a[k] for k in ('red','green','blue')])


def save_bundle(snapshot,clouds,options,output_root=None):
    if not any(len(c['xyz']) for c in clouds.values()): raise ValueError('No valid map points to save')
    root=Path(output_root or Path(snapshot['run_dir'])/'maps');root.mkdir(parents=True,exist_ok=True)
    name=f'save_{time.time_ns()}_v{snapshot["version"]}_{uuid.uuid4().hex[:6]}'
    staging=root/('.'+name+'.partial');staging.mkdir()
    destination=root/name
    try:
        meta={'schema':2,'version':snapshot['version'],'final':snapshot['final'],'active_map':snapshot['active_map'],
              'frame_prefix':snapshot['frame_prefix'],'options':asdict(options),'writer':snapshot.get('writer',{}),
              'created_unix':time.time(),'maps':{},'pose_source':'map-lock snapshot of optimized keyframes; no cross-map identity transform'}
        source=Path(snapshot['run_dir'])
        for m in snapshot['maps']:
            c=clouds[m['id']]; mid=str(m['id'])
            write_ply(staging/f'map_{mid}.ply',c['xyz'],c['rgb'])
            with (staging/f'KeyFrameTrajectory_map_{mid}.txt').open('w') as f:
                for k in m['keyframes']:
                    f.write(' '.join(f'{x:.9f}' for x in [k['timestamp'],*k['pose']])+'\n')
            meta['maps'][mid]={k:v for k,v in c.items() if k not in ('xyz','rgb')}
            meta['maps'][mid].update(points=len(c['xyz']),frame_id=snapshot['frame_prefix']+mid,
                                     keyframes=len(m['keyframes']),ply=f'map_{mid}.ply')
            for k in m['keyframes']:
                for field in ('rgb','depth'):
                    src=safe_path(source,k[field]); dest=safe_path(staging,k[field])
                    if not src.exists(): continue
                    dest.parent.mkdir(parents=True,exist_ok=True)
                    if not dest.exists():
                        try: os.link(src,dest)
                        except OSError: shutil.copy2(src,dest)
        for name in ('intrinsics.json','camera.yaml','camera_info.txt','runtime_config.json'):
            if (source/name).exists(): shutil.copy2(source/name,staging/name)
        saved=dict(snapshot);saved['run_dir']='.'
        (staging/'snapshot.json').write_text(json.dumps(saved,indent=2)+'\n')
        meta['snapshot_sha256']=hashlib.sha256((staging/'snapshot.json').read_bytes()).hexdigest()
        (staging/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
        staging.rename(destination)
        return destination
    except BaseException:
        # Keep an explicitly marked partial bundle for diagnostics, never expose it as a completed save.
        raise


def load_snapshot(path):
    path=Path(path).resolve();s=json.loads(path.read_text())
    if s['run_dir']=='.': s['run_dir']=str(path.parent)
    return s


class WorkQueue:
    """One latest preview + one save job; the running job is separate and bounded."""
    def __init__(self):
        self.condition=threading.Condition();self.preview=None;self.save=None;self.stopped=False;self.replaced=0
    def put_preview(self,item):
        with self.condition:
            if self.stopped: return False
            if self.preview is not None:self.replaced+=1
            self.preview=item;self.condition.notify();return True
    def put_save(self,item):
        with self.condition:
            if self.stopped or self.save is not None:return False
            self.save=item;self.condition.notify();return True
    def get(self):
        with self.condition:
            self.condition.wait_for(lambda:self.stopped or self.preview is not None or self.save is not None)
            if self.save is not None:r=self.save;self.save=None;return 'save',r
            if self.stopped:return None
            r=self.preview;self.preview=None;return 'preview',r
    def stop(self):
        with self.condition:self.stopped=True;self.preview=None;self.condition.notify_all()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('snapshot',type=Path);p.add_argument('--output',type=Path)
    p.add_argument('--voxel',type=float,default=.01);p.add_argument('--stride',type=int,default=2)
    p.add_argument('--near',type=float,default=.3);p.add_argument('--far',type=float,default=4)
    a=p.parse_args();o=Options(voxel=a.voxel,stride=a.stride,near=a.near,far=a.far)
    s=load_snapshot(a.snapshot);print(save_bundle(s,rebuild(s,o),o,a.output))
