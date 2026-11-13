#!/usr/bin/env python3
"""Fuse recorded, rectified RGB-D using final current-map keyframe Twc poses.

No pose interpolation: only recorded images matching an exported keyframe are used.
Voxel centroids/colors are averaged. This is colored point fusion, not TSDF meshing.
"""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np


def rotation(q):
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid pose quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def project(depth, bgr, intr, pose, stride, near, far):
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride].astype(np.float64) / intr['depth_units_per_metre']
    valid = np.isfinite(z) & (z >= near) & (z <= far)
    xyz = np.column_stack(((u[valid]-intr['cx'])*z[valid]/intr['fx'],
                           (v[valid]-intr['cy'])*z[valid]/intr['fy'], z[valid]))
    # TUM keyframe poses are camera-to-world (Twc), not Tcw.
    xyz = xyz @ rotation(pose[4:8]).T + pose[1:4]
    rgb = bgr[::stride, ::stride][valid][:, ::-1].astype(np.float64)
    return xyz, rgb


def aggregate(xyz, rgb, size, counts=None):
    if counts is None:
        counts = np.ones(len(xyz))
    keys, inv = np.unique(np.floor(xyz/size).astype(np.int64), axis=0, return_inverse=True)
    n = np.bincount(inv, weights=counts, minlength=len(keys))
    p = np.column_stack([np.bincount(inv, weights=xyz[:,i]*counts, minlength=len(keys))/n for i in range(3)])
    c = np.column_stack([np.bincount(inv, weights=rgb[:,i]*counts, minlength=len(keys))/n for i in range(3)])
    return p, c, n


def write_ply(path, xyz, rgb):
    vertices = np.empty(len(xyz), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),
                                       ('red','u1'),('green','u1'),('blue','u1')])
    for i, name in enumerate(('x','y','z')):
        vertices[name] = xyz[:,i]
    for i, name in enumerate(('red','green','blue')):
        vertices[name] = np.clip(np.rint(rgb[:,i]), 0, 255).astype(np.uint8)
    temp = path.with_suffix('.ply.tmp')
    with temp.open('wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\n'
                 'comment ORB-SLAM3 final current-map keyframes; coordinates in metres\n'
                 f'element vertex {len(vertices)}\n'
                 'property float x\nproperty float y\nproperty float z\n'
                 'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode())
        vertices.tofile(f)
    temp.replace(path)


def fuse(run, voxel=0.01, stride=2, near=0.3, far=4.0):
    intr = json.loads((run/'intrinsics.json').read_text())
    if not intr.get('rectified') or any(not np.isfinite(intr[k]) or intr[k] <= 0
            for k in ('fx','fy','width','height','depth_units_per_metre')):
        raise ValueError('Invalid rectified-camera intrinsics')
    frames = [s.split() for s in (run/'frames.txt').read_text().splitlines()
              if s.strip() and not s.startswith('#')]
    if not frames:
        raise ValueError('No RGB-D frames recorded')
    times = np.array([float(r[0]) for r in frames])
    if np.any(np.diff(times) <= 0):
        raise ValueError('Recorded timestamps must increase strictly')
    path = run/'KeyFrameTrajectory_final.txt'
    if not path.exists() or not path.read_text().strip():
        raise ValueError('No final keyframe trajectory. Stop scanning with Ctrl+C and wait for clean shutdown.')
    poses = np.loadtxt(path, ndmin=2)
    if poses.shape[1] != 8 or not np.isfinite(poses).all():
        raise ValueError('Invalid TUM pose file')
    xyz = np.empty((0,3)); rgb = np.empty((0,3)); counts = np.empty(0)
    pending_p, pending_c = [], []
    used = skipped = 0

    def merge():
        nonlocal xyz, rgb, counts
        p, c = np.concatenate(pending_p), np.concatenate(pending_c)
        xyz, rgb, counts = aggregate(np.concatenate((xyz,p)), np.concatenate((rgb,c)),
                                    voxel, np.concatenate((counts,np.ones(len(p)))))
        pending_p.clear(); pending_c.clear()

    for pose in poses:
        idx = int(np.searchsorted(times, pose[0]))
        candidates = [j for j in (idx-1,idx) if 0 <= j < len(times)]
        idx = min(candidates, key=lambda j: abs(times[j]-pose[0]))
        # SaveKeyFrameTrajectoryTUM rounds timestamps to microseconds.
        if abs(times[idx]-pose[0]) > 2e-6:
            skipped += 1
            continue
        bgr = cv2.imread(str(run/frames[idx][1]), cv2.IMREAD_COLOR)
        depth = cv2.imread(str(run/frames[idx][2]), cv2.IMREAD_UNCHANGED)
        shape = (int(intr['height']),int(intr['width']))
        if bgr is None or depth is None or depth.dtype != np.uint16 or depth.shape != shape or bgr.shape[:2] != shape:
            raise ValueError(f'Missing or incompatible RGB-D pair: {frames[idx]}')
        p, c = project(depth,bgr,intr,pose,stride,near,far)
        if not len(p):
            skipped += 1
            continue
        pending_p.append(p); pending_c.append(c); used += 1
        if len(pending_p) == 10:
            merge()
            print(f'Integrated {used} keyframes, {len(xyz)} voxels', flush=True)
    if pending_p:
        merge()
    if not len(xyz):
        raise ValueError('No valid points; check tracking, RGB-D data and depth range')
    output = run/'dense_map.ply'
    write_ply(output,xyz,rgb)
    report = dict(keyframes_total=len(poses), keyframes_used=used, keyframes_skipped=skipped,
                  points=len(xyz), voxel_m=voxel, pixel_stride=stride, depth_range_m=[near,far],
                  pose_source='final active-map keyframes after shutdown',
                  method='voxel-averaged RGB-D points; no TSDF, mesh or statistical outlier removal')
    (run/'dense_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    print(f'Saved: {output}')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, nargs='?', help='Completed dense recording directory')
    parser.add_argument('--latest', action='store_true', help='Use latest outputs/dense_runs recording')
    parser.add_argument('--voxel', type=float, default=0.01, help='Voxel size in metres')
    parser.add_argument('--stride', type=int, default=2, help='Pixel sampling stride; 1 uses every pixel')
    parser.add_argument('--near', type=float, default=0.3)
    parser.add_argument('--far', type=float, default=4.0)
    args = parser.parse_args()
    if args.voxel <= 0 or args.stride < 1 or not (0 < args.near < args.far) or not np.isfinite([args.voxel,args.near,args.far]).all():
        parser.error('Require finite voxel>0, stride>=1 and 0<near<far')
    if args.latest:
        if args.run:
            parser.error('Choose a directory or --latest')
        runs = sorted((Path(__file__).resolve().parent/'dense_runs').glob('*/frames.txt'))
        if not runs:
            parser.error('No dense recording found; run the camera recorder first')
        args.run = runs[-1].parent
    if not args.run:
        parser.error('Provide a recording directory or --latest')
    try:
        fuse(args.run.resolve(),args.voxel,args.stride,args.near,args.far)
    except (ValueError,OSError) as exc:
        print(f'Cannot fuse: {exc}',file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
