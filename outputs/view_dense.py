#!/usr/bin/env python3
"""Lightweight rotatable preview of saved XYZRGB binary PLY maps."""
import argparse
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('ply', type=Path, nargs='?')
parser.add_argument('--latest', action='store_true')
parser.add_argument('--save', type=Path, help='Save a PNG instead of opening a window')
args = parser.parse_args()
if args.latest:
    base=Path(__file__).resolve().parent/'dense_runs'
    bundles=sorted(base.glob('*/maps/save_*/metadata.json'))
    if bundles:
        import json
        meta=json.loads(bundles[-1].read_text())
        mid=str(meta['active_map'])
        if not meta['maps'].get(mid,{}).get('points'):
            mid=max(meta['maps'],key=lambda m:meta['maps'][m]['points'])
        args.ply=bundles[-1].parent/meta['maps'][mid]['ply']
    else:
        files=sorted(base.glob('*/dense_map.ply'))
        if not files:parser.error('No saved point cloud found; finish scanning or call save_map first')
        args.ply=files[-1]
if not args.ply:
    parser.error('Provide a PLY path or --latest')
dtype = [('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')]
with args.ply.open('rb') as f:
    lines = []
    for _ in range(100):
        line = f.readline().decode('ascii').strip()
        lines.append(line)
        if line == 'end_header':
            break
    else:
        parser.error('Invalid PLY header')
    required = ['property float x','property float y','property float z',
                'property uchar red','property uchar green','property uchar blue']
    if 'format binary_little_endian 1.0' not in lines or [s for s in lines if s.startswith('property ')] != required:
        parser.error('Viewer accepts only the XYZRGB binary PLY produced by the dense mapper')
    count = int(next(s.split()[2] for s in lines if s.startswith('element vertex ')))
    points = np.fromfile(f,dtype=dtype,count=count)
if len(points) != count or count == 0:
    parser.error('Empty or truncated point cloud')
print(f'File: {args.ply}\nFull cloud: {count} points; preview displays at most 40000 points')
rng = np.random.default_rng(0)
if count > 40000:
    points = points[rng.choice(count,40000,replace=False)]
xyz = np.column_stack([points[k] for k in ('x','y','z')])
rgb = np.column_stack([points[k] for k in ('red','green','blue')])/255.0
import matplotlib
if args.save:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(111,projection='3d')
ax.scatter(*xyz.T,c=rgb,s=1,depthshade=False)
ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)',title='Dense RGB-D map (sampled preview)')
mid=(xyz.min(0)+xyz.max(0))/2
radius=max(float(np.ptp(xyz,axis=0).max())*0.55,0.01)
ax.set_xlim(mid[0]-radius,mid[0]+radius)
ax.set_ylim(mid[1]-radius,mid[1]+radius)
ax.set_zlim(mid[2]-radius,mid[2]+radius)
ax.set_box_aspect((1,1,1))
fig.tight_layout()
if args.save:
    fig.savefig(args.save,dpi=160)
    print(f'Preview: {args.save}')
else:
    plt.show()
