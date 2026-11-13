import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import sys
import unittest
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'work/ros2_ws/src/slam3_orbbec/scripts'))
from dense_mapping import Options,WorkQueue,rebuild,save_bundle,load_snapshot,read_ply,rotation

class DenseTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        for name in ('rgb','depth'):(self.root/name).mkdir()
        self.intr=dict(width=4,height=4,fx=4,fy=4,cx=1.5,cy=1.5,depth_units_per_metre=1000,rectified=True)
        for key in ('a','b'):
            cv2.imwrite(str(self.root/'rgb'/f'{key}.png'),np.full((4,4,3),[10,20,230],dtype=np.uint8))
            cv2.imwrite(str(self.root/'depth'/f'{key}.png'),np.full((4,4),1000,dtype=np.uint16))
        def key(k,i):return dict(id=i,frame_id=i,key=k,timestamp=float(i),rgb=f'rgb/{k}.png',depth=f'depth/{k}.png',pose=[0,0,0,0,0,0,1])
        self.a=key('a',1);self.b=key('b',2)
        self.s=dict(schema=2,version=1,final=False,run_dir=str(self.root),active_map=0,frame_prefix='slam_map_',intrinsics=self.intr,
                    maps=[dict(id=0,change_index=0,keyframes=[self.a])],writer=dict(written=2,dropped=0,failed=0))
        self.opts=Options(stride=1,edge_threshold=0,outlier_neighbors=0)
    def tearDown(self):self.tmp.cleanup()
    def test_units_rgb_and_pose_direction(self):
        a=copy.deepcopy(self.s);a['maps'][0]['keyframes'][0]['pose']=[2,3,4,0,0,np.sqrt(.5),np.sqrt(.5)]
        c=rebuild(a,self.opts)[0]
        np.testing.assert_allclose(c['xyz'].mean(0),[2,3,5],atol=1e-6)
        np.testing.assert_array_equal(c['rgb'][0],[230,20,10])
    def test_loop_correction_rebuild_replaces_old_world_points(self):
        before=rebuild(self.s,self.opts)[0]['xyz']
        s=copy.deepcopy(self.s);s['version']=2;s['maps'][0]['keyframes'][0]['pose'][0]=5
        after=rebuild(s,self.opts)[0]['xyz']
        self.assertEqual(len(before),len(after));self.assertGreater(after[:,0].min(),4)
        np.testing.assert_allclose(after.mean(0)-before.mean(0),[5,0,0],atol=1e-6)
    def test_maps_culling_merge_and_reset(self):
        s=copy.deepcopy(self.s);s['maps'].append(dict(id=1,change_index=0,keyframes=[self.b]))
        self.assertEqual(set(rebuild(s,self.opts)),{0,1})
        s['maps'][1]['keyframes'][0]['pose'][0]=2
        s['maps']=[dict(id=1,change_index=1,keyframes=[self.a,s['maps'][1]['keyframes'][0]])]
        merged=rebuild(s,self.opts);self.assertEqual(set(merged),{1});self.assertEqual(len(merged[1]['xyz']),32)
        s['maps'][0]['keyframes']=[self.a]
        self.assertEqual(len(rebuild(s,self.opts)[1]['xyz']),16)
        s['maps']=[];self.assertEqual(rebuild(s,self.opts),{})
    def test_duplicate_map_membership_rejected(self):
        s=copy.deepcopy(self.s);s['maps'].append(dict(id=1,change_index=0,keyframes=[self.a]))
        with self.assertRaisesRegex(ValueError,'Duplicate keyframe'):rebuild(s,self.opts)
    def test_missing_images_and_memory_limit(self):
        (self.root/'depth/a.png').unlink()
        self.assertEqual(rebuild(self.s,self.opts)[0]['missing'],['a'])
        with self.assertRaisesRegex(ValueError,'No valid map'):save_bundle(self.s,rebuild(self.s,self.opts),self.opts)
        self.s['maps'][0]['keyframes']=[self.b]
        with self.assertRaisesRegex(ValueError,'max_voxels'):rebuild(self.s,Options(stride=1,max_voxels=2,outlier_neighbors=0))
    def test_queue_bounded_latest_and_save_priority(self):
        q=WorkQueue()
        for i in range(10000):q.put_preview(i)
        self.assertEqual(q.replaced,9999);self.assertTrue(q.put_save('save'));self.assertFalse(q.put_save('extra'))
        self.assertEqual(q.get(),('save','save'));self.assertEqual(q.get(),('preview',9999))
        q.stop();self.assertIsNone(q.get());self.assertFalse(q.put_preview('later'))
    def test_portable_save_reload_unique_names(self):
        clouds=rebuild(self.s,self.opts)
        one=save_bundle(self.s,clouds,self.opts);two=save_bundle(self.s,clouds,self.opts)
        self.assertNotEqual(one,two)
        (self.root/'rgb/a.png').unlink();(self.root/'depth/a.png').unlink()
        saved=load_snapshot(one/'snapshot.json');c=rebuild(saved,self.opts)
        xyz,rgb=read_ply(one/'map_0.ply')
        np.testing.assert_allclose(c[0]['xyz'],xyz);np.testing.assert_array_equal(c[0]['rgb'],rgb)
        self.assertEqual(json.loads((one/'metadata.json').read_text())['maps']['0']['points'],16)
        with self.assertRaises(OSError):save_bundle(saved,c,self.opts,one/'map_0.ply')
    def test_depth_edge_and_outlier_filter(self):
        depth=np.full((4,4),1000,np.uint16);depth[0,0]=3000
        cv2.imwrite(str(self.root/'depth/a.png'),depth)
        self.assertEqual(len(rebuild(self.s,Options(stride=1,outlier_neighbors=0))[0]['xyz']),12)
        self.assertEqual(len(rebuild(self.s,Options(stride=1,edge_threshold=0,outlier_radius=.01,outlier_neighbors=1))[0]['xyz']),0)

if __name__=='__main__':unittest.main()
