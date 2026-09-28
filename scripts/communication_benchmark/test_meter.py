import json,tempfile,unittest
from pathlib import Path
from meter import Totals,owner,publisher_bus_rows,native_logical_totals,aggregate_probe
class MeterTests(unittest.TestCase):
 def test_native_counter_layouts(self):
  self.assertEqual(list(publisher_bus_rows([[['/d',[[7,1617672,1617672,14,0]]]],[]])),[('/d',7,1617672,14)])
  self.assertEqual(list(publisher_bus_rows([1,'',[[['/d',100,[[7,100,5,True]]]],[],[]]])),[('/d',7,100,5)])
 def test_shared_directed_ros2_topics_have_exact_sender_fanout(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);(root/'tx').mkdir()
   (root/'communication-graphs.json').write_text(json.dumps([dict(wall_ns=10,topics=[['/r0/cslam/request',['/r0/node','/r1/node'],[0]],['/cslam/global',['/r0/node','/r1/node'],[0,1]]])]))
   (root/'communication-measured.json').write_text(json.dumps(dict(total_peer_payload_bytes=0,observer_errors={'ambiguous':2})))
   (root/'tx/1.tsv').write_text('20\t/r0/node\t/r0/cslam/request\t10\t0\n21\t/r1/node\t/r0/cslam/request\t20\t0\n22\t/r0/node\t/cslam/global\t30\t0\n23\t/r1/node\t/cslam/global\t40\t0\n')
   result=aggregate_probe(root)
   self.assertEqual(result['total_peer_payload_bytes'],90)
   self.assertTrue(result['complete_probe'])
 def test_tcp_copies_reconnects_and_robot_dedup(self):
  rows=[dict(publisher='/a/source',topic='/d',peer_node=node,serialized_body_bytes=size,native_messages=1) for node,size in [('/b/one',100),('/b/one',20),('/b/two',110),('/c/one',120)]]
  result=native_logical_totals(rows,'dcl')
  self.assertEqual(result['total_peer_payload_bytes'],240)
  self.assertEqual(result['native_all_peer_connection_bytes'],350)
 def test_distinct_remote_recipients_only(self):
  with tempfile.TemporaryDirectory() as d:
   m=Totals(d,'dcl');m.add('/a/test','/a/native',[0,1,1,2,None],100)
   v=m.save();self.assertEqual(v['total_peer_payload_bytes'],200);self.assertEqual(v['total_serialized_publication_bytes'],100)
   m.add('/a/test','/a/native',[0,None],800);self.assertEqual(m.save()['total_peer_payload_bytes'],200)
 def test_multiple_publishers_are_not_misassigned(self):
  with tempfile.TemporaryDirectory() as d:
   m=Totals(d,'swarm');m.add('/cslam/global_descriptors','/r0/node',[0,1,2],10);m.add('/cslam/global_descriptors','/r1/node',[0,1,2],30)
   v=m.save();self.assertEqual(v['total_peer_payload_bytes'],80);self.assertEqual(len(v['topics']),2)
 def test_unowned_observer_is_excluded(self):
  with tempfile.TemporaryDirectory() as d:
   m=Totals(d,'disco');m.add('/context/x','/observer',[0,1,2],300);self.assertEqual(m.save()['total_peer_payload_bytes'],0)
 def test_centralized_scope_is_explicit(self):
  with tempfile.TemporaryDirectory() as d:
   m=Totals(d,'gac');m.add('/laser_full_3','/plOdometryNode',['frontend','backend'],500)
   v=m.save();self.assertEqual(v['total_peer_payload_bytes'],500);self.assertIn('not native interrobot',v['boundary'])
 def test_owner_boundaries(self):
  for node,method,expected in [('/a/native','dcl',0),('/b/native','dcl',1),('/jackal2_mapFusion','disco',2),('/r10/cslam','swarm',10),('/rosout','swarm',None)]:self.assertEqual(owner(node,method),expected)
if __name__=='__main__':unittest.main()
