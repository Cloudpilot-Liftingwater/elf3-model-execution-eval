import gzip
import json
from pathlib import Path
import socket
import tempfile
import time
import unittest
from unittest.mock import patch
import uuid
from elf3_eval.tap import Tap
from elf3_eval.record import AsyncWriter

class ObserverTests(unittest.TestCase):
    def test_no_listener_does_not_construct_payload(self):
        tap=Tap('test_'+uuid.uuid4().hex)
        for _ in range(100):
            self.assertFalse(tap.capture(lambda:self.fail('serialized without listener')))
        self.assertEqual(tap.errors,0)

    def test_full_receiver_queue_drops_without_blocking(self):
        name='test_'+uuid.uuid4().hex
        rx=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);rx.bind('\0'+name)
        self.addCleanup(rx.close);tap=Tap(name);self.addCleanup(tap.close)
        start=time.monotonic()
        for _ in range(1000):tap.capture(lambda:{'kind':'test','q':[0.]*29})
        self.assertGreater(tap.dropped,0)
        self.assertLess(time.monotonic()-start,2.)

    def test_factory_failure_does_not_escape(self):
        name='test_'+uuid.uuid4().hex
        rx=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);rx.bind('\0'+name)
        self.addCleanup(rx.close);tap=Tap(name);self.addCleanup(tap.close)
        def bad():raise RuntimeError('injected observer error')
        self.assertFalse(tap.capture(bad));self.assertEqual(tap.errors,1)

    def test_payload_roundtrip(self):
        name='test_'+uuid.uuid4().hex
        rx=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);rx.bind('\0'+name);rx.settimeout(1)
        self.addCleanup(rx.close);tap=Tap(name);self.addCleanup(tap.close)
        tap.capture(lambda:{'kind':'model','q':[.1]*29})
        got=json.loads(rx.recv(65536));self.assertEqual(got['q'],[.1]*29)

    def test_writer_flush_preserves_raw_fields(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'events.jsonl.gz';w=AsyncWriter(p)
            for i in range(100):w.submit(dict(kind='actual',q=[.1]*31,source_stamp_ns=i,received_monotonic=float(i)))
            w.close();self.assertFalse(w.errors)
            with gzip.open(p,'rt') as f:rows=[json.loads(x) for x in f]
            self.assertEqual(len(rows),100);self.assertEqual(rows[-1]['source_stamp_ns'],99)

    def test_writer_disk_failure_no_shutdown_deadlock(self):
        class Broken:
            def write(self,_):raise OSError('disk full injected')
            def close(self):pass
        with patch('elf3_eval.record.gzip.open',return_value=Broken()):
            w=AsyncWriter('unused');w.submit(dict(kind='actual'))
            start=time.monotonic();w.close()
            self.assertTrue(w.errors);self.assertLess(time.monotonic()-start,2)

    def test_writer_capacity_bounded(self):
        import threading
        release=threading.Event()
        class Slow:
            def write(self,_):release.wait(1)
            def close(self):pass
        with patch('elf3_eval.record.gzip.open',return_value=Slow()):
            w=AsyncWriter('unused',capacity=2)
            for _ in range(50):w.submit(dict(kind='actual'))
            self.assertLessEqual(w.queue.qsize(),2);self.assertGreater(w.dropped['actual'],0)
            release.set();w.close()

    def test_mode_json_parsed_on_writer(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'x.gz';w=AsyncWriter(p)
            w.submit(dict(kind='mode',state_json=json.dumps({'current':{'name':'normal'},'transition':{'active':True}})))
            w.close()
            with gzip.open(p,'rt') as f:r=json.loads(f.readline())
            self.assertEqual(r['state'],'normal');self.assertEqual(r['phase'],'blend')

    def test_ros_message_callbacks_with_fake_ros_no_publication(self):
        from types import ModuleType,SimpleNamespace as NS
        from elf3_eval.record import record
        from elf3_eval.contract import BODY
        subscriptions=[];sent=[]
        class State:pass
        class Command:pass
        class String:pass
        class Node:
            def get_clock(self):return NS(now=lambda:NS(nanoseconds=time.monotonic_ns()+10**12))
            def create_subscription(self,cls,topic,callback,qos):subscriptions.append((cls,topic,callback))
            def destroy_node(self):pass
            def create_publisher(self,*a,**kw):raise AssertionError('recorder must not publish')
        node=Node();rclpy=ModuleType('rclpy');rclpy.init=lambda:None;rclpy.ok=lambda:True
        rclpy.shutdown=lambda:None;rclpy.create_node=lambda *a,**kw:node
        def spin(*a,**kw):
            if sent:return
            sent.append(True)
            for cls,topic,callback in subscriptions:
                stamp=time.monotonic_ns()+10**12-100000
                msg=NS(header=NS(stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9),frame_id='elf3'))
                names=list(BODY)+['head_y_joint','head_z_joint']
                if cls is State:
                    msg.name=names;msg.position=[.1]*31;msg.velocity=[0.]*31;msg.effort=[0.]*31
                elif cls is Command:
                    msg.actuators_name=names;msg.pos=[.2]*31;msg.vel=[0.]*31;msg.kp=[10.]*31;msg.kd=[1.]*31;msg.torque=[0.]*31
                else:msg.data=json.dumps({'current':{'name':'normal'},'transition':None})
                callback(msg)
        rclpy.spin_once=spin
        qos=ModuleType('rclpy.qos');qos.QoSProfile=lambda **kw:kw;qos.ReliabilityPolicy=NS(BEST_EFFORT=1);qos.DurabilityPolicy=NS(VOLATILE=1)
        communication=ModuleType('communication.msg');communication.ActuatorStates=State;communication.ActuatorCmds=Command
        std=ModuleType('std_msgs.msg');std.String=String
        with tempfile.TemporaryDirectory() as d,patch.dict('sys.modules',{'rclpy':rclpy,'rclpy.qos':qos,'communication.msg':communication,'std_msgs.msg':std}):
            result=record(d,seconds=.02,address='test_'+uuid.uuid4().hex)
            self.assertEqual(result['hardware_commands_sent'],0);self.assertEqual(result['written'],3)
            with gzip.open(Path(d)/'events.jsonl.gz','rt') as f:rows=[json.loads(x) for x in f]
            self.assertEqual(rows[0]['value']['frame_id'],'elf3')
            self.assertEqual(len(rows[0]['value']['names']),31)
            self.assertEqual(rows[1]['value']['position'],[.2]*31)

if __name__=='__main__':unittest.main()
