import json
from types import SimpleNamespace as NS

from elf3_eval import host
from elf3_eval.adapters.legacy import convert
from elf3_eval.contract import BODY
from elf3_eval.diagnostics import TemperatureSummary, summarize_events
from elf3_eval.storage import import_recording, open_db, load_epoch


def test_real_recorder_callbacks_save_motor_and_driver_temperatures(tmp_path,monkeypatch):
    """Replace transport only; execute the real recorder callback and writer."""
    import gzip
    import importlib
    import sys
    import time
    from types import ModuleType
    recorder=importlib.import_module('elf3_eval.record')
    class Datagram:
        def bind(self,address):pass
        def setblocking(self,value):pass
        def recv(self,size):raise BlockingIOError
        def close(self):pass
    monkeypatch.setattr(recorder.socket,'socket',lambda *a,**k:Datagram())
    callbacks=[];sent=[]
    class Node:
        def get_clock(self):return NS(now=lambda:NS(nanoseconds=time.monotonic_ns()+10**12))
        def create_subscription(self,cls,topic,callback,qos):callbacks.append((topic,callback))
        def destroy_node(self):pass
        def create_publisher(self,*a,**kw):raise AssertionError('passive recorder published')
    ros=ModuleType('rclpy');ros.init=lambda:None;ros.shutdown=lambda:None;ros.ok=lambda:True
    ros.create_node=lambda *a,**kw:Node()
    def spin(*a,**kw):
        if sent:return
        sent.append(True)
        ns=time.monotonic_ns()+10**12-100000
        message=NS(header=NS(stamp=NS(sec=ns//10**9,nanosec=ns%10**9),frame_id='elf3'),
                   name=['head_z_joint',*reversed(BODY),'head_y_joint'],position=[0.]*31,
                   velocity=[0.]*31,effort=[0.]*31,motor_temperature=[40.]*31,driver_temperature=[45.]*31)
        next(cb for topic,cb in callbacks if topic.endswith('actuator_states'))(message)
    ros.spin_once=spin
    qos=ModuleType('rclpy.qos');qos.QoSProfile=lambda **kw:kw
    qos.ReliabilityPolicy=NS(BEST_EFFORT=1);qos.DurabilityPolicy=NS(VOLATILE=1)
    messages=ModuleType('communication.msg');messages.ActuatorStates=type('State',(),{});messages.ActuatorCmds=type('Command',(),{})
    std=ModuleType('std_msgs.msg');std.String=type('String',(),{})
    for name,value in {'rclpy':ros,'rclpy.qos':qos,'communication.msg':messages,'std_msgs.msg':std}.items():monkeypatch.setitem(sys.modules,name,value)
    result=recorder.record(tmp_path,seconds=.02)
    assert result['callback_errors']=={} and result['hardware_commands_sent']==0
    with gzip.open(tmp_path/'events.jsonl.gz','rt') as stream:raw=json.loads(next(stream))
    assert raw['value']['motor_temperature']==[40.]*31
    assert raw['value']['driver_temperature']==[45.]*31
    assert raw['value']['names'][0]=='head_z_joint'


def test_named31_temperatures_survive_conversion_and_storage(tmp_path):
    names=['head_z_joint',*reversed(BODY),'head_y_joint']
    raw=dict(kind='state',received_monotonic=1.,ros_now_ns=10**9,
        value=dict(stamp_ns=10**9,names=names,position=[0.]*31,
                   motor_temperature=list(range(20,51)),driver_temperature=list(range(30,61))))
    event=convert(raw)
    summary=TemperatureSummary();summary.add(event)
    r=summary.report({})
    assert r['status']=='observed' and r['by_joint']['motor_temperature']['head_z_joint']['maximum_c']==20
    source=tmp_path/'events.jsonl';source.write_text(json.dumps(raw)+'\n')
    import_recording(source,tmp_path/'index.sqlite')
    db=open_db(tmp_path/'index.sqlite',readonly=True)
    restored=list(load_epoch(db,0))[0];db.close()
    assert len(restored['names'])==29 and list(restored['temperature_names'])==names
    summary=TemperatureSummary();summary.add(restored)
    assert summary.report({})['by_joint']==r['by_joint']
    assert summary.report({'metadata.json':{'topics':{'state':'/simulation/actuator_states'}}})['status']=='not_assessed_simulation'


def test_absent_temperature_is_explicit():
    s=TemperatureSummary();s.add(dict(kind='actual',names=BODY))
    assert s.report({})['status']=='unobserved'


def test_structured_faults_and_old_event_signatures(monkeypatch):
    events=[]
    monkeypatch.setattr(host,'_enabled',True)
    monkeypatch.setattr(host,'_tap',NS(errors=0,capture=lambda factory:events.append(factory())))
    host.test_event('test','failed','old signature')
    host.test_event('test','failed','overheat',code='MOTOR_OVER_TEMPERATURE',category='temperature',
                    details={'joint':BODY[0],'observed_c':75.,'threshold_c':75.,'invalid':float('nan')})
    host.test_event('test','cancelled','operator')
    assert [e['severity'] for e in events]==['error','error','warning']
    assert events[1]['details']['invalid'] is None
    assert events[1]['details']['observed_c']==75.
    assert summarize_events(events)['by_category']['temperature']==1
    json.dumps(events,allow_nan=False)


def test_timing_warnings_remain_warnings_in_summary():
    event=dict(state='dance',status='warning',severity='warning',code='CONTROL_TIMING',category='timing')
    result=summarize_events([event])
    assert result['by_severity']=={'warning':1} and result['by_category']=={'timing':1}
