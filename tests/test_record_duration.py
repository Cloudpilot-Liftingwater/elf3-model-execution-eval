from collections import Counter
import importlib
from types import SimpleNamespace as NS

import pytest

m=importlib.import_module('elf3_eval.record')


@pytest.mark.parametrize('value',[-1,86401,float('inf'),float('nan')])
def test_invalid_duration_rejected_before_recording(value):
    with pytest.raises(ValueError):m.recording_deadline(10,value)


@pytest.mark.parametrize('seconds,expected_waits',[(0,3),(1800,0)])
def test_until_stopped_continues_past_old_thirty_minute_limit(monkeypatch,tmp_path,seconds,expected_waits):
    class Stop:
        waits=0
        def is_set(self):return self.waits>=3
        def set(self):self.waits=3
        def wait(self,delay):self.waits+=1
    stop=Stop()
    class Sock:
        def bind(self,address):pass
        def setblocking(self,value):pass
        def recv(self,size):raise BlockingIOError
        def close(self):pass
    class Writer:
        def __init__(self,path):
            self.errors=[];self.accepted=Counter();self.dropped=Counter();self.written=0
        def close(self):pass
    clock=iter([0.,2050.])
    last=[0.]
    def now():
        last[0]=next(clock,last[0]);return last[0]
    monkeypatch.setattr(m,'time',NS(monotonic=now,time_ns=lambda:0))
    monkeypatch.setattr(m,'threading',NS(Event=lambda:stop))
    monkeypatch.setattr(m,'signal',NS(SIGINT=2,SIGTERM=15,signal=lambda *args:None))
    monkeypatch.setattr(m,'socket',NS(AF_UNIX=1,SOCK_DGRAM=2,socket=lambda *args:Sock(),gethostname=lambda:'test'))
    monkeypatch.setattr(m,'AsyncWriter',Writer)
    result=m.record(tmp_path,seconds=seconds,socket_only=True)
    assert stop.waits==expected_waits
    assert result['stop_reason']==('signal' if seconds==0 else 'duration_reached')
