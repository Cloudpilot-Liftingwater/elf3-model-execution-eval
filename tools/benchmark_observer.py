"""No motor/ROS: real local socket + bounded writer, synthetic 1000Hz feedback."""
import argparse,json,os,resource,socket,subprocess,sys,time,uuid
from pathlib import Path
import numpy as np
from elf3_eval.contract import BODY,angle_event,dump
from elf3_eval.tap import Tap

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=5);a=p.parse_args()
 a.output.mkdir(parents=True,exist_ok=True)
 address='elf3_eval_bench_'+uuid.uuid4().hex
 process=subprocess.Popen([sys.executable,'-m','elf3_eval','record','--socket-only','--address',address,'--seconds',str(a.seconds+1),'--output',str(a.output/'capture')],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
 probe=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
 for _ in range(100):
  try:probe.connect('\0'+address);break
  except OSError:time.sleep(.01)
 else:process.terminate();raise RuntimeError('recorder failed to start')
 probe.close()
 tap=Tap(address);durations=[];start=time.monotonic();count=int(a.seconds*1000)
 q=[.1]*29
 for i in range(count):
  deadline=start+i*.001;delay=deadline-time.monotonic()
  if delay>0:time.sleep(delay)
  now=time.monotonic()
  if i%20==0:
   tap.capture(lambda:angle_event('model',now,q,model_id='synthetic-benchmark',model_hash='f'*64,session='benchmark',seq=i//20,phase='active',position_target=True))
   tap.capture(lambda:angle_event('command',time.monotonic(),q,kp=[10.]*29))
  before=time.perf_counter_ns();tap.capture(lambda:angle_event('actual',time.monotonic(),q));durations.append(time.perf_counter_ns()-before)
 elapsed=time.monotonic()-start;tap.close();stdout,stderr=process.communicate(timeout=15)
 if process.returncode:raise RuntimeError(stderr+stdout)
 capture=json.loads((a.output/'capture/capture_result.json').read_text())
 absent=Tap('absent_'+uuid.uuid4().hex);calls=0
 def never():
  nonlocal calls
  calls+=1;return {}
 missing=[]
 for _ in range(20000):
  before=time.perf_counter_ns();absent.capture(never);missing.append(time.perf_counter_ns()-before)
 def stats(v):return dict(p50_us=float(np.percentile(v,50)/1000),p95_us=float(np.percentile(v,95)/1000),p99_us=float(np.percentile(v,99)/1000),max_us=float(max(v)/1000))
 result=dict(environment='workstation Linux, no ROS, no motors',feedback_hz_target=1000,model_hz_target=50,
  feedback_attempted=count,elapsed_s=elapsed,tap_sent=tap.sent,tap_dropped=tap.dropped,tap_errors=tap.errors,
  feedback_capture_overhead=stats(durations),absent_recorder_overhead=stats(missing),absent_factory_calls=calls,
  capture=capture,compressed_bytes=(a.output/'capture/events.jsonl.gz').stat().st_size,
  limits='Python workstation measurements, not a robot real-time guarantee; ROS callback cost not measured.')
 assert calls==0;assert tap.sent==capture['written'];assert not capture['queue_lost'];assert not capture['writer_errors']
 dump(a.output/'benchmark.json',result);print(json.dumps(result,indent=2))
if __name__=='__main__':main()
