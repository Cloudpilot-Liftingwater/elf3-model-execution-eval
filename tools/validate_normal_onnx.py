"""Optional real-ONNX offline check. Encoder feedback below is SYNTHETIC."""
import argparse,ast,hashlib,json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from elf3_eval.adapters.normal import decode_action
from elf3_eval.contract import BODY,angle_event,dump
from elf3_eval.storage import import_recording
from elf3_eval.analysis import analyze

def main():
 p=argparse.ArgumentParser();p.add_argument('model',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 import onnxruntime as ort
 o=ort.SessionOptions();o.intra_op_num_threads=1;o.inter_op_num_threads=1
 backend=ort.InferenceSession(str(a.model),sess_options=o,providers=['CPUExecutionProvider'])
 meta=backend.get_modelmeta().custom_metadata_map
 default=np.array(ast.literal_eval(meta['default_joint_pos']),np.float32);scale=np.array(ast.literal_eval(meta['action_scale']),np.float32)
 ns=dict(np=np,InferenceFrame=object,PolicyOutput=object)
 source=Path(__file__).resolve().parents[1]/'tests/fixtures/normal_step.py.txt';exec(source.read_text(),ns)
 q=np.zeros(29,np.float32);obs=np.zeros((1,96),np.float32)
 raw=[]
 def run(inputs):
  action=backend.run(['actions'],inputs)[0];raw.append(action.copy());return dict(actions=action)
 joints=NS(position=default.copy(),velocity=np.zeros(29,np.float32))
 policy=NS(_action=np.zeros(29,np.float32),_action_checkpoint=np.zeros(29,np.float32),_scaled_action=np.zeros(29,np.float32),
  _target=q,_obs=obs,_gravity=np.zeros(3,np.float32),_inputs={'obs':obs},_backend=NS(run=run),
  _parameters=NS(default_position=default,action_scale=scale),output=NS(position=q),bind_joints=lambda _:joints,
  _project_gravity=lambda quat,out:np.copyto(out,[0,0,-1]))
 frame=NS(command=np.zeros(3,np.float32),angular_velocity=np.zeros(3,np.float32),quat_wxyz=np.array([1,0,0,0]))
 digest=hashlib.sha256(a.model.read_bytes()).hexdigest();events=[];maximum=0.
 for i in range(200):
  frame.command[0]=.1*np.sin(i/20);joints.position[:]=default+.01*np.sin(i/10)
  ns['step'](policy,frame,.02)
  decoded=decode_action(raw[-1],meta);maximum=max(maximum,float(np.max(np.abs(decoded-q))))
  t=100+i*.02
  events.append(angle_event('model',t,q,model_id='NormalMotionPolicyMjlab:real_onnx_offline',model_hash=digest,session='offline-normal',seq=i,phase='active',position_target=True,adapter='normal',evidence='real ONNX, synthetic observations'))
  events.append(angle_event('command',t+.0001,q,model_id='NormalMotionPolicyMjlab:real_onnx_offline',model_hash=digest,model_session='offline-normal',model_seq=i,kp=[10.]*29))
  events.append(angle_event('actual',t+.001,q.astype(float)+.03,evidence='SYNTHETIC encoder; not a robot run'))
 assert maximum==0.,maximum
 a.output.mkdir(parents=True,exist_ok=True)
 path=a.output/'events.jsonl';path.write_text(''.join(json.dumps(e)+'\n' for e in events))
 import_recording(path,a.output/'recording.sqlite');r=analyze(a.output/'recording.sqlite',a.output/'analysis')
 mae=r['segments'][0]['comparisons']['actual_minus_model']['body29']['mae_deg']
 assert abs(mae-np.degrees(.03))<1e-7
 dump(a.output/'validation.json',dict(status='passed_offline',real_model=str(a.model.resolve()),model_sha256=digest,
  onnxruntime=ort.__version__,onnx_inferences=len(raw),source_step_adapter_max_abs_rad=maximum,
  metadata=meta,synthetic_encoder_offset_rad=.03,recovered_mae_deg=mae,
  field_status='NOT TESTED ON ROBOT',note='Actual model and reviewed source step executed; all observations, commands and encoder feedback here are synthetic.'))
 print('PASS: 200 real ONNX inferences; source/adapter exact agreement; synthetic 0.03 rad error recovered.')
if __name__=='__main__':main()
