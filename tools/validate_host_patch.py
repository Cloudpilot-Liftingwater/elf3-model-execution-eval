"""Apply the supplied patch only to temporary copies; compare source step behavior."""
import argparse,ast,hashlib,json,subprocess,tempfile,textwrap
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from elf3_eval.contract import dump

p=argparse.ArgumentParser();p.add_argument('package_root',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
root=Path(__file__).resolve().parents[1];manifest=json.loads((root/'patches/expected_sources.json').read_text())
with tempfile.TemporaryDirectory(prefix='elf3-patch-') as temp:
 dest=Path(temp)
 for rel,sha in manifest.items():
  data=(args.package_root/rel).read_bytes();assert hashlib.sha256(data).hexdigest()==sha
  target=dest/rel;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
 subprocess.run(['patch','--batch','--fuzz=0','-p1','-d',str(dest),'-i',str(root/'patches/underlay-normal-observer.patch')],check=True)
 for rel in manifest:compile((dest/rel).read_text(),rel,'exec')
 def method(file,cls,name,ns):
  tree=ast.parse(file.read_text());node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==cls)
  f=next(n for n in node.body if isinstance(n,ast.FunctionDef) and n.name==name)
  code='from __future__ import annotations\n'+textwrap.dedent(ast.get_source_segment(file.read_text(),f))
  exec(code,ns);return ns[name]
 class Broken:
  def __getattr__(self,name):raise RuntimeError('Injected observation helper failure')
 checks=[]
 for observer in (None,Broken()):
  namespace=dict(np=np,time=NS(monotonic=lambda:123.),_execution_eval=observer)
  step=method(dest/'bxi_example_py_elf3/policies/normal.py','NormalMotionPolicyMjlab','step',namespace)
  q=np.zeros(29,np.float32);raw=np.arange(29,dtype=np.float32)/29
  fake=NS(_action=np.zeros(29,np.float32),_action_checkpoint=np.zeros(29,np.float32),_scaled_action=np.zeros(29,np.float32),
   _target=q,_obs=np.zeros((1,96),np.float32),_gravity=np.zeros(3,np.float32),_inputs={},_backend=NS(run=lambda _:dict(actions=raw)),
   _parameters=NS(default_position=np.full(29,.4,np.float32),action_scale=np.full(29,.7,np.float32)),output=NS(position=q),
   bind_joints=lambda _:NS(position=np.zeros(29),velocity=np.zeros(29)),_project_gravity=lambda quat,out:out.fill(0))
  frame=NS(command=np.zeros(3),angular_velocity=np.zeros(3),quat_wxyz=np.array([1,0,0,0]))
  returned=step(fake,frame,.02);assert returned is fake.output
  np.testing.assert_array_equal(q,np.full(29,.4,np.float32)+raw*np.full(29,.7,np.float32))
  checks.append('Normal step unaffected: '+('no helper' if observer is None else 'helper raises'))
  ns=dict(_execution_eval=observer)
  update=method(dest/'bxi_example_py_elf3/framework/runtime/controller.py','RobotControlFramework','update',ns)
  motor=NS(qpos=q,kp=q,kd=q,vel=q,torque=q)
  fake=NS(_closed=False,current_cmd_vel=np.zeros(3),loop_count=0,_set_observation=lambda _:None,
   _apply_pending_state_requests=lambda:None,last_motor_frame=NS(update=lambda *a,**kw:None))
  def state_update(dt):fake._motor_target=motor
  fake.state_machine=NS(in_transition=False,update=lambda dt,events:False,update_current_state=state_update)
  assert update(fake,None,[],.02) is motor;assert fake.loop_count==1
  checks.append('Framework update unaffected: '+('no helper' if observer is None else 'helper raises'))
  # Final publish still uses the override result, irrespective of observer failures.
  published=[]
  class Msg:
   def __init__(self):self.header=NS()
  ns=dict(_execution_eval=observer,time=NS(monotonic=lambda:1.),robot_name='elf3',bxiMsg=NS(ActuatorCmds=Msg))
  publish=method(dest/'bxi_example_py_elf3/bxi_example_demo.py','BxiExample','publish_motor_frame',ns)
  overridden=NS(layout=NS(names=['joint']),qpos=np.array([.9]),vel=np.array([0.]),torque=np.array([0.]),kp=np.array([10.]),kd=np.array([1.]))
  fake=NS(runtime=NS(framework=NS(current_state_name='normal',state_machine=NS(in_transition=False))),
   motor_override_allow_in_zero_torque=False,_motor_override=NS(apply=lambda *a,**kw:overridden),
   get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS(sec=1,nanosec=1))),act_pub=NS(publish=published.append))
  publish(fake,motor);assert len(published)==1;assert published[0].pos==[.9]
  checks.append('Post-override publish unaffected: '+('no helper' if observer is None else 'helper raises'))
 dump(args.output,dict(status='passed_offline',checks=checks,patched_files=len(manifest),original_sources_modified=False))
 print('PASS:',len(checks),'source-method checks; patch applied only to temporary copies.')
