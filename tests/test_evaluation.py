import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from elf3_eval.contract import BODY,angle_event,vector
from elf3_eval.math_core import metrics,causal_indices
from elf3_eval.storage import import_recording
from elf3_eval.analysis import analyze,Options
from elf3_eval.lag import estimate
from elf3_eval.adapters.normal import decode_action

def sample(kind,t,q=0.,**kw):
    q=np.full(29,q) if np.isscalar(q) else q
    if kind=='model':kw=dict(model_id='normal',model_hash='a'*64,session='s',seq=int(round(t*50)),phase='active',position_target=True,**kw)
    if kind=='command':kw.setdefault('kp',[10.]*29)
    return angle_event(kind,t,q,**kw)

def fixture(n=12,offset=.1,model=True):
    events=[]
    for i in range(n):
        t=1+i*.02
        if model:events.append(sample('model',t,0.))
        events.append(sample('command',t+.001,.02))
        events.append(sample('actual',t+.002,offset))
    return events

class EvaluationTests(unittest.TestCase):
    def run_case(self,events,**kw):
        folder=tempfile.TemporaryDirectory();self.addCleanup(folder.cleanup);root=Path(folder.name)
        source=root/'events.jsonl';source.write_text(''.join(json.dumps(e)+'\n' for e in events))
        meta=import_recording(source,root/'input.sqlite')
        opts=Options(command_age_ms=50,model_age_ms=50,feedback_gap_ms=50,lag_max_ms=10,lag_step_ms=5,**kw)
        result=analyze(root/'input.sqlite',root/'report',opts)
        self.assertEqual(source.read_text(),''.join(json.dumps(e)+'\n' for e in events))
        # Strict JSON and all required deliverables.
        json.loads((root/'report/report.json').read_text(),parse_constant=lambda x:self.fail(x))
        self.assertTrue((root/'report/report.html').exists());self.assertTrue((root/'report/aligned.npz').exists())
        return result,meta

    def test_known_physical_error(self):
        r,_=self.run_case(fixture())
        m=r['segments'][0]['comparisons']
        self.assertAlmostEqual(m['actual_minus_model']['waist_y_joint']['mae_deg'],np.degrees(.1))
        self.assertAlmostEqual(m['actual_minus_command']['waist_y_joint']['rmse_deg'],np.degrees(.08))
        self.assertAlmostEqual(m['command_minus_model']['waist_y_joint']['bias_deg'],np.degrees(.02))
        self.assertEqual(r['summary']['model_valid_body29_frames'],12)
        self.assertEqual(r['segments'][0]['inference_events'],12)

    def test_command_only_does_not_claim_model(self):
        r,_=self.run_case(fixture(model=False))
        self.assertEqual(r['summary']['model_valid_body29_frames'],0)
        self.assertEqual(r['summary']['command_valid_body29_frames'],12)
        self.assertIn('未观测',r['summary']['model_result'])
        self.assertIsNone(r['segments'][0]['comparisons']['actual_minus_model']['body29']['rmse_deg'])

    def test_head_joints_and_permutation(self):
        e=fixture()
        for x in e:
            x['names']=['head_y_joint']+list(reversed(BODY))+['head_z_joint']
            x['q']=[999.]+list(reversed(x['q']))+[-999.]
            if x['kind']=='command':x['kp']=[0.]+[10.]*29+[0.]
        r,_=self.run_case(e)
        self.assertEqual(r['summary']['model_valid_body29_frames'],12)
        self.assertAlmostEqual(r['segments'][0]['comparisons']['actual_minus_model']['body29']['mae_deg'],np.degrees(.1))

    def test_missing_joint_remains_missing(self):
        e=fixture()
        for x in e:
            if x['kind']=='actual':x['names']=x['names'][:-1];x['q']=x['q'][:-1]
        r,m=self.run_case(e)
        self.assertEqual(m['counts']['missing_joint_values'],12)
        self.assertEqual(r['segments'][0]['comparisons']['actual_minus_model'][BODY[-1]]['n'],0)
        self.assertEqual(r['segments'][0]['comparisons']['actual_minus_model'][BODY[0]]['n'],12)

    def test_duplicate_joint_names_rejected(self):
        e=fixture();e[0]['names'][1]=e[0]['names'][0]
        _,m=self.run_case(e)
        self.assertTrue(any(k.startswith('rejected:') for k in m['counts']))

    def test_nonfinite_joint_not_zero(self):
        e=fixture();e[2]['q'][0]=None
        r,_=self.run_case(e)
        self.assertEqual(r['segments'][0]['comparisons']['actual_minus_model'][BODY[0]]['n'],11)

    def test_normal_metadata_not_sonic(self):
        meta={'joint_names':','.join(BODY),'action_scale':str([2.]*29),'default_joint_pos':str([.5]*29)}
        np.testing.assert_allclose(decode_action(np.ones(29),meta),2.5)
        meta['joint_names']=','.join(reversed(BODY))
        with self.assertRaises(ValueError):decode_action(np.ones(29),meta)

    def test_native_normal_step_matches_adapter(self):
        # Execute the reviewed source step itself with deterministic backend output.
        from types import SimpleNamespace as NS
        ns=dict(np=np,InferenceFrame=object,PolicyOutput=object)
        source=Path(__file__).with_name('fixtures').joinpath('normal_step.py.txt').read_text()
        exec(source,ns)
        action=np.linspace(-3,3,29,dtype=np.float32)
        scale=np.linspace(.1,.8,29,dtype=np.float32);default=np.linspace(-.2,.5,29,dtype=np.float32)
        target=np.zeros(29,np.float32);before=action.copy()
        fake=NS(_action=np.zeros(29,np.float32),_action_checkpoint=np.zeros(29,np.float32),
            _scaled_action=np.zeros(29,np.float32),_target=target,_obs=np.zeros((1,96),np.float32),
            _parameters=NS(default_position=default,action_scale=scale),_gravity=np.zeros(3,np.float32),
            _inputs={},_backend=NS(run=lambda _:dict(actions=action)),output=NS(position=target),
            bind_joints=lambda _:NS(position=np.zeros(29),velocity=np.zeros(29)),
            _project_gravity=lambda q,out:out.fill(0))
        frame=NS(command=np.zeros(3),angular_velocity=np.zeros(3),quat_wxyz=np.array([1,0,0,0]))
        ns['step'](fake,frame,.02)
        meta=dict(joint_names=','.join(BODY),action_scale=scale.tolist(),default_joint_pos=default.tolist())
        np.testing.assert_array_equal(decode_action(action,meta),target)
        np.testing.assert_array_equal(action,before)

    def test_velocity_model_not_eligible(self):
        e=fixture()
        for x in e:
            if x['kind']=='model':x['position_target']=False
        r,_=self.run_case(e)
        self.assertEqual(r['summary']['model_valid_body29_frames'],0)
        self.assertEqual(r['summary']['command_valid_body29_frames'],12)

    def test_zero_kp_excluded(self):
        e=fixture()
        for x in e:
            if x['kind']=='command':x['kp']=[0.]*29
        r,_=self.run_case(e);self.assertEqual(r['summary']['command_valid_body29_frames'],0)

    def test_unknown_enable_not_assumed(self):
        e=fixture()
        for x in e:
            if x['kind']=='command':x.pop('kp')
        r,_=self.run_case(e);self.assertEqual(r['summary']['command_valid_body29_frames'],0)
        self.assertEqual(r['summary']['reasons']['enable_state_unknown'],12)

    def test_hold_and_blend_separate(self):
        e=fixture()
        for i,x in enumerate(e):
            if x['kind']=='model' and i>=18:x['phase']='hold'
        r,_=self.run_case(e)
        self.assertEqual(r['summary']['model_valid_body29_frames'],6)
        self.assertEqual([s['phase'] for s in r['segments']],['active','hold'])

    def test_model_switch_does_not_pair_old_command(self):
        e=fixture()
        for x in e[18:]:
            if x['kind']=='model':x['model_id']='second'
        r,_=self.run_case(e)
        self.assertEqual([s['model_id'] for s in r['segments']],['normal','second'])
        self.assertEqual(r['summary']['model_valid_body29_frames'],12)

    def test_same_id_new_session_segmented(self):
        e=fixture()
        for x in e[18:]:
            if x['kind']=='model':x['session']='restart'
        r,_=self.run_case(e);self.assertEqual(len(r['segments']),2)

    def test_explicit_sequence_wrong_rejected(self):
        e=fixture()
        for x in e:
            if x['kind']=='command':x.update(model_seq=999,model_session='s',model_id='normal',model_hash='a'*64)
        r,_=self.run_case(e);self.assertEqual(r['summary']['model_valid_body29_frames'],0)

    def test_explicit_sequence_correct(self):
        e=fixture()
        for i in range(0,len(e),3):
            e[i+1].update(model_seq=e[i]['seq'],model_session='s',model_id='normal',model_hash='a'*64)
        r,_=self.run_case(e);self.assertEqual(r['summary']['model_valid_body29_frames'],12)

    def test_command_link_exact_stamp(self):
        e=fixture(1);e[1]['source_stamp_ns']=10000
        e.append(dict(schema=1,kind='command_link',t=1.003,received_monotonic=1.003,source_stamp_ns=10000,
                      model_id='normal',model_hash='a'*64,model_session='s',model_seq=e[0]['seq'],
                      association='same_control_cycle',phase='active'))
        r,_=self.run_case(e)
        self.assertEqual(r['segments'][0]['lineage']['explicit_sequence'],1)
        self.assertEqual(r['summary']['model_valid_body29_frames'],1)

    def test_no_model_in_cycle_no_old_model_fallback(self):
        e=fixture(1);e[1]['source_stamp_ns']=10000
        e.append(dict(schema=1,kind='command_link',t=1.003,received_monotonic=1.003,source_stamp_ns=10000,
                      model_seq=None,association='same_control_cycle',phase='unknown'))
        r,_=self.run_case(e)
        self.assertEqual(r['summary']['model_valid_body29_frames'],0)
        self.assertEqual(r['summary']['command_valid_body29_frames'],1)

    def test_explicit_disabled_feedback(self):
        e=fixture(1);e[-1]['enabled']=False
        r,_=self.run_case(e);self.assertEqual(r['summary']['command_valid_body29_frames'],0)

    def test_future_target_not_used(self):
        e=fixture(1);e=[e[1],e[2],e[0]];e[-1]['t']=1.01;e[-1]['received_monotonic']=1.01
        r,_=self.run_case(e);self.assertEqual(r['summary']['model_valid_body29_frames'],0)

    def test_stale_command_excluded(self):
        e=fixture(1);e[-1]['t']=2.;e[-1]['received_monotonic']=2.
        r,_=self.run_case(e);self.assertEqual(r['summary']['command_valid_body29_frames'],0)

    def test_sequence_gap_reported(self):
        e=fixture();e=[x for x in e if not(x['kind']=='model' and x['seq']==53)]
        _,m=self.run_case(e);self.assertEqual(m['counts']['model_sequence_missing'],1)

    def test_clock_jump_splits(self):
        e=fixture()
        for i,x in enumerate(e):x['ros_now_ns']=int((x['received_monotonic']+1000+(2 if i>=18 else 0))*1e9)
        r,m=self.run_case(e);self.assertEqual(m['clock_epochs'],2)
        self.assertEqual(len(r['segments']),2)

    def test_out_of_order_retained_flagged(self):
        e=fixture();x=dict(e[5]);x['received_monotonic']=e[-1]['t']+.001;e.append(x)
        _,m=self.run_case(e);self.assertEqual(m['counts']['out_of_order:actual'],1)
        self.assertEqual(m['counts']['stored:actual'],13)

    def test_duplicate_frame_retained_flagged(self):
        e=fixture();x=dict(e[-1]);x['received_monotonic']+=.001;e.append(x)
        _,m=self.run_case(e);self.assertEqual(m['counts']['duplicate_timestamp:actual'],1)

    def test_unit_degrees_not_accepted(self):
        e=fixture();e[0]['unit']='deg'
        _,m=self.run_case(e);self.assertEqual(m['counts']['rejected:only physical radians are accepted'],1)

    def test_lag_known_80ms(self):
        t=np.arange(0,6,.005);q=30*np.sin(t*3)+7*np.sin(t*8)
        actual=30*np.sin((t-.08)*3)+7*np.sin((t-.08)*8)
        r=estimate(t,q,actual,np.ones(len(t),bool),max_lag=.2,step=.005,max_gap=.01)
        self.assertAlmostEqual(r['estimated_lag_ms'],80,delta=5)
        self.assertGreater(r['zero_lag_same_samples_rmse_deg'],r['compensated_rmse_deg'])

    def test_lag_insufficient_motion(self):
        t=np.arange(0,5,.01)
        r=estimate(t,t*0,t*0,np.ones(len(t),bool),max_lag=.2,step=.005,max_gap=.02)
        self.assertEqual(r['status'],'insufficient_motion')

    def test_metrics_all_missing_not_zero(self):
        self.assertIsNone(metrics([np.nan])['mae_deg'])

    def test_causal_indices(self):
        np.testing.assert_array_equal(causal_indices([1.,2.],[.9,1.,1.5,2.01],.1),[-1,0,-1,1])

if __name__=='__main__':unittest.main()
