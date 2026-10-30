"""Independent error magnitudes, coverage gates, cancellation, and report integration."""
from copy import deepcopy
import json

import numpy as np
import pytest

from elf3_eval.acceptance import PROFILE, assess, grade_segment, validate, execution_status
from elf3_eval.analysis import analyze, Options
from elf3_eval.contract import BODY
from elf3_eval.storage import import_recording
from test_evaluation import fixture


def good():
    row=dict(mae_deg=1.,rmse_deg=2.,p95_abs_deg=3.,bias_deg=-1.,max_abs_deg=4.,time_coverage_fraction=1.)
    return dict(id=0,model_id='dance:model',model_hash='a'*64,phase='active',span_s=5.,
                inference_events=5000,independent_inference_events=250,
                comparisons={'actual_minus_model':{n:dict(row) for n in BODY}})


@pytest.mark.parametrize('metric',list(PROFILE['limits_deg']))
@pytest.mark.parametrize('offset,expected',[(0.,'screening_pass'),(.1,'screening_fail')])
def test_every_joint_is_checked_at_inclusive_limits(metric,offset,expected):
    segment=good()
    segment['comparisons']['actual_minus_model'][BODY[-1]][metric]=PROFILE['limits_deg'][metric]+offset
    assert grade_segment(segment,PROFILE)['status']==expected


@pytest.mark.parametrize('value',[-3.1,3.1])
def test_bias_both_signs(value):
    s=good();s['comparisons']['actual_minus_model'][BODY[0]]['bias_deg']=value
    assert grade_segment(s,PROFILE)['status']=='screening_fail'


@pytest.mark.parametrize('change',[{'span_s':1.9},{'independent_inference_events':49}])
def test_high_feedback_counts_do_not_replace_time_or_inference_count(change):
    s=good();s.update(change)
    assert grade_segment(s,PROFILE)['status']=='insufficient_data'


@pytest.mark.parametrize('metric,value',[('rmse_deg',None),('mae_deg',np.nan),('bias_deg',True),('rmse_deg',-1),('time_coverage_fraction',.94)])
def test_bad_or_missing_data_never_becomes_zero_error(metric,value):
    s=good();s['comparisons']['actual_minus_model'][BODY[0]][metric]=value
    assert grade_segment(s,PROFILE)['status']=='insufficient_data'


@pytest.mark.parametrize('phase',['blend','hold','unknown'])
def test_transition_is_excluded(phase):
    s=good();s['phase']=phase
    assert grade_segment(s,PROFILE)['status']=='not_assessed'


def test_missing_model_is_not_command_tracking():
    s=good();s['model_id']=None
    assert grade_segment(s,PROFILE)['status']=='not_assessed'


def test_joint_override_and_invalid_profiles():
    s=good();s['comparisons']['actual_minus_model'][BODY[0]]['rmse_deg']=7.
    p=deepcopy(PROFILE);p['joint_overrides']={BODY[0]:{'rmse_deg':8.}}
    assert grade_segment(s,p)['status']=='screening_pass'
    for key,value in [('minimum_time_coverage',1.1),('minimum_span_s',True),('minimum_inference_events',False),('id',{})]:
        p=deepcopy(PROFILE);p[key]=value
        with pytest.raises(ValueError):validate(p)
    with pytest.raises(ValueError):validate({'id':'missing'})


def test_early_return_never_counts_as_completed_execution():
    events=[dict(state='com.bxi.execution_suite/sonic06',status=status,session='s',t=t)
            for t,status in [(1.,'started'),(3.,'cancelled')]]
    r=assess({'segments':[good()],'test_events':events})
    assert r['status']=='screening_pass' and r['execution']['status']=='incomplete'
    assert r['automatic_control_action']=='none'
    events[-1]['status']='playback_complete'
    assert execution_status(events)['status']=='playback_complete'
    events.append(dict(events[0],t=10.))
    assert execution_status(events)['status']=='incomplete'


def test_recorder_loss_prevents_positive_overall_screen():
    r=assess({'segments':[good()],'data_integrity':{'recorder':{'queue_lost':{'state':2}}}})
    assert r['status']=='insufficient_data' and r['data_integrity_status']=='issues_observed'


def test_short_boundary_fragment_does_not_hide_a_qualified_failed_segment():
    short=good();short['span_s']=.01
    failed=good();failed['id']=1
    failed['comparisons']['actual_minus_model'][BODY[-1]]['rmse_deg']=6.
    r=assess({'segments':[short,failed]})
    assert r['status']=='screening_fail' and r['incomplete_segments']==[0]


@pytest.mark.parametrize('angle,expected',[(1.,'screening_pass'),(6.,'screening_fail')])
def test_real_import_analysis_and_html_with_known_degrees(tmp_path,angle,expected):
    events=fixture(n=151,offset=np.deg2rad(angle))
    events[0]['adapter']='dance'
    source=tmp_path/'events.jsonl';source.write_text(''.join(json.dumps(e)+'\n' for e in events))
    import_recording(source,tmp_path/'input.sqlite')
    result=analyze(tmp_path/'input.sqlite',tmp_path/'report',Options(
        acceptance_profile=PROFILE['id'],lag_max_ms=10,lag_step_ms=5))
    assert result['acceptance']['status']==expected
    assert result['segments'][0]['independent_inference_events']==151
    assert result['acceptance']['segments'][0]['joints'][BODY[0]]['values_deg']['rmse_deg']==pytest.approx(angle)
    assert '精度初筛' in (tmp_path/'report/report.html').read_text()
    assert json.loads((tmp_path/'report/report.json').read_text())['acceptance']['status']==expected
