"""Optional host bridge used ONLY by the supplied, unapplied source patch.

Enabled explicitly at process startup via ELF3_EVAL_ENABLE=1. All public observer
functions contain their own exceptions. Normal model fingerprinting happens at
policy construction; inference/control callbacks perform no file access.
"""
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import uuid
import weakref
from .tap import Tap
from .contract import angle_event

_enabled=os.environ.get('ELF3_EVAL_ENABLE')=='1'
_tap=Tap(os.environ.get('ELF3_EVAL_SOCKET','elf3_model_eval_v1'))
_policies=weakref.WeakKeyDictionary();_local=threading.local()
_session=uuid.uuid4().hex

def register_normal(policy,model):
    if not _enabled:return
    try:
        paths=[Path(model)] if isinstance(model,(str,Path)) else [Path(a.path) for a in model.artifacts]
        fingerprints={}
        for path in paths:
            if path.is_file():
                h=hashlib.sha256()
                with path.open('rb') as f:
                    for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
                fingerprints[str(path)]=h.hexdigest()
        if not fingerprints:raise ValueError('no readable model artifact')
        digest=next(iter(fingerprints.values())) if len(fingerprints)==1 else hashlib.sha256(json.dumps(fingerprints,sort_keys=True).encode()).hexdigest()
        _policies[policy]=dict(model_id='NormalMotionPolicyMjlab:'+paths[0].stem,
                               model_hash=digest,artifacts=fingerprints,seq=0,
                               hash_scope='single_artifact' if len(fingerprints)==1 else 'artifact_manifest')
    except Exception:_tap.errors+=1

def begin_cycle():
    try:_local.latest=None;_local.blend=False;_local.cycle_had_transition=False
    except Exception:_tap.errors+=1

def set_transition(active):
    try:
        _local.blend=bool(active)
        _local.cycle_had_transition=getattr(_local,'cycle_had_transition',False) or bool(active)
    except Exception:_tap.errors+=1

def normal_output(policy,started,advance):
    if not _enabled:return
    try:
        m=_policies.get(policy)
        if m is None:return
        m['seq']+=1;end=time.monotonic()
        meta=dict(model_id=m['model_id'],model_hash=m['model_hash'],session=_session,seq=m['seq'])
        # A transition can evaluate a policy without applying that exact output.
        phase='blend' if getattr(_local,'blend',False) else 'active' if advance else 'hold'
        _local.latest=dict(meta,phase=phase)
        def factory():
            out=angle_event('model',end,policy.output.joints.position,
                names=policy.output.joints.layout.names,**meta,phase=phase,position_target=True,
                inference_start=started,inference_end=end,adapter='normal',
                stage='post_model_scale_offset_pre_framework',hash_scope=m['hash_scope'],
                model_artifacts=m['artifacts'],default_q=policy._parameters.default_position.tolist(),
                action_scale=policy._parameters.action_scale.tolist())
            return out
        _tap.capture(factory)
    except Exception:_tap.errors+=1

def command_published(msg,state,transition_active=False):
    if not _enabled:return
    try:
        end=time.monotonic();latest=getattr(_local,'latest',None)
        def factory():
            e=dict(schema=1,kind='command_link',t=end,received_monotonic=end,
                   source_stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                   state=state,phase='blend' if transition_active or getattr(_local,'cycle_had_transition',False) else (latest['phase'] if latest else 'unknown'),
                   association='same_control_cycle',model_seq=None)
            if latest:e.update(model_id=latest['model_id'],model_hash=latest['model_hash'],
                               model_session=latest['session'],model_seq=latest['seq'])
            return e
        _tap.capture(factory)
    except Exception:_tap.errors+=1
