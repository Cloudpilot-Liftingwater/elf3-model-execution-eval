"""NormalMotionPolicyMjlab: runtime model metadata, never SONIC constants."""
import ast
import numpy as np
from ..contract import BODY, vector

def decode_action(action, metadata):
    names = tuple(n.strip() for n in metadata['joint_names'].split(','))
    if names != BODY:
        raise ValueError('NormalMotionPolicyMjlab requires ELF3_POLICY_JOINTS order')
    def values(key):
        x = metadata[key]
        x = ast.literal_eval(x) if isinstance(x, str) else x
        a = np.asarray(x, dtype=np.float32)
        if a.shape != (29,) or not np.isfinite(a).all():
            raise ValueError('invalid Normal metadata: '+key)
        return a
    raw = np.asarray(action, dtype=np.float32).reshape(-1)
    if raw.shape != (29,) or not np.isfinite(raw).all():
        raise ValueError('invalid Normal action')
    # Same float32 multiply then add as policies/normal.py step(). No action clip there.
    q = values('default_joint_pos') + raw * values('action_scale')
    return vector(q, names)
