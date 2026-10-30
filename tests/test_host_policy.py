"""Physical-target bridge and per-cycle lineage, independent of ROS."""
import hashlib
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

from elf3_eval import host
from elf3_eval.contract import BODY


class Policy:
    def __init__(self):
        self.output = NS(joints=NS(position=np.linspace(-.2,.2,29,dtype=np.float32),
                                  layout=NS(names=tuple(reversed(BODY)))))


class Collector:
    errors = 0

    def __init__(self):
        self.events = []

    def capture(self, factory):
        self.events.append(factory())


class HostPolicyTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.model = Path(self.folder.name)/'dance.onnx'
        self.model.write_bytes(b'unit-test model identity')
        self.motion = self.model.with_suffix('.npz')
        self.motion.write_bytes(b'unit-test motion identity')
        self.collector = Collector()
        for target, value in (('_enabled', True), ('_tap', self.collector)):
            p = patch.object(host, target, value)
            p.start(); self.addCleanup(p.stop)
        host.begin_cycle()
        self.policy = Policy()

    def register(self):
        host.register_policy(self.policy, self.model, adapter='dance',
            policy_class='VerifiedPolicy', source_files=[self.motion], details={'variant':'mjlab'})

    def msg(self):
        return NS(header=NS(stamp=NS(sec=1,nanosec=5)))

    def test_exact_decoded_target_and_named_order_are_preserved(self):
        self.register()
        host.policy_output(self.policy, 1., True, reference_frame=4.5)
        e = self.collector.events[-1]
        np.testing.assert_array_equal(e['q'], self.policy.output.joints.position)
        self.assertEqual(e['names'],list(reversed(BODY)))
        self.assertEqual(e['model_hash'], hashlib.sha256(self.model.read_bytes()).hexdigest())
        self.assertEqual(e['reference_frame'],4.5)
        self.assertEqual(e['unit'],'rad')
        self.assertEqual(e['stage'],'post_host_decoder_pre_framework')
        self.assertEqual(len(e['source_artifacts']),1)

    def test_command_links_exact_sequence_and_next_cycle_clears_it(self):
        self.register();host.policy_output(self.policy,1.,True)
        host.command_published(self.msg(),'dance')
        self.assertEqual(self.collector.events[-1]['model_seq'],1)
        host.begin_cycle();host.command_published(self.msg(),'normal')
        self.assertIsNone(self.collector.events[-1]['model_seq'])
        self.assertNotIn('model_id', self.collector.events[-1])

    def test_preview_hold_and_transition_exclusion_are_explicit(self):
        self.register();host.policy_output(self.policy,1.,False)
        self.assertEqual(self.collector.events[-1]['phase'],'hold')
        host.set_transition(True);host.policy_output(self.policy,1.,True)
        self.assertEqual(self.collector.events[-1]['phase'],'blend')
        host.set_transition(False);host.command_published(self.msg(),'dance')
        self.assertEqual(self.collector.events[-1]['phase'],'blend')

    def test_observer_failure_never_changes_target(self):
        self.register();before=self.policy.output.joints.position.copy()
        with patch.object(self.collector,'capture',side_effect=RuntimeError('injected')):
            host.policy_output(self.policy,1.,True)
            host.command_published(self.msg(),'dance')
            host.test_event('dance','failed','injected')
        np.testing.assert_array_equal(before,self.policy.output.joints.position)
        self.assertEqual(self.collector.errors,3)

    def test_disabled_observer_does_not_read_model(self):
        with patch.object(host,'_enabled',False), patch.object(Path,'open',side_effect=AssertionError('opened')):
            host.register_policy(self.policy,self.model,adapter='dance',policy_class='P')
            host.policy_output(self.policy,1.,True)
        self.assertFalse(self.collector.events)

    def test_missing_model_never_registers_false_identity(self):
        host.register_policy(self.policy,self.model.with_name('missing'),adapter='dance',policy_class='P')
        host.policy_output(self.policy,1.,True)
        self.assertEqual(self.collector.errors,1)
        self.assertFalse(self.collector.events)

    def test_test_marker_does_not_claim_accuracy_pass(self):
        host.test_event('dance','playback_complete')
        self.assertEqual(self.collector.events[-1]['accuracy_acceptance'],
                         'not_assessed_by_playback_controller')
