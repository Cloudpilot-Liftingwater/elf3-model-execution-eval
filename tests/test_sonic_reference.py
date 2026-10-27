import base64
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from elf3_eval.adapters.legacy import convert
from elf3_eval.adapters.stk1 import ARRAYS, SIZES
from elf3_eval.analysis import analyze, Options
from elf3_eval.contract import BODY, angle_event
from elf3_eval.sonic_reference import PACKING, SonicReference, unpack, valid_token
from elf3_eval.storage import import_recording, load_epoch, open_db


POLICY_HASH = 'a' * 64


def pack_window(q, dq):
    # Build the training wire layout from independent angle/velocity arrays.
    raw = np.zeros((len(q), 10, 64), np.float32)
    for i in range(len(q)):
        wire = list(q[i].flat) + list(dq[i].flat)
        raw[i, :, :58] = np.asarray(wire).reshape(10, 58)
    return raw


class FakeSession:
    def __init__(self):
        self.calls = 0
        self.fail = False
        self.sine = False

    def run(self, _, inputs):
        self.calls += 1
        if self.fail:
            raise RuntimeError('synthetic decoder failure')
        token = inputs['motion_token'].reshape(-1, 64)
        n = len(token)
        q = np.broadcast_to(.05 + np.arange(29) * .001, (n, 10, 29)).copy()
        q[:, 0] += token[:, :1]
        if self.sine:
            frame = (token[:, 0] * 16 + 16) + 32 * (token[:, 1] * 16 + 16)
            q[:, 0] = trajectory(frame * .02)[:, None]
        q[:, 1:] += 10  # Future slots must never be used as current targets.
        return [pack_window(q, np.full_like(q, -7))]


def trajectory(t):
    return .3 * np.sin(t * 3) + .07 * np.sin(t * 8)


def events(n=12):
    rows = []
    for i in range(n):
        t = 1 + i * .02
        rows.extend([
            angle_event('model', t, [.2] * 29, model_id='sonic', model_hash=POLICY_HASH,
                session='s', seq=i, phase='active', position_target=True, adapter='sonic-stk1',
                motion_token=[0.] * 64, sonic_source='live', sonic_policy_status='live_reference',
                source_blend_active=False),
            angle_event('command', t + .001, [.3] * 29, kp=[10.] * 29),
            angle_event('actual', t + .002, [.4] * 29)])
    return rows


class SonicReferenceTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name)
        self.bundle = self.root / 'bundle'
        self.bundle.mkdir()
        model = self.bundle / 'g1_kin_step28800.onnx'
        model.write_bytes(b'synthetic model stand-in; inference is mocked')
        self.manifest = dict(schema=1, reference_kind='learned_kinematic_reconstruction',
            units='rad', current_slot=0, robot_window_frames=10, robot_window_dt_s=.1,
            joint_names=list(BODY), packing=PACKING, compatible_policy_sha256=[POLICY_HASH],
            reference_angle_transform='absolute joint coordinates; no action scaling or default-pose addition',
            kinematic_onnx_sha256=hashlib.sha256(model.read_bytes()).hexdigest())
        self.write_manifest()
        self.session = FakeSession()
        self.session_patch = patch.object(SonicReference, '_open_session', return_value=self.session)
        self.open_session = self.session_patch.start()
        self.addCleanup(self.session_patch.stop)

    def write_manifest(self):
        (self.bundle / 'kinematic_manifest.json').write_text(json.dumps(self.manifest))

    def evaluate(self, rows, **options):
        source = self.root / 'events.jsonl'
        source.write_text(''.join(json.dumps(e) + '\n' for e in rows))
        db = self.root / 'input.sqlite'
        meta = import_recording(source, db)
        out = self.root / 'report'
        result = analyze(db, out, Options(sonic_reference=str(self.bundle), **options))
        json.loads((out / 'report.json').read_text(), parse_constant=lambda x: self.fail(x))
        return result, meta, out

    def test_unpack_preserves_all_frames_and_separates_velocities(self):
        q = np.arange(580).reshape(2, 10, 29) / 100.
        dq = -100 - q
        actual_q, actual_dq, rot = unpack(pack_window(q, dq))
        np.testing.assert_allclose(actual_q, q)
        np.testing.assert_allclose(actual_dq, dq)
        self.assertEqual(rot.shape, (2, 10, 6))
        with self.assertRaises(ValueError): unpack(np.zeros((2, 29)))

    def test_known_reference_errors_and_existing_results_unchanged(self):
        rows = events()
        for e in rows:
            if e['kind'] == 'model': e.update(action_scale=[999.] * 29, default_q=[999.] * 29)
        r, _, out = self.evaluate(rows)
        base = analyze(self.root / 'input.sqlite', self.root / 'baseline', Options())
        for label, metrics in base['segments'][0]['comparisons'].items():
            self.assertEqual(r['segments'][0]['comparisons'][label], metrics)
        comparisons = r['segments'][0]['comparisons']
        for label, delta in [('actual_minus_reference', .35), ('model_minus_reference', .15),
                             ('command_minus_reference', .25)]:
            self.assertAlmostEqual(comparisons[label][BODY[0]]['bias_deg'], np.degrees(delta), places=5)
        self.assertEqual(r['summary']['reference_valid_body29_frames'], 12)
        with np.load(out / 'aligned.npz', allow_pickle=False) as data:
            np.testing.assert_allclose(data['s0_q_ref'][0], .05 + np.arange(29) * .001)
            self.assertTrue(data['s0_valid_reference'].all())
            np.testing.assert_allclose(data['s0_reference_age_s'], .002)
        with np.load(out / 'sonic_reference.npz', allow_pickle=False) as data:
            self.assertEqual(data['reference_window_rad'].shape, (12, 10, 29))
            self.assertGreater(float(data['reference_window_rad'][0, 1, 0]), 10)
        self.assertIn('actual_minus_reference', (out / 'metrics.csv').read_text())
        self.assertIn('q_ref', (out / 'report.html').read_text())

    def test_legacy_trace_retains_token_and_phase_provenance(self):
        arrays = {k: np.zeros(n, np.float32) for k, n in zip(ARRAYS, SIZES)}
        arrays['motion_token'] = np.tile(np.arange(-16, 16) / 16, 2)
        m = dict(schema=1, model_id='sonic', model_sha256=POLICY_HASH, joint_names=list(BODY),
            default_q=[0.] * 29, action_scale=[1.] * 29, source='live', policy_status='live_reference',
            source_blend_active=False, session='s', sequence=2, started_monotonic=1.,
            completed_monotonic=1.001, reference_frame_index=23, reference_sequence=4,
            reference_received_monotonic=.999)
        header = json.dumps(m).encode()
        packet = b'STK1' + struct.pack('<I', len(header)) + header
        packet += b''.join(np.asarray(arrays[k], dtype='<f4').tobytes() for k in ARRAYS)
        event = dict(kind='trace', value=dict(packet_b64=base64.b64encode(packet).decode()), received_monotonic=1.002)
        converted = convert(event)
        np.testing.assert_array_equal(converted['motion_token'], arrays['motion_token'])
        self.assertEqual(converted['reference_frame_index'], 23)
        self.assertEqual(converted['sonic_policy_status'], 'live_reference')
        r, meta, _ = self.evaluate([event])
        self.assertEqual(meta['sonic_trace_fields_version'], 1)
        with open_db(self.root / 'input.sqlite', readonly=True) as db:
            stored = list(load_epoch(db, 0))[0]
        self.assertEqual(stored['motion_token'], converted['motion_token'])

    def test_missing_token_old_index_keeps_command_and_model_results(self):
        rows = events()
        for e in rows:
            e.pop('motion_token', None)
        r, _, _ = self.evaluate(rows)
        self.assertEqual(r['summary']['reference_valid_body29_frames'], 0)
        self.assertEqual(r['summary']['model_valid_body29_frames'], 12)
        self.assertEqual(r['summary']['command_valid_body29_frames'], 12)
        self.assertEqual(r['sonic_reference']['inference_status_counts']['missing_motion_token_reimport_required'], 12)
        self.open_session.assert_not_called()

    def test_no_trace_still_measures_command_error(self):
        r, _, out = self.evaluate([e for e in events() if e['kind'] != 'model'])
        self.assertEqual(r['summary']['command_valid_body29_frames'], 12)
        self.assertEqual(r['summary']['reference_result'], 'unavailable')
        self.assertIsNone(r['segments'][0]['comparisons']['actual_minus_reference']['body29']['rmse_deg'])
        with np.load(out / 'sonic_reference.npz', allow_pickle=False) as data:
            self.assertEqual(data['reference_window_rad'].shape, (0, 10, 29))

    def test_wrong_policy_hash_does_not_use_reference(self):
        rows = events()
        for e in rows:
            if e['kind'] == 'model': e['model_hash'] = 'b' * 64
        r, _, _ = self.evaluate(rows)
        self.assertEqual(r['sonic_reference']['inference_status_counts']['incompatible_policy_hash'], 12)
        self.assertEqual(r['summary']['reference_valid_body29_frames'], 0)
        self.assertEqual(r['summary']['model_valid_body29_frames'], 12)
        self.open_session.assert_not_called()

    def test_invalid_fsq_and_nan_rejected(self):
        self.assertIsNone(valid_token([.01] * 64))
        self.assertIsNone(valid_token([float('nan')] * 64))
        self.assertIsNone(valid_token([1.] * 64))
        self.assertIsNone(valid_token([0.] * 63))
        rows = events()
        for e in rows:
            if e['kind'] == 'model': e['motion_token'] = [.01] * 64
        r, _, _ = self.evaluate(rows)
        self.assertEqual(r['sonic_reference']['inference_status_counts']['invalid_fsq_token'], 12)
        self.assertEqual(r['summary']['reference_valid_body29_frames'], 0)

    def test_hold_and_blend_remain_excluded_even_when_model_scored(self):
        rows = events()
        for i, e in enumerate(rows):
            if e['kind'] == 'model': e['phase'] = 'hold' if i < 18 else 'blend'
        r, _, _ = self.evaluate(rows, include_phases=('active', 'hold', 'blend'))
        self.assertEqual(r['summary']['model_valid_body29_frames'], 12)
        self.assertEqual(r['summary']['reference_valid_body29_frames'], 0)
        self.assertEqual(r['sonic_reference']['inference_status_counts']['hold_or_blend_reference_excluded'], 12)

    def test_idle_reference_not_claimed_live(self):
        rows = events()
        for e in rows:
            if e['kind'] == 'model': e['sonic_source'] = 'idle'
        r, _, _ = self.evaluate(rows)
        self.assertEqual(r['sonic_reference']['inference_status_counts']['not_live_reference'], 12)

    def test_non_sonic_model_does_not_get_reference(self):
        rows = events()
        for e in rows:
            if e['kind'] == 'model': e['adapter'] = 'normal'
        r, _, _ = self.evaluate(rows)
        self.assertEqual(r['sonic_reference']['inference_status_counts']['not_sonic_trace'], 12)
        self.assertEqual(r['summary']['command_valid_body29_frames'], 12)

    def test_model_switch_cannot_reuse_previous_reference(self):
        rows = events()
        for e in rows[18:]:
            if e['kind'] == 'model': e.update(model_hash='b' * 64, session='new')
        r, _, _ = self.evaluate(rows)
        self.assertEqual(len(r['segments']), 2)
        self.assertEqual(r['summary']['reference_valid_body29_frames'], 6)
        self.assertEqual(r['segments'][1]['sonic_reference']['valid_body29_frames'], 0)

    def test_future_inference_is_not_paired(self):
        rows = events(1)
        rows[0]['t'] = rows[0]['received_monotonic'] = 1.01
        rows = rows[1:] + rows[:1]
        r, _, _ = self.evaluate(rows, command_age_ms=50, model_age_ms=50, feedback_gap_ms=50)
        self.assertEqual(r['summary']['reference_valid_body29_frames'], 0)

    def test_explicit_missing_sequence_never_guesses_reference(self):
        rows = events()
        for e in rows:
            if e['kind'] == 'command':
                e.update(model_session='s', model_id='sonic', model_hash=POLICY_HASH, model_seq=999)
        r, _, _ = self.evaluate(rows)
        self.assertEqual(r['summary']['command_valid_body29_frames'], 12)
        self.assertEqual(r['summary']['reference_valid_body29_frames'], 0)

    def test_clock_jump_and_missing_sequence_keep_boundaries(self):
        rows = events()
        for i, e in enumerate(rows):
            e['ros_now_ns'] = int((e['received_monotonic'] + 1000 + (1 if i >= 18 else 0)) * 1e9)
        rows = [e for e in rows if not (e['kind'] == 'model' and e['seq'] == 3)]
        r, meta, _ = self.evaluate(rows)
        self.assertEqual(meta['clock_epochs'], 2)
        self.assertEqual(meta['counts']['model_sequence_missing'], 1)
        self.assertLess(r['summary']['reference_valid_body29_frames'], 12)
        self.assertTrue(any(s['boundary'] == 'model_sequence_gap' for s in r['segments']))

    def test_decoder_failure_does_not_destroy_execution_metrics(self):
        self.session.fail = True
        r, _, _ = self.evaluate(events())
        self.assertEqual(r['summary']['model_valid_body29_frames'], 12)
        self.assertEqual(r['summary']['reference_valid_body29_frames'], 0)
        self.assertEqual(r['sonic_reference']['inference_status_counts']['decoder_failed'], 12)
        self.assertTrue(r['sonic_reference']['decoder_errors'])

    def test_manifest_reordered_names_are_mapped(self):
        self.manifest['joint_names'] = list(reversed(BODY)); self.write_manifest()
        _, _, out = self.evaluate(events())
        with np.load(out / 'aligned.npz', allow_pickle=False) as data:
            np.testing.assert_allclose(data['s0_q_ref'][0], (.05 + np.arange(29) * .001)[::-1])

    def test_changed_model_file_refused(self):
        (self.bundle / 'g1_kin_step28800.onnx').write_bytes(b'wrong weights')
        with self.assertRaisesRegex(ValueError, 'SHA256'): SonicReference(self.bundle)

    def test_unsupported_units_packing_and_future_slot_refused(self):
        for key, value in [('units', 'deg'), ('packing', 'take_first_29'), ('current_slot', 1)]:
            with self.subTest(key=key):
                old = self.manifest[key]
                self.manifest[key] = value; self.write_manifest()
                with self.assertRaises(ValueError): SonicReference(self.bundle)
                self.manifest[key] = old

    def test_known_reference_delay_80ms(self):
        self.session.sine = True
        original = events(300)
        rows = []
        for i in range(300):
            model, command, _ = original[i * 3:i * 3 + 3]
            model['motion_token'][:2] = [(i % 32 - 16) / 16, (i // 32 - 16) / 16]
            model['q'] = command['q'] = [float(trajectory(i * .02))] * 29
            rows.extend([model, command])
        # 200 Hz feedback observes the held 50 Hz target delayed by exactly 80 ms.
        # A 50 Hz feedback-only series cannot resolve the lag to a 5 ms grid.
        for i in range(1200):
            rows.append(angle_event('actual', 1.002 + i * .005,
                [float(trajectory(((i - 16) // 4) * .02))] * 29))
        rows.sort(key=lambda e: e['received_monotonic'])
        r, _, _ = self.evaluate(rows)
        lag = r['segments'][0]['comparisons']['actual_minus_reference'][BODY[0]]['lag']
        self.assertAlmostEqual(lag['estimated_lag_ms'], 80, delta=5)
        self.assertGreater(lag['zero_lag_same_samples_rmse_deg'], lag['compensated_rmse_deg'])


if __name__ == '__main__':
    unittest.main()
