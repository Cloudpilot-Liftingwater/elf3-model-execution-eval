"""Summarize recorded diagnostics without changing scoring or robot control."""
from collections import Counter
import math
from numbers import Real


class TemperatureSummary:
    def __init__(self):
        self.fields = {'motor_temperature': {}, 'driver_temperature': {}}
        self.invalid_arrays = Counter()

    def add(self, event):
        if event.get('kind') != 'actual':return
        names = event.get('temperature_names', event.get('names', ()))
        for field, rows in self.fields.items():
            values = event.get(field)
            if values is None:continue
            if len(names) != len(set(names)) or len(values) != len(names):
                self.invalid_arrays[field] += 1
                continue
            for name, value in zip(names, values):
                row = rows.setdefault(name, dict(samples=0, invalid=0, minimum_c=None, maximum_c=None))
                if not isinstance(value, Real) or isinstance(value, bool) or not math.isfinite(value):
                    row['invalid'] += 1
                    continue
                value=float(value)
                row['samples'] += 1
                row['minimum_c'] = value if row['minimum_c'] is None else min(value, row['minimum_c'])
                row['maximum_c'] = value if row['maximum_c'] is None else max(value, row['maximum_c'])

    def report(self, source):
        topic = source.get('metadata.json', {}).get('topics', {}).get('state', '')
        simulated = topic.lstrip('/').startswith('simulation/')
        observed = any(r['samples'] for rows in self.fields.values() for r in rows.values())
        return dict(status='not_assessed_simulation' if simulated else 'observed' if observed else 'unobserved',
                    unit='degC', by_joint=self.fields, invalid_arrays=dict(self.invalid_arrays),
                    note='Motor protection thresholds come from robot motor configuration; driver thresholds are not inferred.')


def summarize_events(events):
    # Older recordings contain lifecycle events without severity or fault code.
    rows = []
    for event in events:
        if event.get('status') not in ('failed', 'cancelled', 'interrupted', 'not_assessed', 'warning'):
            continue
        row = dict(event)
        row.setdefault('severity', 'error' if row['status']=='failed' else 'warning' if row['status'] in ('cancelled','interrupted','warning') else 'info')
        row.setdefault('category', 'operator' if row['status'] in ('cancelled','interrupted') else 'unclassified')
        rows.append(row)
    return dict(events=rows, by_severity=dict(Counter(r['severity'] for r in rows)),
                by_category=dict(Counter(r.get('category') or 'unclassified' for r in rows)))
