"""Detector sweep endpoints, in degrees relative to the recorded arm centre."""
import math

DEFAULTS = dict(minimum_deg=None, maximum_deg=None)


def validate(value):
    if not isinstance(value, dict):
        raise ValueError('Expected detector sweep range')
    low, high = value.get('minimum_deg'), value.get('maximum_deg')
    if low is None and high is None:
        return dict(DEFAULTS)
    if not all(type(v) in (int, float) and math.isfinite(v) and -360 <= v <= 360 for v in (low, high)) or low >= high:
        raise ValueError('Enter ordered right and left sweep limits in degrees')
    return dict(minimum_deg=float(low), maximum_deg=float(high))


def resolve_ticks(value, calibration):
    value = validate(value)
    if not calibration or not all(type(calibration.get(k)) is int for k in ('minimum','center','maximum')):
        raise ValueError('Record and save the full arm limits before setting the sweep range')
    low, center, high = (calibration[k] for k in ('minimum','center','maximum'))
    if not low < center < high:
        raise ValueError('Invalid full arm limits')
    if value['minimum_deg'] is None:
        return low, high
    start = math.floor(center+value['minimum_deg']*4096/360+.5)
    end = math.floor(center+value['maximum_deg']*4096/360+.5)
    if not low <= start < end <= high:
        raise ValueError('Detector sweep range must stay within the full arm limits')
    return start, end
