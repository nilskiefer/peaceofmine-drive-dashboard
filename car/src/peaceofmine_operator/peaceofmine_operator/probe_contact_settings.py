"""Probe contact source and raw-count limit relative to a saved zero."""
import math

DEFAULTS = dict(source='adc', threshold_counts=None, zero_counts=None)


def validate(value):
    if not isinstance(value,dict) or value.get('source') not in ('adc','esp32'):
        raise ValueError('Select ADC or ESP32 load cell')
    result = {key:value.get(key,default) for key,default in DEFAULTS.items()}
    limit, zero = result['threshold_counts'], result['zero_counts']
    if limit is not None and (type(limit) not in (int,float) or not math.isfinite(limit) or limit <= 0):
        raise ValueError('Threshold must be positive counts')
    if zero is not None and (type(zero) not in (int,float) or not math.isfinite(zero) or not -8388608 < zero < 8388607):
        raise ValueError('Invalid load-cell zero')
    return result
