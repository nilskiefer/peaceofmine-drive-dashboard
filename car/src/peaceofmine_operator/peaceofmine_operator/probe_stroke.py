"""Car settings for calibrated probe travel and automatic homing."""
import sys
import math

DEFAULTS = dict(down_mm_s=5., retract_mm_s=5., retract_slow_mm_s=5., home_load_percent=10., contact_slowdown_factor=1.)
HOME_SLOW_ZONE_MM = 10.


def units_per_mm_s(calibration):
    """Convert linear speed using the existing measured encoder travel."""
    return calibration['travel_ticks']/calibration['max_extension_mm'] * 360/(4096*.684)


def motor_speed_limit(calibration, stroke, fallback):
    if not calibration:
        return fallback
    requested = max(stroke['down_mm_s'], stroke['retract_mm_s'],stroke.get('retract_slow_mm_s',5.)) * units_per_mm_s(calibration)
    # Independent driver limit follows the saved linear speeds. The hardware
    # maximum still applies, and zero must never reach the MX speed register.
    return max(fallback, math.ceil(min(1023,requested)))


def motor_acceleration_limit(calibration, stroke, fallback=4):
    if not calibration:
        return fallback
    factor = units_per_mm_s(calibration)
    speed = min(1023/factor,max(stroke['down_mm_s'],stroke['retract_mm_s']))
    # Leave a quarter of the measured stroke for acceleration and another
    # quarter for braking. The former fixed shaft acceleration prevented
    # different requested linear speeds from being reached on short strokes.
    linear_acceleration = 2*speed*speed/calibration['max_extension_mm']
    units = linear_acceleration*factor*.684/8.583
    return max(fallback,math.ceil(min(254,units)))


def validate(value):
    if not isinstance(value, dict):
        raise ValueError('Expected probe stroke settings')
    value = dict(value)
    value.setdefault('contact_slowdown_factor',1.)
    if 'retract_slow_mm_s' not in value:
        fast=value.get('retract_mm_s')
        value['retract_slow_mm_s'] = min(fast,5.) if type(fast) in (int,float) else 5.
    for key in ('down_mm_s', 'retract_mm_s','retract_slow_mm_s'):
        speed = value.get(key)
        if type(speed) not in (int, float) or not 0 < speed <= sys.float_info.max:
            raise ValueError('Down and retract speeds must be positive mm/s')
    if value['retract_slow_mm_s'] > value['retract_mm_s']:
        raise ValueError('Near-home retract speed must not exceed fast retract speed')
    factor=value['contact_slowdown_factor']
    if type(factor) not in (int,float) or not math.isfinite(factor) or factor < 1:
        raise ValueError('Contact slowdown factor must be finite and at least 1')
    load = value.get('home_load_percent')
    if type(load) not in (int, float) or not 10 <= load <= 60:
        raise ValueError('Automatic home load must be between 10 and 60 percent')
    return {key: float(value[key]) for key in DEFAULTS}
