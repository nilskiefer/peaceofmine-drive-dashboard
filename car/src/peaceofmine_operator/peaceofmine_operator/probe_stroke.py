"""Car settings for calibrated probe travel and automatic homing."""
import sys
import math

DEFAULTS = dict(down_mm_s=5., retract_mm_s=5., home_load_percent=10.)


def units_per_mm_s(calibration):
    """Convert linear speed using the existing measured encoder travel."""
    return calibration['travel_ticks']/calibration['max_extension_mm'] * 360/(4096*.684)


def motor_speed_limit(calibration, stroke, fallback):
    if not calibration:
        return fallback
    requested = max(stroke['down_mm_s'], stroke['retract_mm_s']) * units_per_mm_s(calibration)
    # Independent driver limit follows the saved linear speeds. The hardware
    # maximum still applies, and zero must never reach the MX speed register.
    return max(fallback, math.ceil(min(1023,requested)))


def validate(value):
    if not isinstance(value, dict):
        raise ValueError('Expected probe stroke settings')
    for key in ('down_mm_s', 'retract_mm_s'):
        speed = value.get(key)
        if type(speed) not in (int, float) or not 0 < speed <= sys.float_info.max:
            raise ValueError('Down and retract speeds must be positive mm/s')
    load = value.get('home_load_percent')
    if type(load) not in (int, float) or not 10 <= load <= 60:
        raise ValueError('Automatic home load must be between 10 and 60 percent')
    return {key: float(value[key]) for key in DEFAULTS}
