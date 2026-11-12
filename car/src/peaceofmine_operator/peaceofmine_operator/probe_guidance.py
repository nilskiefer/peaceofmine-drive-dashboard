"""Planar detector/probe geometry, in metres and odometry radians."""
import math

DEFAULTS = None  # No dimensions are inferred for an unmeasured mounting.
KEYS = ('pivot_x_m', 'pivot_y_m', 'coil_radius_m', 'probe_radius_m')


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate(value):
    if value is None:
        return None
    if not isinstance(value, dict) or not all(finite(value.get(k)) for k in KEYS):
        raise ValueError('Enter all four mounting dimensions in metres')
    if value['coil_radius_m'] <= 0 or value['probe_radius_m'] <= 0:
        raise ValueError('Coil and probe radii must be greater than zero')
    return {k: float(value[k]) for k in KEYS}


def valid_pose(pose):
    return isinstance(pose, dict) and all(finite(pose.get(k)) for k in ('x', 'y', 'yaw'))


def pivot_point(pose, geometry):
    c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
    x, y = geometry['pivot_x_m'], geometry['pivot_y_m']
    return dict(x=pose['x'] + c*x - s*y, y=pose['y'] + s*x + c*y)


def point(pose, angle_deg, geometry, radius):
    pivot = pivot_point(pose, geometry)
    bearing = pose['yaw'] + math.radians(angle_deg)
    return dict(x=pivot['x'] + radius*math.cos(bearing), y=pivot['y'] + radius*math.sin(bearing))


def metal_point(pose, fixture_angle_deg, geometry):
    return point(pose, fixture_angle_deg, geometry, geometry['coil_radius_m'])


def probe_point(pose, fixture_angle_deg, offset_deg, geometry):
    return point(pose, fixture_angle_deg + offset_deg, geometry, geometry['probe_radius_m'])


def guidance(pose, arm_angle, offset, geometry, target, lower, upper):
    tip = probe_point(pose, arm_angle, offset, geometry)
    dx, dy = target['x'] - tip['x'], target['y'] - tip['y']
    c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
    pivot = pivot_point(pose, geometry)
    px, py = target['x'] - pivot['x'], target['y'] - pivot['y']
    radial = math.hypot(px, py) - geometry['probe_radius_m']
    angle = math.degrees(math.atan2(py, px) - pose['yaw']) - offset
    angle = (angle + 180) % 360 - 180
    candidates = [angle + 360*k for k in (-1, 0, 1) if lower <= angle + 360*k <= upper]
    if candidates:
        angle = min(candidates, key=lambda a: abs(a - arm_angle))
    reachable = bool(candidates) and abs(radial) <= .05 + 1e-12
    return dict(available=True, target=dict(target), tip=tip, distance_m=math.hypot(dx, dy),
                forward_m=c*dx+s*dy, left_m=-s*dx+c*dy, radial_error_m=radial,
                angle_deg=angle, reachable=reachable,
                reason='' if reachable else 'Drive the probe arc onto the mark.' if not abs(radial) <= .05+1e-12 else 'Target is outside arm limits.')
