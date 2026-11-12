import math
import unittest
from peaceofmine_operator import probe_guidance as g


class GeometryTest(unittest.TestCase):
    def setUp(self):
        self.geom = g.validate(dict(pivot_x_m=.2, pivot_y_m=.1, coil_radius_m=1., probe_radius_m=.5))
        self.pose = dict(x=2., y=3., yaw=0.)

    def test_points_rotate_pivot_and_use_separate_radii(self):
        self.assertEqual(g.metal_point(self.pose, 0., self.geom), dict(x=3.2, y=3.1))
        self.pose['yaw'] = math.pi/2
        point = g.probe_point(self.pose, 0., 90., self.geom)
        self.assertAlmostEqual(point['x'], 1.4)
        self.assertAlmostEqual(point['y'], 3.2)

    def test_error_is_from_tip_but_reach_is_from_pivot(self):
        target = dict(id='mark', x=2.75, y=3.1)
        result = g.guidance(self.pose, 0., 0., self.geom, target, -90., 90.)
        self.assertAlmostEqual(result['distance_m'], .05)
        self.assertAlmostEqual(result['forward_m'], .05)
        self.assertTrue(result['reachable'])
        target['x'] += .001
        self.assertFalse(g.guidance(self.pose, 0., 0., self.geom, target, -90., 90.)['reachable'])

    def test_wrapping_and_limits(self):
        target = g.probe_point(self.pose, 180., 30., self.geom)
        result = g.guidance(self.pose, 170., 30., self.geom, target, 160., 200.)
        self.assertTrue(result['reachable'])
        self.assertAlmostEqual(result['angle_deg'], 180.)
        self.assertFalse(g.guidance(self.pose, 0., 30., self.geom, target, -45., 45.)['reachable'])

    def test_dimensions_must_be_measured(self):
        self.assertIsNone(g.validate(None))
        for value in ({}, dict(self.geom, coil_radius_m=0), dict(self.geom, probe_radius_m=-1), dict(self.geom, pivot_x_m=float('nan'))):
            with self.assertRaises(ValueError):
                g.validate(value)
