"""Sweep limits are separate from the arm's physical envelope."""
import unittest
from peaceofmine_operator import arm_sweep


class ArmSweepTest(unittest.TestCase):
    calibration = dict(minimum=1024,center=2048,maximum=3072)

    def test_legacy_defaults_and_asymmetric_sweep_preserve_full_range(self):
        self.assertEqual(arm_sweep.resolve_ticks(arm_sweep.DEFAULTS,self.calibration),(1024,3072))
        self.assertEqual(arm_sweep.resolve_ticks(dict(minimum_deg=-22.5,maximum_deg=45.),self.calibration),(1792,2560))
        self.assertEqual(self.calibration,dict(minimum=1024,center=2048,maximum=3072))

    def test_invalid_or_outside_sweep_is_rejected(self):
        for low,high in ((None,10.),(1.,1.),(20.,-20.),(-100.,20.),(-20.,100.),(False,20.),(0.,float('nan')),(0.,.0001)):
            with self.subTest(low=low,high=high),self.assertRaises(ValueError):
                arm_sweep.resolve_ticks(dict(minimum_deg=low,maximum_deg=high),self.calibration)
