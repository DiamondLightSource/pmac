from unittest import TestCase, skip
from test.brick.testbrick import TestBrick
from test.test_system.trajectories import trajectory_fast_scan, trajectory_scan_appending, trajectory_quick_scan
from cothread import Sleep, catools as ca
import pytest
import os


class TestSysTrajectory(TestCase):
    def test_simple_trajectory(self):
        """ very quick test of trajectory for debugging
            """
        tb = TestBrick()
        # why does this fail?
        # tb.set_cs_group(tb.g1)
        # Sleep(.1)
        trajectory_fast_scan(self, tb, 1, millisecs=100000)
        self.assertAlmostEqual(tb.m1.pos, 1, 1)
