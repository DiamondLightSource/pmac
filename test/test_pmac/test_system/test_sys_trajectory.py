from unittest import TestCase
from test.brick.testbrick import TBrick
from test.test_pmac.test_system.trajectories import trajectory_fast_scan


class TestSysTrajectory(TestCase):
    def test_simple_trajectory(self):
        """ very quick test of trajectory for debugging
            """
        tb = TBrick()
        # why does this fail?
        # tb.set_cs_group(tb.g1)
        # Sleep(.1)
        trajectory_fast_scan(self, tb, 1, millisecs=100000)
        self.assertAlmostEqual(tb.m1.pos, 1, 1)
