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
        trajectory_fast_scan(self, tb, 1, microsecs=100000)
        self.assertAlmostEqual(tb.m1.pos, 1, 1)

    def test_trajectory_error(self):
        """
        Verify that coordinate system error 1 can be detected
        and displayed
        """
        tb = TBrick()
        with self.assertRaises(RuntimeError):
            trajectory_fast_scan(self, tb, 1, microsecs=10)

        error_string = tb.get_trajectory_error()
        self.assertTrue("calculation time" in error_string)

        with self.assertRaises(RuntimeError):
            trajectory_fast_scan(self, tb, 1, microsecs=10000, distance=10000)

        error_string = tb.get_trajectory_error()
        self.assertTrue("M1 Lim" in error_string)

        tb.m1.set_limits(-100000, 100000)

        with self.assertRaises(RuntimeError):
            trajectory_fast_scan(self, tb, 1, microsecs=10000, distance=10000)

        error_string = tb.get_trajectory_error()
        self.assertTrue("following error" in error_string)
