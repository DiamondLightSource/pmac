from unittest import TestCase
from test.brick.testbrick import TestBrick
from test.test_system.trajectories import trajectory_fast_scan, trajectory_scan_appending

# Tests for historical issues with trajectory scanning

class TestTrajectory(TestCase):

    def test_cs_switched_trajectory(self):
        """ test problem seen on VMXI where switching CS right after a traj
            scan causes the CS to immediately go into 'No motors in CS'
            error (err code 10)
            """
        tb = TestBrick()
        for iteration in range(20):
            trajectory_fast_scan(self, tb)
            tb.set_cs_group(tb.g2)
            tb.height.go(0)

    def test_appending(self):
        """ ensure that appended points are not added twice
        (due to busy record handling)
        """
        tb = TestBrick()
        trajectory_scan_appending(self, tb)
