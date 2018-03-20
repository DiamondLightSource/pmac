from unittest import TestCase, skip
from test.brick.testbrick import TestBrick
from test.test_system.trajectories import trajectory_fast_scan, trajectory_scan_appending, trajectory_quick_scan
from cothread import Sleep

# Tests for historical issues with trajectory scanning


class TestTrajectory(TestCase):
    def test_cs_switched_trajectory(self):
        """ test problem seen on VMXI where switching CS right after a traj
            scan causes the CS to immediately go into 'No motors in CS'
            error (err code 10)
            """
        tb = TestBrick()
        for iteration in range(20):
            trajectory_fast_scan(self, tb, 1)
            # todo this is required because the CS may be still decelerating after it
            # todo   reports it delivered the last point and completed
            # todo   VMXI needed just this sleep - I feel that fixing it will slow those scans
            # todo   that do not require CS group switching (but only by a tiny bit)
            # Sleep(0.1) todo it turns out this is not required here but that just means we
            # cannot reproduce the VMXi issue in this case
            tb.set_cs_group(tb.g2)
            tb.height.go(0)

    def test_appending(self):
        """ ensure that appended points are not added twicen_axes
        (due to busy record handling)
        """

        tb2 = TestBrick()
        trajectory_scan_appending(self, tb2)

    def test_4_dimension_trajectory(self):
        tb = TestBrick()
        # if this succeeds without error then we are all good
        trajectory_quick_scan(self, tb, tb.g3)

    def test_mres_offsets_trajectory(self):
        tb = TestBrick()
        tb.set_cs_group(tb.cs2)
        trajectory_quick_scan(self, tb, 1)
