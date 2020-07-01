from unittest import TestCase
from test.brick.movemonitor import MoveMonitor
from datetime import datetime
from test.brick.testbrick import TBrick, DECIMALS, test_pv_root
from test.test_pmac.test_system.trajectories import trajectory_quick_scan
from cothread import Sleep, catools as ca
import pytest
import os

# these are test_regression tests for the set of issues that pmacController::makeCSDemandsConsistent
# is trying to solve

class TestMakeCsConsistent(TestCase):
    def test_kill_resets_cs_demands(self):
        tb = TBrick()
        tb.set_cs_group(tb.g3)

        tb.cs3.set_deferred_moves(True)
        tb.cs3.set_move_time(3000)
        tb.height.go(1000, wait=False)
        tb.cs3.M1.go_direct(1000, wait=False)

        # verify no motion yet
        Sleep(0.1)
        self.assertAlmostEqual(tb.height.pos, 0, DECIMALS)
        self.assertAlmostEqual(tb.m1.pos, 0, DECIMALS)

        # start motion and then kill all
        tb.cs3.set_deferred_moves(False)
        Sleep(0.5)
        tb.send_command("&3a")
        tb.send_command("#1k")
        # let the motors settle
        Sleep(1)

        h = tb.height.pos
        m1 = tb.m1.pos
        self.assertGreater(h, 0)
        self.assertGreater(m1, 0)

        # now move a different motor in the CS, the other two would continue to
        # their previous destinations if makeCSDemandsConsistent has failed
        tb.cs3.M2.go_direct(10)

        self.assertAlmostEqual(h, tb.height.pos, DECIMALS)
        self.assertAlmostEqual(m1, tb.m1.pos, DECIMALS)
        self.assertAlmostEqual(10, tb.m2.pos, DECIMALS)

    @pytest.mark.skipif(
        os.environ.get("PPMAC") == "True", reason="not supported on PPMAC yet"
    )
    def _test_auto_home_resets_cs_demands(self):
        tb = TBrick()
        tb.set_cs_group(tb.g3)

        tb.height.go(10)
        tb.cs3.M1.go_direct(10)
        self.assertAlmostEqual(10, tb.height.pos, DECIMALS)
        self.assertAlmostEqual(10, tb.m1.pos, DECIMALS)

        try:
            # auto home everything
            ca.caput('{}:HM:HMGRP'.format(test_pv_root), 'All')
            ca.caput('{}:HM:HOME'.format(test_pv_root), 1, wait=True, timeout=20)

            self.assertAlmostEqual(0, tb.height.pos, DECIMALS)
            self.assertAlmostEqual(0, tb.m1.pos, DECIMALS)
            
            Sleep(1.0)

            # now move a different motor in the CS, the other two would continue to
            # their previous destinations if makeCSDemandsConsistent has failed
            tb.cs3.M2.go_direct(10)
            self.assertAlmostEqual(0, tb.height.pos, DECIMALS)
            self.assertAlmostEqual(0, tb.m1.pos, DECIMALS)
            self.assertAlmostEqual(10, tb.m2.pos, DECIMALS)
        finally:
            # ensure good state if homing failed
            ca.caput('{}:HM:ABORT.PROC'.format(test_pv_root), 1)

    def test_abort_cs_resets_cs_demands(self):
        tb = TBrick()
        tb.set_cs_group(tb.g3)

        tb.cs3.set_deferred_moves(True)
        tb.cs3.set_move_time(3000)
        tb.height.go(1000, wait=False)
        tb.cs3.M1.go_direct(1000, wait=False)

        # verify no motion yet
        Sleep(0.1)
        self.assertAlmostEqual(tb.height.pos, 0, DECIMALS)
        self.assertAlmostEqual(tb.m1.pos, 0, DECIMALS)

        # start motion and then abort CS moves
        tb.cs3.set_deferred_moves(False)
        # allow the axes to get started
        Sleep(0.1)
        tb.cs3.abort()
        # let the motors settle
        Sleep(0.5)

        h = tb.height.pos
        m1 = tb.m1.pos
        self.assertGreater(h, 0)
        self.assertGreater(m1, 0)

        # now move a different motor in the CS, the other two would continue to
        # their previous destinations if makeCSDemandsConsistent has failed
        tb.cs3.M2.go_direct(10)
        self.assertLess(tb.height.pos, h + 1)  # some settling after abort is OK
        self.assertAlmostEqual(m1, tb.m1.pos, DECIMALS)
        self.assertAlmostEqual(10, tb.m2.pos, DECIMALS)

    def test_stop_on_limit_resets_cs_demands(self):
        tb = TBrick()
        tb.set_cs_group(tb.g3)

        m = MoveMonitor(tb.height.pv_root)
        tb.cs3.set_deferred_moves(True)
        tb.cs3.set_move_time(3000)
        tb.height.go(10, wait=False)
        tb.cs3.M1.go_direct(10, wait=False)

        # verify no motion yet
        Sleep(0.1)
        self.assertAlmostEqual(tb.height.pos, 0, DECIMALS)
        self.assertAlmostEqual(tb.m1.pos, 0, DECIMALS)

        # start motion but stop on lim
        tb.m3.set_limits(-1, 1)
        tb.cs3.set_deferred_moves(False)
        # let axes settle
        m.wait_for_one_move(2)

        h = tb.height.pos
        m1 = tb.m1.pos
        self.assertLess(h, 10)
        self.assertLess(m1, 10)

        # for some reason a large wait is required before go_direct
        # in a real scenario this is fine even though I don't understand it
        Sleep(5)
        # now move a different motor in the CS, the other two would continue to
        # their previous destinations if makeCSDemandsConsistent has failed
        tb.cs3.M2.go_direct(10)
        self.assertAlmostEqual(h, tb.height.pos, DECIMALS)
        self.assertAlmostEqual(m1, tb.m1.pos, DECIMALS)
        self.assertAlmostEqual(10, tb.m2.pos, DECIMALS)

    def test_ffe_trajectory_scan(self):
        """
            Do we get fatal following error when doing a traj scan if one of the
            motors has a radically incorrect Q7x ?
        :return: None
        """
        tb = TBrick()
        tb.set_cs_group(tb.g3)
        tb.cs3.set_move_time(0)

        # make axis 1 demand cache Q71 radically wrong
        command = "&3 Q71=90000"
        tb.send_command(command)
        # this check is probably not required but leaving it in to verify reliability
        self.assertEquals(tb.get_command(), command)

        # if this succeeds without error then we are all good
        trajectory_quick_scan(self, tb)

    def test_kinematic_axis_creep(self):
        """
            Check that a move in a coordinate system starts from the previous demand for axes
            that are not being demanded in this move. i.e. jitter does not accumulate in CS axes
            that are not being demanded.

        :return: None
        """
        tb = TBrick()
        tb.set_cs_group(tb.g3)
        tb.cs3.set_move_time(0)

        # do a CS move of Height
        tb.height.go(1)

        # pretend that axis 3 moved by itself and monitor change in height.
        monitor = MoveMonitor(tb.height.pv_root)
        tb.send_command("#3J:1000")
        monitor.wait_for_one_move(10)
        # this should make Height 1.5mm
        self.assertAlmostEqual(tb.height.pos, 1.5, DECIMALS)

        # now move Angle
        tb.angle.go(1)
        self.assertAlmostEqual(tb.angle.pos, 1, DECIMALS)
        # Height should return to 1
        self.assertAlmostEqual(tb.height.pos, 1, DECIMALS)

    def test_direct_axis_creep(self):
        """
            Check that a move in a coordinate system starts from the previous demand for axes
            that are not being demanded in this move. i.e. jitter does not accumulate in CS axes
            that are not being demanded.

            This test is the same as test_kinematic_axis_creep except that the initial move and
            position verification is via real axes. This did raise an interesting
            issue not seen in the previous test. See comment NOTE below

        :return: None
        """
        tb = TBrick()
        # set the appropriate coordinate system group
        tb.set_cs_group(tb.g3)
        # the first CS move after a coord sys group switch clears the cached real motor positions
        # so this test must do that initial CS move here
        # MAKE SURE this actually initiates a move, else makeCSDemandsConsistent wont be called
        tb.height.go(1)
        # NOTE: the above is a little artificial and masks a very minor issue: any axis creep that occurs
        # between changing coordinate system mappings and the first CS move will be kept - fixing
        # this would require architecture changes for an unlikely event with little consequence

        for cs_move in range(6, 10):
            move = cs_move
            tb.cs3.set_move_time(0)
            # move affected axes to start
            # note this is also testing that moving the real axes updates the
            # Q7x variables for the associated virtual axes
            tb.all_go([tb.m1, tb.m2, tb.m3, tb.m4], [0, 0, 0, 0])

            # pretend that axis 1 moved by itself.
            monitor = MoveMonitor(tb.m1.pv_root)
            tb.send_command("#1J:{}".format(move))
            monitor.wait_for_one_move(1, throw=False)
            # this should put axis 1 at {move}mm
            self.assertEquals(tb.m1.pos, move)

            # now move the CS axis Height
            tb.height.go(cs_move)
            self.assertAlmostEqual(tb.height.pos, cs_move, DECIMALS)
            # Axis 1 should return to 0
            self.assertAlmostEqual(tb.m1.pos, 0, DECIMALS)

    def test_very_slow_moves(self):
        """
            If Q7x is very different from Qx for an axis that is not in the kinematics or 1-1 maps
            then we used to get very slow motion because it gets included in the FRAX and hence
            the speed calculations for the entire move. Verify this no longer occurs.

        :return: None
        """
        tb = TBrick()
        tb.set_cs_group(tb.g3)
        tb.cs3.set_move_time(0)

        for retry in range(5):
            # move affected axes to 0
            tb.all_go([tb.m1, tb.m2, tb.m3, tb.m4], [0, 0, 0, 0])
            # make axis 5 demand radically wrong (not in a CS)
            tb.send_command("&3 Q75=900000")

            # make a CS move and make sure it happens quickly enough
            then = datetime.now()
            # Sleep(.5)   - this is required for reliable pass on PowerBrick BUT WHY?
            tb.height.go(1)
            self.assertAlmostEqual(tb.height.pos, 1, DECIMALS)
            elapsed = datetime.now() - then
            self.assertLess(elapsed.seconds, 4)

    def test_switch_cs_group(self):
        tb = TBrick()
        tb.set_cs_group(tb.g3)
        tb.all_go([tb.A3, tb.B3], [3, 3])

        self.assertAlmostEqual(tb.m1.pos, 3)
        self.assertAlmostEqual(tb.m2.pos, 3)

        tb.set_cs_group(tb.g1)
        tb.all_go([tb.A2, tb.B2], [2, 2])

        # scaling of 1000 and 100 on A and B on CS2 axes gives 20, 200 on axes 1, 2
        self.assertAlmostEqual(tb.m1.pos, 20)
        self.assertAlmostEqual(tb.m2.pos, 200)

        tb.set_cs_group(tb.g3)
        tb.all_go([tb.A3], [3])

        self.assertAlmostEqual(tb.m1.pos, 3)
        # if make CS consistent has failed then this will also have moved back to pos 3
        self.assertAlmostEqual(tb.m2.pos, 200)

    def test_return_to_zero(self):
        """ look for a bug where having the angle set in CS 3 means that both jacks cannot return to
            zero when another test is initialized. This was a bug in the test framework, not pmac
        """
        tb = TBrick()
        tb.set_cs_group(tb.g3)

        tb.all_go([tb.jack1, tb.jack2], [5, 5])

        # check height(x) and angle(y) in CS3
        self.assertAlmostEqual(tb.height.pos, 5, DECIMALS)
        self.assertAlmostEqual(tb.angle.pos, 0, DECIMALS)

        tb.jack1.go(6)

        tb2 = TBrick()
        self.assertAlmostEqual(tb2.m3.pos, 0, DECIMALS)
        self.assertAlmostEqual(tb2.m4.pos, 0, DECIMALS)
