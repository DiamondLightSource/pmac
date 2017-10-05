from unittest import TestCase
from cothread import catools as ca
from test.helper.trajectory import Trajectory
from test.helper.movemonitor import MoveMonitor
from datetime import datetime, timedelta

# these are test_regression tests for the set of issues that pmacController::makeCSDemandsConsistent
# is trying to solve

P = 'PMAC_BRICK_TEST:'
PB = 'PMAC_BRICK_TEST:GB1:'


class TestMakeCsConsistent(TestCase):

    def test_ffe_trajectory_scan(self):
        """
            Do we get fatal following error when doing a traj scan if one of the
            motors has a radically incorrect Q7x ?
        :return: None
        """
        command = '&3 Q71=90000'
        ca.caput(PB+'COORDINATE_SYS_GROUP', 'MIXED')
        # move affected axes to 0
        ca.caput([P+'M1', P+'M2', P+'M3', P+'M4'], [0, 0, 0, 0], wait=True, timeout=30)
        # make axis 1 demand radically wrong
        ca.caput('PMAC_BRICK_TEST:SendCmd', command, datatype=ca.DBR_CHAR_STR)
        # this check is probably not required but leaving it in to verify reliability
        self. assertEquals(ca.caget('PMAC_BRICK_TEST:SendCmd', datatype=ca.DBR_CHAR_STR), command)

        # if this succeeds without error then we are all good
        Trajectory.quick_scan(self)

    def test_kinematic_axis_creep(self):
        """
            Check that a move in a coordinate system starts from the previous demand for axes
            that are not being demanded in this move. i.e. jitter does not accumulate in CS axes
            that are not being demanded.

        :return: None
        """
        # set the appropriate coordinate system group
        ca.caput(PB+'COORDINATE_SYS_GROUP', '3,4->I', wait=True)
        # move affected axes to 0
        ca.caput([P+'M3', P+'M4'], [0, 0], wait=True, timeout=30)

        # do a CS move of Height
        ca.caput(P+'X_CS3', 1, wait=True, timeout=5)

        # pretend that axis 3 moved by itself.
        monitor = MoveMonitor(P+'X_CS3')
        ca.caput('PMAC_BRICK_TEST:SendCmd', '#3J:1000', datatype=ca.DBR_CHAR_STR)
        monitor.wait_for_one_move(5)
        # this should make Height 1.5mm
        self.assertEquals(ca.caget(P+'X_CS3.RBV'), 1.5)

        # now move Angle
        ca.caput(P+'Y_CS3', 1, wait=True, timeout=5)
        self.assertEquals(ca.caget(P+'Y_CS3.RBV'), 1)
        # Height should return to 1
        self.assertEquals(ca.caget(P+'X_CS3.RBV'), 1)

    def test_direct_axis_creep(self):
        """
            Check that a move in a coordinate system starts from the previous demand for axes
            that are not being demanded in this move. i.e. jitter does not accumulate in CS axes
            that are not being demanded.

        :return: None
        """
        move = 10
        cs_move = 10
        # set the appropriate coordinate system group
        ca.caput(PB+'COORDINATE_SYS_GROUP', 'MIXED', wait=True)
        # move affected axes to
        ca.caput([P+'M1', P+'M2', P+'M3', P+'M4'], [0, 0, 0, 0], wait=True, timeout=30)

        # pretend that axis 1 moved by itself.
        monitor = MoveMonitor(P+'M1')
        ca.caput('PMAC_BRICK_TEST:SendCmd', '#1J:{}'.format(move), datatype=ca.DBR_CHAR_STR)
        monitor.wait_for_one_move(30)
        # this should make axis 1 to {movement}mm
        self.assertEquals(ca.caget(P+'M1.RBV'), move)

        # now move the CS axis Height
        ca.caput(P+'X_CS3', cs_move, wait=True, timeout=30)
        self.assertAlmostEqual(ca.caget(P+'X_CS3.RBV'), cs_move)
        # Axis 1 should return to 0
        self.assertEquals(ca.caget(P+'M1.RBV'), 0)

    def test_very_slow_moves(self):
        """
            If Q7x is very different from Qx for an axis that is not in the kinematics or 1-1 maps
            then we used to get very slow motion because it gets included in the FRAX and hence
            the speed calculations for the entire move. Verify this no longer occurs.

        :return: None
        """
        command = '&3 Q75=90000'
        # set the appropriate coordinate system group
        ca.caput(PB+'COORDINATE_SYS_GROUP', 'MIXED', wait=True)
        # move affected axes to
        ca.caput([P+'M1', P+'M2', P+'M3', P+'M4'], [0, 0, 0, 0], wait=True, timeout=30)

        # make axis 5 demand radically wrong (not in a CS)
        ca.caput('PMAC_BRICK_TEST:SendCmd', command, datatype=ca.DBR_CHAR_STR)

        # make a CS move and make sure it happens quickly enough
        then = datetime.now()
        ca.caput(P+'X_CS3', 1, wait=True, timeout=5)
        self.assertEquals(ca.caget(P+'X_CS3.RBV'), 1)
        elapsed = datetime.now() - then
        self.assertFalse(elapsed.seconds > 2)