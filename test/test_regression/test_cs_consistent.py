from unittest import TestCase
from cothread import catools as ca
from test.helper.trajectory import Trajectory
from test.helper.movemonitor import MoveMonitor
from datetime import datetime

# these are test_regression tests for the set of issues that pmacController::makeCSDemandsConsistent
# is trying to solve

PM = 'PMAC_BRICK_TEST:'
PCS3 = 'BRICK1CS3:'
PCS2 = 'BRICK1CS2:'
PB = 'BRICK1:'
CS3 = 'BRICK1:CS3:'
DECIMALS = 6


class TestMakeCsConsistent(TestCase):

    def test_ffe_trajectory_scan(self):
        """
            Do we get fatal following error when doing a traj scan if one of the
            motors has a radically incorrect Q7x ?
        :return: None
        """
        command = '&3 Q71=90000'
        ca.caput(PB+'COORDINATE_SYS_GROUP', 'MIXED')
        ca.caput(PB+'ProfileCsName', 'BRICK1.CS3')
        ca.caput(CS3+'CsMoveTime', 0)
        # move affected axes to 0
        ca.caput([PM + 'MOTOR1', PM + 'MOTOR2', PM + 'MOTOR3', PM + 'MOTOR4'], [0, 0, 0, 0], wait=True,
                 timeout=30)
        # make axis 1 demand radically wrong
        ca.caput(PB+'SendCmd', command, datatype=ca.DBR_CHAR_STR)
        # this check is probably not required but leaving it in to verify reliability
        self. assertEquals(ca.caget(PB+'SendCmd', datatype=ca.DBR_CHAR_STR), command)

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
        ca.caput(CS3+'CsMoveTime', 0)
        # move affected axes to 0
        ca.caput([PM + 'MOTOR3', PM + 'MOTOR4'], [0, 0], wait=True, timeout=30)

        # do a CS move of Height
        ca.caput(PCS3 + 'X', 1, wait=True, timeout=5)

        # pretend that axis 3 moved by itself.
        monitor = MoveMonitor(PCS3 + 'X')
        ca.caput(PB+'SendCmd', '#3J:1000', datatype=ca.DBR_CHAR_STR)
        monitor.wait_for_one_move(5)
        # this should make Height 1.5mm
        self.assertAlmostEqual(ca.caget(PCS3 + 'X.RBV'), 1.5, DECIMALS)

        # now move Angle
        ca.caput(PCS3 + 'Y', 1, wait=True, timeout=5)
        self.assertAlmostEqual(ca.caget(PCS3 + 'Y.RBV'), 1, DECIMALS)
        # Height should return to 1
        self.assertAlmostEqual(ca.caget(PCS3 + 'X.RBV'), 1, DECIMALS)

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
        # the first CS move after a coord sys group switch clears the cached real motor positions
        # so this test must do that initial CS move here
        ca.caput(PCS3 + 'X', 0, wait=True, timeout=30)
        # the above is a little artificial and masks an very minor issue: any axis creep that occurs
        # between changing coordinate system mappings and the first CS move will be kept - fixing
        # this would require

        ca.caput(CS3+'CsMoveTime', 0)
        # move affected axes to start
        ca.caput([PM + 'MOTOR1', PM + 'MOTOR2', PM + 'MOTOR3', PM + 'MOTOR4'], [0, 0, 0, 0], wait=True,
                 timeout=30)

        # pretend that axis 1 moved by itself.
        monitor = MoveMonitor(PM + 'MOTOR1')
        ca.caput(PB+'SendCmd', '#1J:{}'.format(move), datatype=ca.DBR_CHAR_STR)
        monitor.wait_for_one_move(30)
        # this should make axis 1 to {move}mm
        self.assertEquals(ca.caget(PM + 'MOTOR1.RBV'), move)

        # now move the CS axis Height
        ca.caput(PCS3 + 'X', cs_move, wait=True, timeout=30)
        self.assertAlmostEqual(ca.caget(PCS3 + 'X.RBV'), cs_move, DECIMALS)
        # Axis 1 should return to 0
        self.assertAlmostEqual(ca.caget(PM + 'MOTOR1.RBV'), 0, DECIMALS)

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
        ca.caput(CS3+'CsMoveTime', 0)
        # move affected axes to 0
        ca.caput([PM + 'MOTOR1', PM + 'MOTOR2', PM + 'MOTOR3', PM + 'MOTOR4'], [0, 0, 0, 0],
                 wait=True, timeout=30)

        # make axis 5 demand radically wrong (not in a CS)
        ca.caput(PB+'SendCmd', command, datatype=ca.DBR_CHAR_STR)

        # make a CS move and make sure it happens quickly enough
        then = datetime.now()
        ca.caput(PCS3 + 'X', 1, wait=True, timeout=5)
        self.assertAlmostEqual(ca.caget(PCS3 + 'X.RBV'), 1, DECIMALS)
        elapsed = datetime.now() - then
        self.assertFalse(elapsed.seconds > 2)

    def test_switch_cs_group(self):
        ca.caput([PM+'MOTOR1', PM+'MOTOR2'], [0, 0], wait=True, timeout=30)
        ca.caput(PB + 'COORDINATE_SYS_GROUP', 'MIXED')
        ca.caput([PCS3+'A', PCS3+'B'], [3, 3], wait=True)

        self.assertAlmostEqual(ca.caget(PM+'MOTOR1.RBV'), 3)
        self.assertAlmostEqual(ca.caget(PM+'MOTOR2.RBV'), 3)

        ca.caput(PB + 'COORDINATE_SYS_GROUP', '1,2->A,B')
        ca.caput([PCS2+'A', PCS2+'B'], [2, 2], wait=True)

        # scaling of 1000 and 100 or A and B on CS2 axes gives 20, 200 on axes 1, 2
        self.assertAlmostEqual(ca.caget(PM+'MOTOR1.RBV'), 20)
        self.assertAlmostEqual(ca.caget(PM+'MOTOR2.RBV'), 200)

        # wait here is important since then next move will not see that there has been a switch if called too soon
        ca.caput(PB + 'COORDINATE_SYS_GROUP', 'MIXED', wait=True)
        ca.caput(PCS3+'A', 3, wait=True)

        self.assertAlmostEqual(ca.caget(PM+'MOTOR1.RBV'), 3)
        # if make CS consistent has failed then this will also have moved back to pos 3
        self.assertAlmostEqual(ca.caget(PM+'MOTOR2.RBV'), 200)
