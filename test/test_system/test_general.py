from unittest import TestCase

import cothread.catools as ca
from datetime import datetime

from test.brick.testbrick import TestBrick, DECIMALS
from cothread import Sleep
from test.brick.movemonitor import MoveMonitor, MotorCallback


# These tests verify additional functions of the driver


class TestGeneral(TestCase):

    def test_auto_home(self):
        """ verify that autohome works as expected
        """
        tb = TestBrick()

        for axis in tb.real_axes.values():
            axis.go(1, False)

        ca.caput('PMAC_BRICK_TEST:HM:HMGRP', 'All')
        ca.caput('PMAC_BRICK_TEST:HM:HOME', 1)

        Sleep(1)

        for axis in tb.real_axes.values():
            self.assertAlmostEquals(axis.pos, 0)
