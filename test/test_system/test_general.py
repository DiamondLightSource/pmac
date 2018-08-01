from unittest import TestCase

import cothread.catools as ca
from datetime import datetime

from test.brick.testbrick import TestBrick, DECIMALS
from cothread import Sleep
from test.brick.movemonitor import MoveMonitor, MotorCallback
import pytest
import os


# These tests verify additional functions of the driver


class TestGeneral(TestCase):

    @pytest.mark.skipif(os.environ['PPMAC'] == 'TRUE', reason="not supported on PPMAC yet")
    def test_auto_home(self):
        """ verify that autohome works as expected
        """
        tb = TestBrick()

        for axis in tb.real_axes.values():
            axis.go(1, False)

        ca.caput('PMAC_BRICK_TEST:HM:HMGRP', 'All')
        ca.caput('PMAC_BRICK_TEST:HM:HOME', 1)

        Sleep(1)

        # ensure good state if homing failed
        ca.caput('PMAC_BRICK_TEST:HM:ABORT.PROC', 1)

        for axis in tb.real_axes.values():
            self.assertAlmostEquals(axis.pos, 0)

    def test_auto_home_readonly(self):
        """ verify that auto home makes the motor records read only
            so that soft limits and user intervention cannot
            interfere with homing
        """
        tb = TestBrick()

        ca.caput('PMAC_BRICK_TESTX:HM:HMGRP', 'All')
        ca.caput('PMAC_BRICK_TESTX:HM:HOME', 1)

        tb.poll_all_now()
        tb.jack1.go(20)
        Sleep(0.5)

        ca.caput('PMAC_BRICK_TESTX:HM:ABORT.PROC', 1)
        self.assertAlmostEqual(tb.jack1.pos, 0, DECIMALS)

    def test_cs_feedrate_protection(self):
        """ verify that feedrate protection works for only those CS we have configured
        """
        tb = TestBrick()

        problem = ca.caget("BRICK1:FEEDRATE_PROBLEM_RBV")
        self.assertEquals(problem, 0)

        # create a feedrate problem by manually setting a configured feedrate
        tb.send_command("&2%50")
        tb.poll_all_now()
        problem = ca.caget("BRICK1:FEEDRATE_PROBLEM_RBV")
        self.assertEquals(problem, 1)

        # use driver feature to reset All feedrates
        ca.caput("BRICK1:FEEDRATE", 100, wait=True)
        tb.poll_all_now()
        problem = ca.caget("BRICK1:FEEDRATE_PROBLEM_RBV")
        self.assertEquals(problem, 0)

        # create a feedrate problem by manually setting another configured feedrate
        tb.send_command("&3%50")
        tb.poll_all_now()
        problem = ca.caget("BRICK1:FEEDRATE_PROBLEM_RBV")
        self.assertEquals(problem, 1)

        # use driver feature to reset All feedrates
        ca.caput("BRICK1:FEEDRATE", 100, wait=True)
        tb.poll_all_now()
        problem = ca.caget("BRICK1:FEEDRATE_PROBLEM_RBV")
        self.assertEquals(problem, 0)

        # check that non-configured CS has no effect
        tb.send_command("&4%50")
        tb.poll_all_now()
        problem = ca.caget("BRICK1:FEEDRATE_PROBLEM_RBV")
        self.assertEquals(problem, 0)
