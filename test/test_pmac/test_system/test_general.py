from unittest import TestCase

import cothread.catools as ca
from datetime import datetime

from test.brick.testbrick import TBrick, DECIMALS, brick_pv_root, test_pv_root
from cothread import Sleep
from test.brick.movemonitor import MoveMonitor, MotorCallback
import pytest
import os


# These tests verify additional functions of the driver

class TestGeneral(TestCase):
    def _test_auto_home(self):
        """ verify that autohome works as expected
        """
        tb = TBrick()

        for axis in tb.real_axes.values():
            axis.go(1, False)
        Sleep(0.5)
        ca.caput('{}:HM:HMGRP'.format(test_pv_root), 'All')
        ca.caput('{}:HM:HOME'.format(test_pv_root), 1, timeout=15, wait=True)
        Sleep(0.5)

        # ensure good state if homing failed
        ca.caput('{}:HM:ABORT.PROC'.format(test_pv_root), 1)

        for axis in tb.real_axes.values():
            self.assertAlmostEqual(axis.pos, 0)

    def _test_auto_home_readonly(self):
        """ verify that auto home makes the motor records read only
            so that soft limits and user intervention cannot
            interfere with homing
        """
        tb = TBrick()

        ca.caput('{}X:HM:HMGRP'.format(test_pv_root), 'All')
        ca.caput('{}X:HM:HOME'.format(test_pv_root), 1)

        tb.poll_all_now()
        tb.jack1.go(20)
        Sleep(1)

        ca.caput('{}X:HM:ABORT.PROC'.format(test_pv_root), 1)
        self.assertAlmostEqual(tb.jack1.pos, 0, DECIMALS)

    def test_cs_feedrate_protection(self):
        """ verify that feedrate protection works for only those CS we have configured
        """
        tb = TBrick()

        try:
            problem = ca.caget("{}:FEEDRATE_PROBLEM_RBV".format(brick_pv_root))
            self.assertEquals(problem, 0)

            # create a feedrate problem by manually setting a configured feedrate
            tb.send_command("&2%50")
            tb.poll_all_now()
            Sleep(2)  # Todo, does requiring this represent an issue?
            problem = ca.caget("{}:FEEDRATE_PROBLEM_RBV".format(brick_pv_root))
            self.assertEquals(problem, 1)

            # use driver feature to reset All feedrates
            ca.caput("{}:FEEDRATE".format(brick_pv_root), 100, wait=True)
            tb.poll_all_now()
            Sleep(2)
            problem = ca.caget("{}:FEEDRATE_PROBLEM_RBV".format(brick_pv_root))
            self.assertEquals(problem, 0)

            # create a feedrate problem by manually setting another configured feedrate
            tb.send_command("&3%50")
            tb.poll_all_now()
            Sleep(2)
            problem = ca.caget("{}:FEEDRATE_PROBLEM_RBV".format(brick_pv_root))
            self.assertEquals(problem, 1)

            # use driver feature to reset All feedrates
            ca.caput("{}:FEEDRATE".format(brick_pv_root), 100, wait=True)
            tb.poll_all_now()
            Sleep(2)
            problem = ca.caget("{}:FEEDRATE_PROBLEM_RBV".format(brick_pv_root))
            self.assertEquals(problem, 0)

            # check that non-configured CS has no effect
            tb.send_command("&4%50")
            tb.poll_all_now()
            Sleep(2)
            problem = ca.caget("{}:FEEDRATE_PROBLEM_RBV".format(brick_pv_root))
            self.assertEquals(problem, 0)
        finally:
            ca.caput("{}:FEEDRATE".format(brick_pv_root), 100, wait=True)
