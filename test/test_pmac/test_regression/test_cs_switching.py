from unittest import TestCase
from test.brick.trajectory import Trajectory
from test.brick.movemonitor import MoveMonitor
from datetime import datetime
from test.brick.testbrick import TBrick, DECIMALS
from cothread import Sleep
import cothread.catools as ca

# these are test_regression tests for the set of issues that pmacController::makeCSDemandsConsistent
# is trying to solve


class TestCsSwitching(TestCase):
    def test_individual_switch(self):
        """
            Vmxi issue: switching coordinate system drive mappings and then trying to move cs motors
            immediately occasionally gets CS runtime error 'no axes in CS'
        """
        tb = TBrick()
        tb.set_cs_group(tb.g3)

        tb.m3.set_cs_port("CS3")
        tb.m4.set_cs_port("CS3")

        for i in range(4):
            print("trying CS switch {}".format(i))
            tb.m3.set_cs_assignment("")
            tb.m4.set_cs_assignment("")
            tb.all_go([tb.m3, tb.m4], [3, 3])
            self.assertAlmostEqual(tb.m3.pos, 3, DECIMALS)
            self.assertAlmostEqual(tb.m4.pos, 3, DECIMALS)

            tb.m3.set_cs_assignment("I")
            tb.m4.set_cs_assignment("I")
            tb.height.go_direct(1)

            self.assertEquals(tb.height.alarm, 0)
            self.assertAlmostEqual(tb.m3.pos, 1, DECIMALS)

    def test_group_switch(self):
        """ This issue showed up after adding parameter library locking on broker polling.
            switching cs group caused motion due to record processing in
            pmacDirectMotor.template"""
        tb = TBrick()

        for i in range(2):
            print("trying CS switch {}".format(i))
            tb.set_cs_group(tb.g3)
            tb.height.go(2)
            self.assertAlmostEqual(tb.jack1.pos, 2, DECIMALS)
            self.assertAlmostEqual(tb.jack2.pos, 2, DECIMALS)
            tb.set_cs_group(tb.g1)
            tb.all_go([tb.jack1, tb.jack2], [5, 5])
            self.assertAlmostEqual(tb.jack1.pos, 5, DECIMALS)
            self.assertAlmostEqual(tb.jack2.pos, 5, DECIMALS)
