from unittest import TestCase
from test.brick.trajectory import Trajectory
from test.brick.movemonitor import MoveMonitor
from datetime import datetime
from test.brick.testbrick import TestBrick, DECIMALS
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
        tb = TestBrick()
        tb.set_cs_group(tb.g3)

        tb.m3.set_cs_port('CS3')
        tb.m4.set_cs_port('CS3')

        for i in range(30):
            print('trying CS switch {}'.format(i))
            tb.m3.set_cs_assignment('')
            tb.m4.set_cs_assignment('')
            tb.m3.set_cs_assignment('I')
            tb.m4.set_cs_assignment('I')
            tb.height.go_direct(0)
            tb.height.go_direct(.5)

            self.assertEquals(tb.height.alarm, 0)


