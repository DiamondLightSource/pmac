from unittest import TestCase
from test.brick.movemonitor import MoveMonitor
from test.brick.testbrick import TBrick, DECIMALS


class TestSwitchGroup(TestCase):
    def test_q_variables1(self):
        """
            ensure that switching between coordinate system groups does not cause extra motors to move
            when commanding a direct demand to a CS motor. This is a regression test for a problem that
            occurred because Q7x variables were not correctly set after a switch
        """
        tb = TBrick()
        tb.set_cs_group(tb.g3)
        tb.all_go_direct([tb.m1, tb.m2], [3, 3])

        self.assertAlmostEqual(tb.m1.pos, 3, DECIMALS)
        self.assertAlmostEqual(tb.m2.pos, 3, DECIMALS)

        tb.set_cs_group(tb.g1)
        tb.all_go_direct([tb.A2, tb.B2], [2, 2])

        self.assertAlmostEqual(tb.m1.pos, 2, DECIMALS)
        self.assertAlmostEqual(tb.m2.pos, 2, DECIMALS)

        tb.set_cs_group(tb.g3)
        tb.m1.go_direct(3)

        self.assertAlmostEqual(tb.m1.pos, 3, DECIMALS)
        # if make CS consistent has failed then this will also have moved back to pos 3
        self.assertAlmostEqual(tb.m2.pos, 2, DECIMALS)
