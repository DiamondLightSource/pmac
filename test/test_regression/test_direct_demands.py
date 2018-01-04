from unittest import TestCase
from test.helper.movemonitor import MoveMonitor
from test.helper.testbrick import TestBrick, DECIMALS


class TestSwitchGroup(TestCase):
    def test_q_variables1(self):
        """
            ensure that switching between coordinate system groups does not cause extra motors to move
            when commanding a direct demand to a CS motor. This is a regression test for a problem that
            occurred because Q7x variables were not correctly set after a switch
        """
        tb = TestBrick()
        tb.set_cs_group(tb.g3)
        monitor1 = MoveMonitor(tb.m1.pv_root)
        tb.m1.go_direct(3, wait=False)
        tb.m2.go_direct(3, wait=False)
        # since this is a CS move - wait for any of the motors
        monitor1.wait_for_one_move(30)

        self.assertAlmostEqual(tb.m1.pos, 3, DECIMALS)
        self.assertAlmostEqual(tb.m2.pos, 3, DECIMALS)

        monitor1.reset()
        tb.set_cs_group(tb.g1)
        tb.A2.go_direct(2, wait=False)
        tb.B2.go_direct(2, wait=False)
        monitor1.wait_for_one_move(30)

        self.assertAlmostEqual(tb.m1.pos, 20, DECIMALS)
        self.assertAlmostEqual(tb.m2.pos, 200, DECIMALS)

        monitor1.reset()
        tb.set_cs_group(tb.g3)
        tb.m1.go_direct(3, wait=False)
        monitor1.wait_for_one_move(30)

        self.assertAlmostEqual(tb.m1.pos, 3, DECIMALS)
        # if make CS consistent has failed then this will also have moved back to pos 3
        self.assertAlmostEqual(tb.m2.pos, 200, DECIMALS)
