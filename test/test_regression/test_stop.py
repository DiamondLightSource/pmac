from unittest import TestCase
from test.brick.movemonitor import MoveMonitor
from test.brick.testbrick import TestBrick
from cothread import Sleep

# These tests verify that stop works for virtual and real motors


class TestStop(TestCase):

    def test_real_stop(self):
        tb = TestBrick()
        tb.set_cs_group(tb.g3)
        big_move = 1000

        monitor = MoveMonitor(tb.m1.pv_root)
        tb.m1.go(big_move, wait=False)
        Sleep(1)
        tb.m1.stop()
        monitor.wait_for_one_move(2)

        self.assertTrue(tb.m1.pos < big_move)

    def test_virtual_stop(self):
        tb = TestBrick()
        tb.set_cs_group(tb.g3)
        big_move = 1000

        monitor = MoveMonitor(tb.height.pv_root)
        tb.height.go(big_move, wait=False)
        Sleep(1)
        tb.height.stop()
        monitor.wait_for_one_move(2)

        self.assertTrue(tb.height.pos < big_move)





