from unittest import TestCase
from test.brick.movemonitor import MoveMonitor
from test.brick.testbrick import TBrick
from cothread import Sleep

# These tests verify that stop works for virtual and real motors


class TestStop(TestCase):
    def test_direct_mapped_real_stop(self):
        tb = TBrick()
        tb.set_cs_group(tb.g3)
        big_move = 1000

        monitor = MoveMonitor(tb.m1.pv_root)
        tb.m1.go(big_move, wait=False)
        Sleep(0.2)
        tb.m1.stop()
        monitor.wait_for_one_move(2)

        self.assertTrue(0 < tb.m1.pos < big_move)

    def test_kinematic_mapped_real_stop(self):
        tb = TBrick()
        tb.set_cs_group(tb.g3)
        big_move = 1000

        monitor = MoveMonitor(tb.m3.pv_root)
        tb.m3.go(big_move, wait=False)
        Sleep(0.2)
        tb.m3.stop()
        monitor.wait_for_one_move(2)

        self.assertTrue(0 < tb.m3.pos < big_move)

    def test_unmapped_real_stop(self):
        tb = TBrick()
        tb.set_cs_group(tb.g3)
        big_move = 1000

        monitor = MoveMonitor(tb.m8.pv_root)
        tb.m8.go(big_move, wait=False)
        Sleep(0.2)
        tb.m8.stop()
        monitor.wait_for_one_move(2)

        self.assertTrue(0 < tb.m8.pos < big_move)

    def test_virtual_stop(self):
        tb = TBrick()
        tb.set_cs_group(tb.g3)
        big_move = 1000

        monitor = MoveMonitor(tb.height.pv_root)
        tb.height.go(big_move, wait=False)
        Sleep(0.2)
        tb.height.stop()
        monitor.wait_for_one_move(2)

        self.assertTrue(0 < tb.height.pos < big_move)

    def test_real_stops_virtual(self):
        tb = TBrick()
        tb.set_cs_group(tb.g3)
        big_move = 1000

        monitor = MoveMonitor(tb.height.pv_root)
        tb.height.go(big_move, wait=False)
        Sleep(0.2)
        tb.jack1.stop()
        monitor.wait_for_one_move(2)

        self.assertTrue(0 < tb.height.pos < big_move)

    def test_virtual_abort(self):
        tb = TBrick()
        tb.set_cs_group(tb.g3)
        big_move = 1000

        monitor = MoveMonitor(tb.height.pv_root)
        tb.height.go(big_move, wait=False)
        Sleep(0.2)
        tb.cs3.abort()
        monitor.wait_for_one_move(2)

        self.assertTrue(0 < tb.height.pos < big_move)
