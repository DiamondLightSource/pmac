from unittest import TestCase
from test.brick.movemonitor import MoveMonitor
from test.brick.testbrick import TBrick, DECIMALS
from cothread import Sleep
from datetime import datetime


# These tests verify deferred moves
class TestDeferred(TestCase):
    def test_cs_defer(self):
        """
        check timed deferred moves and also individual cs moves
        """
        tb = TBrick()
        tb.set_cs_group(tb.g3)

        tb.cs3.set_deferred_moves(True)
        tb.cs3.set_move_time(3000)
        tb.height.go(5, wait=False)
        tb.angle.go(1, wait=False)

        # verify no motion yet
        Sleep(1)
        self.assertAlmostEqual(tb.height.pos, 0, DECIMALS)
        self.assertAlmostEqual(tb.angle.pos, 0, DECIMALS)

        m = MoveMonitor(tb.height.pv_root)
        start = datetime.now()
        tb.cs3.set_deferred_moves(False)
        m.wait_for_one_move(10)
        elapsed = datetime.now() - start
        print(elapsed)

        # verify motion
        self.assertAlmostEqual(tb.angle.pos, 1, DECIMALS)
        self.assertAlmostEqual(tb.height.pos, 5, DECIMALS)
        # todo this seems to take longer than I would expect - is this an issue?
        # todo YES - moves to real and virtual axes are taking an extra SLOW POLL
        # todo   before DMOV is set True
        self.assertTrue(3 <= elapsed.seconds < 6)

        # single axis move should be controlled by speed setting, not CsMoveTime
        start = datetime.now()
        tb.height.go(0)
        elapsed = datetime.now() - start
        print(elapsed)
        self.assertAlmostEqual(tb.height.pos, 0, DECIMALS)
        self.assertTrue(elapsed.seconds < 2)

    def test_real_defer(self):
        """
        check that real axes update as expected on virtual axis moves
        """
        for _ in range(4):  # retry for possible occasional race condition
            tb = TBrick()
            tb.set_cs_group(tb.g2)

            tb.set_deferred_moves(True)

            tb.jack1.go(5, wait=False)
            tb.jack2.go(4, wait=False)
            Sleep(1)

            # verify no motion yet
            self.assertAlmostEqual(tb.jack1.pos, 0, DECIMALS)
            self.assertAlmostEqual(tb.jack2.pos, 0, DECIMALS)

            m = MoveMonitor(tb.jack1.pv_root)
            start = datetime.now()
            tb.set_deferred_moves(False)
            m.wait_for_one_move(10)
            elapsed = datetime.now() - start
            print(elapsed)

            # verify motion
            self.assertAlmostEqual(tb.jack1.pos, 5, DECIMALS)
            self.assertAlmostEqual(tb.jack2.pos, 4, DECIMALS)
            self.assertTrue(elapsed.seconds < 4)
