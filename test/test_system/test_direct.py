from unittest import TestCase

import cothread.catools as ca
from datetime import datetime

from test.brick.testbrick import TestBrick, DECIMALS
from cothread import Sleep
from test.brick.movemonitor import MoveMonitor, MotorCallback


# These tests verify real and virtual motors interact correctly


class TestDirect(TestCase):

    def test_quick(self):
        # prove that timings work OK when not using the TesBrick class
        ca.caput('BRICK1:M5.VMAX', 10, wait=True, timeout=60)
        ca.caput('BRICK1:M5.VELO', 10, wait=True, timeout=60)
        ca.caput('BRICK1:M5', 0.1, wait=True, timeout=60)

        ca.caput('BRICK1:M5:DirectDemand', 10, wait=True, timeout=60)
        start = datetime.now()
        ca.caput('BRICK1:M5:DirectDemand', 0, wait=True, timeout=60)
        elapsed = datetime.now() - start
        print(elapsed)
        self.assertLess(elapsed.seconds, 1.5)

        start = datetime.now()
        ca.caput('BRICK1:M5:DirectDemand', 1, wait=True, timeout=60)
        elapsed = datetime.now() - start
        print(elapsed)
        self.assertLess(elapsed.seconds, 1.5)

    def test_real_moves_cs(self):
        """ check that virtual axes update as expected on real axis moves
        """
        tb = TestBrick()
        tb.set_cs_group(tb.g3)

        # tb.jack1.go_direct(5)
        # tb.jack2.go_direct(5)
        tb.all_go_direct([tb.jack1, tb.jack2], [5, 5])

        # check height(x) and angle(y) in CS3
        self.assertAlmostEqual(tb.height.pos, 5, DECIMALS)
        self.assertAlmostEqual(tb.angle.pos, 0, DECIMALS)

    def test_cs_moves_real(self):
        """ check that real axes update as expected on virtual axis moves
        """
        tb = TestBrick()
        tb.set_cs_group(tb.g3)

        tb.height.go_direct(5)

        self.assertAlmostEqual(tb.jack1.pos, 5, DECIMALS)
        self.assertAlmostEqual(tb.jack2.pos, 5, DECIMALS)

    def test_cs_demand_updates(self):
        """ checks that the internal Q7x demand is updated on a real axis move
        """
        tb = TestBrick()
        tb.set_cs_group(tb.g3)

        tb.jack1.go_direct(2)
        self.assertAlmostEqual(tb.height.pos, 1, DECIMALS)

        tb.angle.go_direct(0)
        # if the Q77 demand for height axis X was not updated then it would return to prev pos 0
        self.assertAlmostEqual(tb.height.pos, 1, DECIMALS)
        self.assertAlmostEqual(tb.jack1.pos, 1, DECIMALS)
        self.assertAlmostEqual(tb.jack2.pos, 1, DECIMALS)

    def test_velocity_real(self):
        """ verify the speed of direct real axes
        """

        tb = TestBrick()
        tb.set_cs_group(tb.g1)

        # direct demand axes will go at the speed of the most recent standard motor record
        # move. This is not ideal but direct axes are really intended for use with deferred moves
        tb.m5.set_speed(10)
        tb.m5.go(0.1)
        start = datetime.now()
        tb.m5.go_direct(10)
        elapsed = datetime.now() - start
        print(elapsed)
        self.assertAlmostEqual(tb.m5.pos, 10, DECIMALS)
        self.assertLess(elapsed.seconds, 2.5)

        tb.m5.go(0)
        tb.m5.set_speed(2)
        tb.m5.go(0.1)
        start = datetime.now()
        tb.m5.go_direct(10)
        elapsed = datetime.now() - start
        print(elapsed)
        self.assertAlmostEqual(tb.m5.pos, 10, DECIMALS)
        self.assertTrue(5 <= elapsed.seconds < 7)

    def test_velocity_cs(self):
        """ verify the speed of direct virtual axes
        """
        tb = TestBrick()
        tb.set_cs_group(tb.g3)

        tb.height.set_speed(2)
        tb.height.go(0.1)

        start = datetime.now()
        tb.height.go_direct(10)
        elapsed = datetime.now() - start
        print(elapsed)
        self.assertAlmostEqual(tb.height.pos, 10, DECIMALS)
        self.assertTrue(5 < elapsed.seconds < 7)

        tb.height.set_speed(100)
        tb.height.go(0)
        # set max motion program speed of jack 1 (axis 3) to 2mm/s
        # cts/msec is the same as mm/s if mres = .001
        tb.send_command('i316=2')
        start = datetime.now()
        tb.height.go(10)
        elapsed = datetime.now() - start
        print(elapsed)
        self.assertAlmostEqual(tb.height.pos, 10, DECIMALS)
        # the faster velocity should be overridden by max program speed of jack 1
        self.assertTrue(5 < elapsed.seconds < 7)

        # set max motion program speed of jack 1 (axis 3) to 500mm/s
        tb.send_command('i316=500')
        tb.height.go(0)
        start = datetime.now()
        tb.height.go(10)
        elapsed = datetime.now() - start
        print(elapsed)
        # this time the motor speed restriction does not apply and the motion is fast
        self.assertAlmostEqual(tb.height.pos, 10, DECIMALS)
        self.assertTrue(elapsed.seconds < 1.5)

    def test_direct_deferred_moves_cs(self):
        """ verify coordinated deferred direct moves work in quick succession
            i16 diffractometer style
        """
        tb = TestBrick()
        tb.set_cs_group(tb.g3)
        waiter = MotorCallback()

        for iteration in range(2):
            for height in range(40, 1, -1):
                angle = height / 20.0
                # todo rapidly mixing direct and standard moves seems to cause issues
                # todo this needs investigation
                tb.all_go_direct([tb.jack1, tb.jack2], [0, 0])
                self.assertAlmostEqual(tb.height.pos, 0, DECIMALS)
                self.assertAlmostEqual(tb.angle.pos, 0, DECIMALS)

                tb.cs3.set_deferred_moves(True)

                waiter.reset_done()
                tb.height.go_direct(height, callback=waiter.moves_done, wait=False)
                tb.angle.go_direct(angle, wait=False)
                Sleep(.2)

                # verify no motion yet
                self.assertAlmostEqual(tb.height.pos, 0, DECIMALS)
                self.assertAlmostEqual(tb.angle.pos, 0, DECIMALS)

                start = datetime.now()
                tb.cs3.set_deferred_moves(False)
                waiter.wait_for_done()
                elapsed = datetime.now() - start
                print("Iteration {}. Direct Deferred Coordinated to height {} took {}".format(iteration, height, elapsed))

                # verify motion
                self.assertAlmostEqual(tb.height.pos, height, DECIMALS)
                self.assertAlmostEqual(tb.angle.pos * 10, angle * 10, DECIMALS)

    def test_direct_deferred_moves(self):
        """ verify real motor deferred direct moves work in quick succession
            i16 diffractometer style
        """
        # this test cannot work due to lack of put with callback support in real
        # motors when when deferred -- todo fix this !
        return
        tb = TestBrick()
        tb.set_cs_group(tb.g3)
        monitor = MoveMonitor(tb.jack1.pv_root)

        for iteration in range(2):
            for height1 in range(40, 1, -1):
                height2 = height1 / 2.0
                # todo mixing direct and standard moves seems to cause issues
                # tb.all_go([tb.jack1, tb.jack2], [0, 0])
                tb.all_go_direct([tb.jack1, tb.jack2], [0, 0])
                self.assertAlmostEqual(tb.jack1.pos, 0, DECIMALS)
                self.assertAlmostEqual(tb.jack2.pos, 0, DECIMALS)

                tb.set_deferred_moves(True)

                # todo put with callback for deferred real motors is not yet working
                # todo this is true of both direct and normal PVs
                # using MoveMonitor instead of wait_for_done until this is fixed
                # self.reset_done()
                monitor.reset()
                # tb.jack1.go_direct(height1, callback=self.moves_done, wait=False)
                tb.jack1.go_direct(height1, wait=False)
                tb.jack2.go_direct(height2, wait=False)
                Sleep(.2)

                # verify no motion yet
                self.assertAlmostEqual(tb.jack1.pos, 0, DECIMALS)
                self.assertAlmostEqual(tb.jack2.pos, 0, DECIMALS)

                start = datetime.now()
                tb.set_deferred_moves(False)
                Sleep(1)
                monitor.wait_for_one_move(10)
                # self.wait_for_done()
                elapsed = datetime.now() - start
                print("Iteration {}. Direct Deferred real motors to height {} took {}".format(iteration, height1, elapsed))

                # verify motion
                self.assertAlmostEqual(tb.jack1.pos, height1, DECIMALS)
                self.assertAlmostEqual(tb.jack2.pos, height2, DECIMALS)
