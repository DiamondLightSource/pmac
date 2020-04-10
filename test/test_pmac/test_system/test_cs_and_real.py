from unittest import TestCase
from test.brick.testbrick import TBrick, DECIMALS
from datetime import datetime

# These tests verify real and virtual motors interact correctly


class TestCsAndReal(TestCase):
    def test_real_moves_cs(self):
        """ check that virtual axes update as expected on real axis moves
        """
        tb = TBrick()
        tb.set_cs_group(tb.g3)

        tb.all_go([tb.jack1, tb.jack2], [5, 5])

        # check height(x) and angle(y) in CS3
        self.assertAlmostEqual(tb.height.pos, 5, DECIMALS)
        self.assertAlmostEqual(tb.angle.pos, 0, DECIMALS)

    def test_cs_moves_real(self):
        """ check that real axes update as expected on virtual axis moves
        """
        tb = TBrick()
        tb.set_cs_group(tb.g3)

        tb.height.go(5)

        self.assertAlmostEqual(tb.jack1.pos, 5, DECIMALS)
        self.assertAlmostEqual(tb.jack2.pos, 5, DECIMALS)

    def test_cs_demand_updates(self):
        """ checks that the internal Q7x demand is updated on a real axis move
        """
        tb = TBrick()
        tb.set_cs_group(tb.g3)

        tb.jack1.go(2)
        self.assertAlmostEqual(tb.height.pos, 1, DECIMALS)

        tb.angle.go(0)
        # if the Q77 demand for height axis X was not updated then it would return to prev pos 0
        self.assertAlmostEqual(tb.height.pos, 1, DECIMALS)
        self.assertAlmostEqual(tb.jack1.pos, 1, DECIMALS)
        self.assertAlmostEqual(tb.jack2.pos, 1, DECIMALS)

    def test_velocity_control(self):
        """
        velocity of coordinate system motors should be controlled by the motor record VELO
        for individual moves but should be limited by max speed in a motion program (ix16)
        for each real motor in the move
        """

        tb = TBrick()
        tb.set_cs_group(tb.g2)

        tb.height.set_speed(10)
        start = datetime.now()
        tb.height.go(10)
        elapsed = datetime.now() - start
        print(elapsed)
        self.assertAlmostEqual(tb.height.pos, 10, DECIMALS)
        self.assertTrue(elapsed.seconds < 2.5)

        tb.height.set_speed(2)
        start = datetime.now()
        tb.height.go(0)
        elapsed = datetime.now() - start
        print(elapsed)
        self.assertAlmostEqual(tb.height.pos, 0, DECIMALS)
        self.assertTrue(5 <= elapsed.seconds < 6.5)

        tb.height.set_speed(100)
        # set max motion program speed of jack 1 (axis 3) to 2mm/s
        tb.send_command("i316=2")
        start = datetime.now()
        tb.height.go(10)
        elapsed = datetime.now() - start
        print(elapsed)
        self.assertAlmostEqual(tb.height.pos, 10, DECIMALS)
        self.assertTrue(5 <= elapsed.seconds < 6.5)

        # set max motion program speed of jack 1 (axis 3) to 500mm/s
        tb.send_command("i316=500")
        start = datetime.now()
        tb.height.go(0)
        elapsed = datetime.now() - start
        print(elapsed)
        self.assertAlmostEqual(tb.height.pos, 0, DECIMALS)
        self.assertLess(elapsed.seconds, 3)
