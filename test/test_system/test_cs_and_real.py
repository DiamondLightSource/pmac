from unittest import TestCase
from test.helper.testbrick import TestBrick, DECIMALS


# These tests verify real and virtual motors interact correctly
class TestCsAndReal(TestCase):
    def test_real_moves_cs(self):
        """ check that virtual axes update as expected on real axis moves
        """
        tb = TestBrick()
        tb.set_cs_group(tb.g3)

        tb.all_go([tb.jack1, tb.jack2], [5, 5])

        # check height(x) and angle(y) in CS3
        self.assertAlmostEqual(tb.height.pos, 5, DECIMALS)
        self.assertAlmostEqual(tb.angle.pos, 0, DECIMALS)

    def test_cs_moves_real(self):
        """ check that real axes update as expected on virtual axis moves
        """
        tb = TestBrick()
        tb.set_cs_group(tb.g3)

        tb.height.go(5)

        self.assertAlmostEqual(tb.jack1.pos, 5, DECIMALS)
        self.assertAlmostEqual(tb.jack2.pos, 5, DECIMALS)

    def test_cs_demand_updates(self):
        """ checks that the internal Q7x demand is updated on a real axis move
        """
        tb = TestBrick()
        tb.set_cs_group(tb.g3)

        tb.jack1.go(2)
        self.assertAlmostEqual(tb.height.pos, 1, DECIMALS)

        tb.angle.go(0)
        # if the Q77 demand for height axis X was not updated then it would return to prev pos 0
        self.assertAlmostEqual(tb.height.pos, 1, DECIMALS)
        self.assertAlmostEqual(tb.jack1.pos, 1, DECIMALS)
        self.assertAlmostEqual(tb.jack2.pos, 1, DECIMALS)
