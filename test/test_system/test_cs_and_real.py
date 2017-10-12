from unittest import TestCase
from cothread import catools as ca
from test.helper.movemonitor import MoveMonitor
from cothread import Sleep

P = 'PMAC_BRICK_TEST:'
PB = 'PMAC_BRICK_TEST:GB1:'


# These tests verify real and virtual motors interact correctly

class TestCsAndReal(TestCase):
    def test_real_moves_cs(self):
        """ check that virtual axes update as expected on real axis moves
        """
        ca.caput(PB + 'COORDINATE_SYS_GROUP', 'MIXED')
        axis_jack1 = P + 'M3'
        axis_jack2 = P + 'M4'
        axis_height = P + 'X_CS3'
        axis_angle = P + 'Y_CS3'
        ca.caput([axis_jack1, axis_jack2, axis_height, axis_angle], [0, 0, 0, 0],
                 wait=True, timeout=30)
        ca.caput([axis_jack1, axis_jack2], [5, 5], wait=True, timeout=30)

        self.assertAlmostEqual(ca.caget(axis_height + '.RBV'), 5)
        self.assertAlmostEqual(ca.caget(axis_angle + '.RBV'), 0)

    def test_cs_moves_real(self):
        """ check that real axes update as expected on virtual axis moves
        """
        ca.caput(PB + 'COORDINATE_SYS_GROUP', 'MIXED')
        axis_jack1 = P + 'M3'
        axis_jack2 = P + 'M4'
        axis_height = P + 'X_CS3'
        axis_angle = P + 'Y_CS3'
        ca.caput([axis_jack1, axis_jack2, axis_height, axis_angle], [0, 0, 0, 0],
                 wait=True, timeout=30)
        ca.caput(axis_height, 5, wait=True, timeout=30)

        self.assertAlmostEqual(ca.caget(axis_jack1 + '.RBV'), 5)
        self.assertAlmostEqual(ca.caget(axis_jack2 + '.RBV'), 5)

    def test_cs_demand_updates(self):
        """ checks that the internal Q7x demand is updated on a real axis move
        """
        ca.caput(PB + 'COORDINATE_SYS_GROUP', 'MIXED')
        axis_jack1 = P + 'M3'
        axis_jack2 = P + 'M4'
        axis_height = P + 'X_CS3'
        axis_angle = P + 'Y_CS3'
        ca.caput([axis_jack1, axis_jack2, axis_height, axis_angle], [0, 0, 0, 0],
                 wait=True, timeout=30)
        ca.caput(axis_jack1, 2, wait=True, timeout=30)
        self.assertAlmostEqual(ca.caget(axis_height + '.RBV'), 1)
        ca.caput(axis_angle, 0, wait=True, timeout=30)
        # if the Q77 demand for height axis X was not updated then it would return to prev pos 0
        self.assertAlmostEqual(ca.caget(axis_height + '.RBV'), 1)
        self.assertAlmostEqual(ca.caget(axis_jack1 + '.RBV'), 1)
        self.assertAlmostEqual(ca.caget(axis_jack2 + '.RBV'), 1)