from unittest import TestCase
from cothread import catools as ca
from test.helper.movemonitor import MoveMonitor
from cothread import Sleep

P = 'PMAC_BRICK_TEST:'
PB = 'PMAC_BRICK_TEST:GB1:'

# These tests verify that stop works for virtual and real motors


class TestStop(TestCase):

    def test_real_stop(self):
        ca.caput(PB+'COORDINATE_SYS_GROUP', 'MIXED')
        axis1 = P + 'M1'
        ca.caput(axis1,  0, wait=True, timeout=30)
        monitor = MoveMonitor(axis1)
        ca.caput(axis1, 1000)
        Sleep(1)
        ca.caput(axis1+'.STOP', 1)
        monitor.wait_for_one_move(2)

        self.assertTrue(ca.caget(axis1+'.RBV') < 1000)

    def test_virtual_stop(self):
        ca.caput(PB+'COORDINATE_SYS_GROUP', 'MIXED')
        axis1 = P + 'X_CS3'
        ca.caput(axis1,  0, wait=True, timeout=30)
        monitor = MoveMonitor(axis1)
        ca.caput(axis1, 1000)
        Sleep(1)
        ca.caput(axis1+'.STOP', 1)
        monitor.wait_for_one_move(2)

        self.assertTrue(ca.caget(axis1+'.RBV') < 1000)





