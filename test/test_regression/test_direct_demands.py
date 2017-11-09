from unittest import TestCase
from cothread import catools as ca
from test.helper.movemonitor import MoveMonitor

P = 'BRICK1:'
PM = 'PMAC_BRICK_TEST:'
CS2 = 'BRICK1:CS2:'
CS3 = 'BRICK1:CS3:'
DD = ':DirectDemand'


class TestSwitchGroup(TestCase):
    def test_q_variables1(self):
        """
            ensure that switching between coordinate system groups does not cause extra motors to move
            when commanding a direct demand to a CS motor. This is a regression test for a problem that
            occurred because Q7x variables were not correctly set after a switch
        """
        ca.caput([PM+'MOTOR1', PM+'MOTOR2'], [0, 0], wait=True, timeout=30)
        monitor1 = MoveMonitor(PM+'MOTOR1')
        ca.caput(P+'COORDINATE_SYS_GROUP', 'MIXED')
        ca.caput(CS3 + 'M1' + DD, 3)
        ca.caput(CS3 + 'M2' + DD, 3)
        # since this is a CS move - wait for any of the motors
        monitor1.wait_for_one_move(30)

        self.assertAlmostEqual(ca.caget(PM+'MOTOR1.RBV'), 3)
        self.assertAlmostEqual(ca.caget(PM+'MOTOR2.RBV'), 3)

        monitor1.reset()
        ca.caput(P+'COORDINATE_SYS_GROUP', '1,2->A,B')
        ca.caput(CS2 + 'M1' + DD, 2)
        ca.caput(CS2 + 'M2' + DD, 2)
        monitor1.wait_for_one_move(30)

        self.assertAlmostEqual(ca.caget(PM+'MOTOR1.RBV'), 2)
        self.assertAlmostEqual(ca.caget(PM+'MOTOR2.RBV'), 2)

        monitor1.reset()
        # wait here is important since then next move will not see that there has been a switch if called too soon
        ca.caput(P+'COORDINATE_SYS_GROUP', 'MIXED', wait=True)
        ca.caput(CS3 + 'M1' + DD, 3)
        monitor1.wait_for_one_move(30)

        self.assertAlmostEqual(ca.caget(PM+'MOTOR1.RBV'), 3)
        # if make CS consistent has failed then this will also have moved back to pos 3
        self.assertAlmostEqual(ca.caget(PM+'MOTOR2.RBV'), 2)
