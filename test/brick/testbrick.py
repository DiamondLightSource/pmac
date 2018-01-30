from cothread import catools as ca
from axis import Axis
from coordsys import CoordSys
from controller import make_controller

# number of decimals to use in verifying positions
# cant be very high on a clipper since it seems to
# set 'in position' a little early
DECIMALS = 1

brick_axes = {'m1': Axis('PMAC_BRICK_TEST:MOTOR1', 1, 0),
              'm2': Axis('PMAC_BRICK_TEST:MOTOR2', 2, 0),
              'm3': Axis('PMAC_BRICK_TEST:MOTOR3', 3, 0),
              'm4': Axis('PMAC_BRICK_TEST:MOTOR4', 4, 0),
              'm5': Axis('PMAC_BRICK_TEST:MOTOR5', 5, 0),
              'm6': Axis('PMAC_BRICK_TEST:MOTOR6', 6, 0),
              'm7': Axis('PMAC_BRICK_TEST:MOTOR7', 7, 0),
              'm8': Axis('PMAC_BRICK_TEST:MOTOR8', 8, 0),
              'A2': Axis('BRICK1CS2:A', 1, 2),
              'B2': Axis('BRICK1CS2:B', 2, 2),
              'A3': Axis('BRICK1CS3:A', 1, 3),
              'B3': Axis('BRICK1CS3:B', 2, 3),
              'X3': Axis('BRICK1CS3:X', 7, 3),
              'Y3': Axis('BRICK1CS3:Y', 8, 3)}

brick_groups = {'g1': '1,2->A,B',
                'g2': '3,4->I',
                'g3': 'MIXED'}

brick_cs = {'cs1': CoordSys('BRICK1:CS1', 1, 'CS1'),
            'cs2': CoordSys('BRICK1:CS2', 2, 'CS2'),
            'cs3': CoordSys('BRICK1:CS3', 3, 'CS3')}

brick_pv_root = 'BRICK1:'

MyBrick = make_controller(brick_axes, brick_cs, brick_groups, brick_pv_root)


class TestBrick(MyBrick):
    # noinspection PyAttributeOutsideInit
    def startup(self):
        a = self.axes
        r = self.real_axes

        # axis aliases
        self.height = self.X3
        self.angle = self.Y3
        self.jack1 = self.m3
        self.jack2 = self.m4

        # make sure deferred moves are not applied
        self.set_deferred_moves(False)
        self.cs1.set_deferred_moves(False)
        self.cs2.set_deferred_moves(False)
        self.cs3.set_deferred_moves(False)

        # make all motors fast to speed up tests
        # (and encourage race conditions)
        acc_pvs = [axis.acc for axis in a.values()]
        values = [0.01] * len(acc_pvs)
        ca.caput(acc_pvs, values, wait=True, timeout=3)
        velo_pvs = [axis.vmax for axis in a.values()]
        velo_pvs += [axis.velo for axis in a.values()]
        values = [150] * len(velo_pvs)
        ca.caput(velo_pvs, values, wait=True, timeout=3)

        # also make motors speed, acceleration in motion programs fast
        self.send_command('i117,8,100=10 i116,8,100=200')

        # move all real motors to zero
        demand_pvs = [axis.demand for axis in r.values()]
        values = [0] * len(demand_pvs)
        ca.caput(demand_pvs, values, wait=True, timeout=30)

        # choose a default coordinate system group
        self.set_cs_group(self.g3)
