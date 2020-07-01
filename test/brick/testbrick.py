from cothread import catools as ca
from cothread import Sleep
from .axis import Axis
from .coordsys import CoordSys
from .controller import make_controller
from .trajectory import Trajectory

# number of decimals to use in verifying positions
# cant be very high on a clipper since it seems to
# set 'in position' a little early
DECIMALS = 2

brick_pv_root = bpr = 'BRICK1'
test_pv_root = "PMAC_BRICK_TEST"

brick_axes = {'m1': Axis(bpr, '{}:MOTOR1'.format(test_pv_root), 1, 0),
              'm2': Axis(bpr, '{}:MOTOR2'.format(test_pv_root), 2, 0),
              'm3': Axis(bpr, '{}:MOTOR3'.format(test_pv_root), 3, 0),
              'm4': Axis(bpr, '{}:MOTOR4'.format(test_pv_root), 4, 0),
              'm5': Axis(bpr, '{}:MOTOR5'.format(test_pv_root), 5, 0),
              'm6': Axis(bpr, '{}:MOTOR6'.format(test_pv_root), 6, 0),
              'm7': Axis(bpr, '{}:MOTOR7'.format(test_pv_root), 7, 0),
              'm8': Axis(bpr, '{}:MOTOR8'.format(test_pv_root), 8, 0),
              'A2': Axis(bpr, '{}CS2:A'.format(brick_pv_root), 1, 2),
              'B2': Axis(bpr, '{}CS2:B'.format(brick_pv_root), 2, 2),
              'A3': Axis(bpr, '{}CS3:A'.format(brick_pv_root), 1, 3),
              'B3': Axis(bpr, '{}CS3:B'.format(brick_pv_root), 2, 3),
              'X3': Axis(bpr, '{}CS3:X'.format(brick_pv_root), 7, 3),
              'Y3': Axis(bpr, '{}CS3:Y'.format(brick_pv_root), 8, 3)}

brick_groups = {'g1': '1,2->A,B',
                'g2': '3,4->I',
                'g3': 'MIXED CS3',
                'g4': 'MIXED CS2'}

brick_cs = {'cs2': CoordSys('{}:CS2'.format(brick_pv_root), 2, 'CS2'),
            'cs3': CoordSys('{}:CS3'.format(brick_pv_root), 3, 'CS3')}

MyBrick = make_controller(brick_axes, brick_cs, brick_groups, '{}:'.format(brick_pv_root))


class TBrick(MyBrick):
    # noinspection PyAttributeOutsideInit
    def startup(self, init=True):
        a = self.axes
        r = self.real_axes

        # axis aliases
        self.height = self.X3
        self.angle = self.Y3
        self.jack1 = self.m3
        self.jack2 = self.m4

        # useful aliases for beamline test authoring
        # makes the clipper look like i16 diff I can share tests
        # with i16
        self.cs2.kphi = self.cs2.M3
        self.cs2.kappa = self.cs2.M4
        self.cs2.Mu = self.cs2.M5
        self.cs2.gamma = self.cs2.M6
        self.cs2.ktheta = self.cs2.M7
        self.cs2.delta = self.cs2.M8
        self.kphi = self.m3
        self.kappa = self.m4
        self.Mu = self.m5
        self.gamma = self.m6
        self.ktheta = self.m7
        self.delta = self.m8

        # make sure deferred moves are not applied
        self.set_deferred_moves(False)
        self.cs2.set_deferred_moves(False)
        self.cs3.set_deferred_moves(False)

        self.trajectory = Trajectory(self.pv_root)

        if init:
            self.disable_polling(False)
            # reset all real mres
            mres_pvs = [axis.mres for axis in r.values()]
            mres_val = [0.001] * len(mres_pvs)
            ca.caput(mres_pvs, mres_val, wait=True, timeout=10)
            Sleep(2.0)
            ca.caput(["{}:M1.MRES PP".format(brick_pv_root), "{}:M2.MRES PP".format(brick_pv_root)], [1, 1], wait=True)
            Sleep(2.0)
            ca.caput(["{}:M1.PROC PP".format(brick_pv_root), "{}:M2.PROC PP".format(brick_pv_root)], [1, 1], wait=True)
            # higher resolution on 7 and 8 for clipper trajectories
            ca.caput(self.m7.mres, 0.001)
            ca.caput(self.m8.mres, 0.001)

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
            self.send_command("i117,8,100=10 i116,8,100=200")
            # reset all UEIP
            e_pvs = [axis.pv_use_encoder for axis in r.values()]
            e_val = [0] * len(mres_pvs)
            ca.caput(e_pvs, e_val, wait=True, timeout=10)

            # reset all offset
            offset_pvs = [axis.off for axis in a.values()]
            values = [0] * len(offset_pvs)
            ca.caput(offset_pvs, values, wait=True, timeout=4)

            # move all real motors to zero
            self.send_command(
                "#1 hmz #2 hmz #3 hmz #4 hmz #5 hmz #6 hmz #7 hmz #8 hmz "
            )
            demand_pvs = [axis.demand for axis in r.values()]
            values = [0] * len(demand_pvs)
            ca.caput(demand_pvs, values, wait=True, timeout=30)

            # choose a default coordinate system group
            self.set_cs_group(self.g3)

            # set wide limits
            lo_pvs = [axis.lo_limit for axis in r.values()]
            hi_pvs = [axis.hi_limit for axis in r.values()]
            lo_limits = [-10000] * len(lo_pvs)
            hi_limits = [10000] * len(hi_pvs)
            ca.caput(lo_pvs, lo_limits, wait=True, timeout=3)
            ca.caput(hi_pvs, hi_limits, wait=True, timeout=3)
