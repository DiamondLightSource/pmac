from cothread import catools as ca

# number of decimals to use in verifying positions
# cant be very high on a clipper since it seems to
# set 'in position' a little early
DECIMALS = 2


class Axis:
    def __init__(self, pv_root, axis_no, cs_no):
        self.pv_root = pv_root
        self.axis_no = axis_no
        self.cs_no = cs_no
        self.demand = pv_root
        self.rbv = pv_root + '.RBV'
        self.acc = pv_root + '.ACCL'
        self.stop_pv = pv_root + '.STOP'
        self.velo = pv_root + '.VELO'
        self.vmax = pv_root + '.VMAX'
        if cs_no > 0:
            cs_name = 'CS{}:'.format(cs_no)
        else:
            cs_name = ''
        self.direct_demand = 'BRICK1:{}M{}:DirectDemand'.format(cs_name, axis_no)
        # the following helps to avoid waiting for a timeout when the IOC is down
        ca.caget(self.direct_demand, timeout=.1)

    @property
    def pos(self):
        return ca.caget(self.rbv)

    def go(self, position, wait=True):
        ca.caput(self.demand, position, wait=wait, timeout=60)

    def go_direct(self, position, wait=True):
        ca.caput(self.direct_demand, position, wait=wait, timeout=60)

    def stop(self):
        ca.caput(self.stop_pv, 1, wait=True)

    def set_speed(self, speed):
        ca.caput(self.vmax, speed, wait=True)
        ca.caput(self.velo, speed, wait=True)

    def set_acceleration(self, acc):
        ca.caput(self.acc, acc, wait=True)


class CoordSys:
    def __init__(self, pv_root, cs_no, port):
        self.pv_root = pv_root
        self.cs_no = cs_no
        self.port = port
        self.move_time = pv_root + ':CsMoveTime'
        self.defer = pv_root + ':DeferMoves'

    def set_move_time(self, move_time):
        ca.caput(self.move_time, move_time, wait=True)

    def set_deferred_moves(self, defer):
        ca.caput(self.defer, defer, wait=True)


def add_trajectory_attributes(cls):
    """
        add items for controlling trajectory scans on the test IOC
    """
    cls.pv_trajectory_cs = 'BRICK1:ProfileCsName'
    return cls


def add_brick_attributes(cls):
    """
        add a dictionary of axes and CS groups to the class and also add each entry in
        the dictionary as an attribute of the class
    """
    cls.axes = {'m1': Axis('PMAC_BRICK_TEST:MOTOR1', 1, 0),
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

    cls.real_axes = {k: v for k, v in cls.axes.items() if k.startswith('m')}

    cls.groups = {'g1': '1,2->A,B',
                  'g2': '3,4->I',
                  'g3': 'MIXED'}

    cls.cs = {'cs1': CoordSys('BRICK1:CS1', 1, 'CS1'),
              'cs2': CoordSys('BRICK1:CS2', 2, 'CS2'),
              'cs3': CoordSys('BRICK1:CS3', 3, 'CS3')}

    cls.pv_root = 'BRICK1:'
    cls.defer = cls.pv_root + 'DeferMoves'
    cls.pv_cs_group = cls.pv_root + 'COORDINATE_SYS_GROUP'
    cls.pv_command = cls.pv_root + 'SendCmd'

    for name, value in cls.axes.items() + cls.groups.items() + cls.cs.items():
        setattr(cls, name, value)

    # axis aliases
    cls.height = cls.X3
    cls.angle = cls.Y3
    cls.jack1 = cls.m3
    cls.jack2 = cls.m4

    return cls


@add_brick_attributes
@add_trajectory_attributes
class TestBrick:
    def __init__(self):
        self.startup()

    def startup(self):
        a = self.__class__.axes
        r = self.__class__.real_axes

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
        self.send_command('i117,8,100=10 i116,8,100=500')

        # move all real motors to zero
        demand_pvs = [axis.demand for axis in r.values()]
        values = [0] * len(demand_pvs)
        ca.caput(demand_pvs, values, wait=True, timeout=30)

        # choose a default coordinate system group
        self.set_cs_group(self.g3)

    @classmethod
    def all_axes(cls, d, attribute):
        """ return a list of pv strings for the given attribute on axes passed in d"""
        return [getattr(s, attribute) for s in d.values()]

    @classmethod
    def all_values(cls, d, value):
        """ create a list of length axis count with value in all elements """
        return [value] * len(d)

    @classmethod
    def all_go(cls, axes, values, wait=True, timeout=10):
        pvs = [axis.demand for axis in axes]
        ca.caput(pvs, values, wait=wait, timeout=timeout)

    @classmethod
    def all_go_direct(cls, axes, values, wait=True, timeout=10):
        pvs = [axis.direct_demand for axis in axes]
        ca.caput(pvs, values, wait=wait, timeout=timeout)

    def set_cs_group(self, group):
        ca.caput(self.pv_cs_group, group, wait=True)

    def set_trajectory_cs(self, cs):
        ca.caput(self.pv_trajectory_cs, cs.port, wait=True)

    def send_command(self, command):
        ca.caput(self.pv_command, command, wait=True, datatype=ca.DBR_CHAR_STR)

    def get_command(self):
        return ca.caget(self.pv_command, datatype=ca.DBR_CHAR_STR)

    def set_deferred_moves(self, defer):
        ca.caput(self.defer, defer, wait=True)
