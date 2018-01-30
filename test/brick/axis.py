from cothread import catools as ca


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
        self.cs_assignment = 'BRICK1:M{}:CsAxis'.format(axis_no)
        self.cs_port = 'BRICK1:M{}:CsPort'.format(axis_no)
        self.direct_demand = 'BRICK1:{}M{}:DirectDemand'.format(cs_name, axis_no)
        # the following helps to avoid waiting for a timeout when the IOC is down
        ca.caget(self.direct_demand, timeout=.1)

    @property
    def pos(self):
        return ca.caget(self.rbv)

    def go(self, position, wait=True):
        ca.caput(self.demand, position, wait=wait, timeout=60)

    def go_direct(self, position, wait=True, callback=None):
        ca.caput(self.direct_demand, position, wait=wait, timeout=60,
                 callback=callback)

    def stop(self):
        ca.caput(self.stop_pv, 1, wait=True)

    def set_speed(self, speed):
        ca.caput(self.vmax, speed, wait=True)
        ca.caput(self.velo, speed, wait=True)

    def set_acceleration(self, acc):
        ca.caput(self.acc, acc, wait=True)

    def set_cs_assignment(self, mapping):
        ca.caput(self.cs_assignment, mapping, wait=True)

    def set_cs_port(self, port):
        ca.caput(self.cs_port, port, wait=True)

    @property
    def alarm(self):
        result = ca.caget(self.pv_root, format=ca.FORMAT_CTRL)
        return result.severity
