from cothread import catools as ca
from cothread import Sleep


class Axis:
    def __init__(self, brick_pv_root, pv_root, axis_no, cs_no):
        self.brick_pv_root = brick_pv_root
        self.pv_root = pv_root
        self.axis_no = axis_no
        self.cs_no = cs_no
        self.demand = pv_root
        self.rbv = pv_root + ".RBV"
        self.acc = pv_root + ".ACCL"
        self.stop_pv = pv_root + ".STOP"
        self.velo = pv_root + ".VELO"
        self.vmax = pv_root + ".VMAX"
        self.off = pv_root + ".OFF"
        self.mres = pv_root + ".MRES"
        self.lo_limit = pv_root + ".LLM"
        self.hi_limit = pv_root + ".HLM"
        self.pv_done_moving = pv_root + ".DMOV"
        self.pv_use_encoder = pv_root + ".UEIP"
        if cs_no > 0:
            cs_name = "CS{}:".format(cs_no)
        else:
            cs_name = ""
        self.cs_assignment = brick_pv_root + ":M{}:CsAxis".format(axis_no)
        self.cs_port = brick_pv_root + ":M{}:CsPort".format(axis_no)
        self.direct_demand = brick_pv_root + ":{}M{}:DirectDemand".format(
            cs_name, axis_no
        )

        # the following helps to avoid waiting for a timeout if the IOC is down
        ca.caget(self.rbv, timeout=0.1)

    @property
    def pos(self):
        return ca.caget(self.rbv)

    def go(self, position, wait=True):
        ca.caput(self.demand, position, wait=wait, timeout=60)
        Sleep(0.05)  # test clipper reports in position a little early sometimes

    def go_direct(self, position, wait=True, callback=None):
        ca.caput(self.direct_demand, position, wait=wait, timeout=60, callback=callback)
        Sleep(0.05)  # test clipper reports in position a little early sometimes

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

    def set_limits(self, lo, hi):
        ca.caput(self.lo_limit, lo, wait=True)
        ca.caput(self.hi_limit, hi, wait=True)

    @property
    def alarm(self):
        result = ca.caget(self.pv_root, format=ca.FORMAT_CTRL)
        return result.severity

    @property
    def moving(self):
        return not ca.caget(self.pv_done_moving)
