from cothread import catools as ca, Sleep
from .trajectory import Trajectory

# use dynamic class since many instances of this class will be created during a test suite
# and this way add_attributes is only called once


def make_controller(c_axes, c_groups, c_cs, pv_root):
    def add_attributes(cls):
        """ add a dictionary of axes and CS groups to the controller and also add each entry in
        the dictionary as an attribute of the controller.

        also add items for controlling trajectory scans on the test IOC
        """
        cls.axes = c_axes
        cls.real_axes = {k: v for k, v in cls.axes.items() if v.cs_no == 0}
        cls.groups = c_groups
        cls.cs = c_cs

        cls.pv_root = pv_root
        cls.defer = pv_root + "DeferMoves"
        cls.pv_cs_group = pv_root + "COORDINATE_SYS_GROUP"
        cls.pv_command = pv_root + "SendCmd"
        cls.pv_pollAllNow = pv_root + "PollAllNow"
        cls.pv_disablePoll = pv_root + "DISABLE_POLL"
        cls.pv_trajectory_error = pv_root + "ProfileExecuteMessage_RBV"

        for name, value in {**cls.axes, **cls.groups, **cls.cs}.items():
            setattr(cls, name, value)

        return cls

    @add_attributes
    class Controller:
        def __init__(self, init=True):
            self.startup(init)

        def startup(self, init):
            """ override this function to put the brick in the initial state required for
                tests. E.g. set speeds and initial motor positions """
            raise NotImplementedError

        @classmethod
        def all_axes(cls, d, attribute):
            """ return a list of pv strings for the given attribute on axes passed in d"""
            return [getattr(s, attribute) for s in d.values()]

        # @classmethod
        # def all_values(cls, d, value):
        #    """ create a list of length axis count with value in all elements """
        #    return [value] * len(d)

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
            # TODO we have race conditions on switch of CS:-
            #  (1) if the direct demand resolutions need to change there is a small delay
            #  (2) the brick itself may take a short time to do the CS mappings
            # for now we require a short wait after a switch
            # (this may not be fixable - (1) showed up on ppmac (2) showed up on VMXI)
            Sleep(0.3)

        def send_command(self, command):
            ca.caput(self.pv_command, command, wait=True, datatype=ca.DBR_CHAR_STR)

        def poll_all_now(self):
            ca.caput(self.pv_pollAllNow, 1, wait=True)

        def get_command(self):
            return ca.caget(self.pv_command, datatype=ca.DBR_CHAR_STR)

        def set_deferred_moves(self, defer):
            ca.caput(self.defer, defer, wait=True)

        def disable_polling(self, disable: bool = True):
            ca.caput(self.pv_disablePoll, disable, wait=True)

        def get_trajectory_error(self):
            return ca.caget(self.pv_trajectory_error, datatype=ca.DBR_CHAR_STR)

    return Controller
