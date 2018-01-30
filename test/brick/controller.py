from cothread import catools as ca


def make_controller(axes, groups, cs, pv_root):
    def add_attributes(cls):
        """ add a dictionary of axes and CS groups to the controller and also add each entry in
        the dictionary as an attribute of the controller.

        also add items for controlling trajectory scans on the test IOC
        """
        cls.axes = axes
        cls.real_axes = {k: v for k, v in cls.axes.items() if v.cs_no == 0}
        cls.groups = groups
        cls.cs = cs

        cls.pv_root = pv_root
        cls.defer = pv_root + 'DeferMoves'
        cls.pv_cs_group = pv_root + 'COORDINATE_SYS_GROUP'
        cls.pv_command = pv_root + 'SendCmd'

        cls.pv_trajectory_cs = pv_root + 'ProfileCsName'

        for name, value in cls.axes.items() + cls.groups.items() + cls.cs.items():
            setattr(cls, name, value)

        return cls

    @add_attributes
    class Controller:
        def __init__(self):
            self.startup()

        def startup(self):
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

        def set_trajectory_cs(self, cs):
            ca.caput(self.pv_trajectory_cs, cs.port, wait=True)

        def send_command(self, command):
            ca.caput(self.pv_command, command, wait=True, datatype=ca.DBR_CHAR_STR)

        def get_command(self):
            return ca.caget(self.pv_command, datatype=ca.DBR_CHAR_STR)

        def set_deferred_moves(self, defer):
            ca.caput(self.defer, defer, wait=True)

    return Controller
