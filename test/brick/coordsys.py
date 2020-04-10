from cothread import catools as ca


class CoordSys:
    class CsAxisAlias:
        def __init__(self, pv):
            self.demand = pv

        def go(self, position, wait=True, callback=None):
            ca.caput(self.demand, position, wait=wait, callback=callback, timeout=60)

        def go_direct(self, position, wait=True, callback=None):
            ca.caput(
                self.demand + ":DirectDemand",
                position,
                wait=wait,
                callback=callback,
                timeout=60,
            )

    def __init__(self, pv_root, cs_no, port, init=True):
        self.pv_root = pv_root
        self.cs_no = cs_no
        self.port = port
        self.move_time = pv_root + ":CsMoveTime"
        self.defer = pv_root + ":DeferMoves"
        self.pv_abort = pv_root + ":Abort"

        # add CS axis alias PVs (these are REQUIRED to control motors that have
        # a CS mapping but no motor record. Usually 1-1 CS-real mapped axes.
        for i in range(1, 9):
            setattr(
                self,
                "M{}".format(i),
                self.CsAxisAlias("{}:M{}".format(self.pv_root, i)),
            )

        if init:
            self.set_move_time(0)
            self.set_deferred_moves(False)

    def set_move_time(self, move_time):
        ca.caput(self.move_time, move_time, wait=True)

    def set_deferred_moves(self, defer):
        ca.caput(self.defer, defer, wait=True)

    def abort(self):
        ca.caput(self.pv_abort, 1, wait=True)
