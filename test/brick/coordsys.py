from cothread import catools as ca


class CoordSys:
    def __init__(self, pv_root, cs_no, port):
        self.pv_root = pv_root
        self.cs_no = cs_no
        self.port = port
        self.move_time = pv_root + ':CsMoveTime'
        self.defer = pv_root + ':DeferMoves'
        self.set_move_time(0)
        self.set_deferred_moves(False)

    def set_move_time(self, move_time):
        ca.caput(self.move_time, move_time, wait=True)

    def set_deferred_moves(self, defer):
        ca.caput(self.defer, defer, wait=True)