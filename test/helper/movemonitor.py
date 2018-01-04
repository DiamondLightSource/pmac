#!/bin/env python
from cothread import catools as ca
from cothread import Sleep


class MoveMonitor:
    def __init__(self, motor_pv):
        self._motor_pv = motor_pv
        self._done_moving_pv = motor_pv + '.DMOV'
        self._completed_one_move = False
        self._moving = not ca.caget(self._done_moving_pv)
        ca.camonitor(self._done_moving_pv, self.state_changed)

    @property
    def completed_one_move(self):
        return self._completed_one_move

    def state_changed(self, done_moving):
        if done_moving:
            if self._moving:
                self._completed_one_move = True
            self._moving = False
        else:
            self._moving = True

    def reset(self):
        self._completed_one_move = False

    def wait_for_one_move(self, timeout):
        interval = .1
        waited = 0
        while not self._completed_one_move:
            Sleep(interval)
            waited += interval
            if waited > timeout:
                raise RuntimeError("timeout waiting for motor {}".format(self._motor_pv))
