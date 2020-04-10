#!/bin/env python
from cothread import catools as ca
from cothread import Sleep


class MoveMonitor:
    """ Monitor a motor's DMOV field to determine it as started and then
        completed motion
        """

    def __init__(self, motor_pv):
        self._motor_pv = motor_pv
        self._done_moving_pv = motor_pv + ".DMOV"
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

    def wait_for_one_move(self, timeout, throw=True):
        interval = 0.1
        waited = 0
        while not self._completed_one_move:
            Sleep(interval)
            waited += interval
            if waited > timeout:
                if throw:
                    raise RuntimeError(
                        "timeout waiting for motor {}".format(self._motor_pv)
                    )
                else:
                    break


class MotorCallback:
    """ Provide a function for 'put with callback' caput to monitor when motion
        completes. To use: instantiate an instance and pass its moves_done method
        as the callback. Then call wait_for_done to block until the motor completes.
        """

    def __init__(self):
        self.moving = False

    def moves_done(self, _):
        self.moving = False

    def reset_done(self):
        self.moving = True

    def wait_for_done(self):
        while self.moving:
            Sleep(0.05)
