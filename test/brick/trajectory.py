#!/bin/env python
from cothread import catools as ca

ALL_AXES = "ABCUVWXYZ"


class AxisSetup:
    def __init__(self):
        self.use = False
        self.points = 0
        self.max_points = 10000
        self.resolution = 1
        self.offset = 0
        self.positions = []


class Trajectory:
    def __init__(self, pv_root):
        self.pv_root = pv_root
        self.axes = []
        for axis in ALL_AXES:
            this_axis = AxisSetup()
            setattr(self, "axis{}".format(axis), this_axis)
            self.axes.append(this_axis)

    def setup_scan(self, times, modes, points, max_points, cs_port):
        self.configure_axes()
        self.setProfileTimeArray(times)
        self.setVelocityMode(modes)

        # setup and execute the scan
        self.setProfilePointsToBuild(points)
        self.setProfileNumPoints(max_points)
        self.setProfileCsName(cs_port)
        self.ProfileBuild()

    def configure_axes(self):

        for axis in ALL_AXES:
            this_axis = getattr(self, "axis" + axis)
            ca.caput("{}{}:UseAxis".format(self.pv_root, axis), this_axis.use, wait=True)

            if this_axis.use:
                print(
                    "PUTTING positions {} TO {}{}:Positions".format(
                        this_axis.positions, self.pv_root, axis
                    )
                )
                ca.caput(
                    "{}{}:Positions".format(self.pv_root, axis),
                    this_axis.positions,
                    wait=True,
                )

    def setProfileTimeArray(self, value):
        ca.caput(self.pv_root + "ProfileTimeArray", value, wait=True)

    def getProfileTimeArray(self):
        return ca.caget(self.pv_root + "ProfileTimeArray")

    def setVelocityMode(self, value):
        ca.caput(self.pv_root + "VelocityMode", value, wait=True)

    def setProfileNumPoints(self, value):
        ca.caput(self.pv_root + "ProfileNumPoints", value, wait=True)

    def setProfilePointsToBuild(self, value):
        ca.caput(self.pv_root + "ProfilePointsToBuild", value, wait=True)

    def setProfileCsName(self, value):
        ca.caput(self.pv_root + "ProfileCsName", value, wait=True)

    def ProfileBuild(self):
        ca.caput(self.pv_root + "ProfileBuild", 1, wait=True)

    def ProfileExecute(self, wait=True, timeout=10):
        ca.caput(self.pv_root + "ProfileExecute", 1, timeout=timeout, wait=wait)

    def AppendPoints(self):
        ca.caput(self.pv_root + "ProfileAppend", 1, wait=True)

    @property
    def BufferLength(self):
        return ca.caget(self.pv_root + "BufferLength_RBV")

    @property
    def ProfileBuildStatus(self):
        return ca.caget(self.pv_root + "ProfileBuildStatus_RBV", datatype=ca.DBR_STRING)

    @property
    def ProfileExecuteMessage(self):
        return ca.caget(
            self.pv_root + "ProfileExecuteMessage_RBV", datatype=ca.DBR_CHAR_STR
        )

    @property
    def ProfileExecuteState(self):
        return ca.caget(
            self.pv_root + "ProfileExecuteState_RBV", datatype=ca.DBR_STRING
        )

    @property
    def ProfileExecuteStatus(self):
        return ca.caget(
            self.pv_root + "ProfileExecuteStatus_RBV", datatype=ca.DBR_STRING
        )

    @property
    def build_OK(self):
        return self.ProfileBuildStatus == "Success"

    @property
    def execute_OK(self):
        result = self.ProfileExecuteStatus == "Success"
        return result

    @property
    def execute_done(self):
        return self.ProfileExecuteState == "Done"
