#!/bin/env python
from unittest import TestCase
from cothread import catools as ca
from test.brick.testbrick import TestBrick

ALL_AXES = set('ABCUVWXYZ')


class Trajectory:
    def __init__(self):
        pass

    @classmethod
    def set_list(cls, prefix, postfix, middle_list, value):
        for middle in middle_list:
            pv = prefix + middle + ':' + postfix
            ca.caput(pv, value)

    @classmethod
    def quick_scan(cls, test):
        """
        Do a short scan involving 2 1-1 motors and 2 virtual axes of two jack CS
        :param (TestCase) test:
        """
        axes = set('ABXY')

        tb = TestBrick()
        pb = tb.pv_root
        # switch to correct CS mappings
        tb.set_cs_group(tb.g3)

        # set up the axis parameters
        ca.caput(pb + 'A:Resolution', "1")
        ca.caput(pb + 'B:Resolution', "1")
        ca.caput(pb + 'X:Resolution', "1")
        ca.caput(pb + 'Y:Resolution', "1")
        cls.set_list(pb, 'Offset', axes, 0)
        cls.set_list(pb, 'UseAxis', axes, 'Yes')
        cls.set_list(pb, 'UseAxis', ALL_AXES - axes, 'No')

        # build trajectories (its a 4d raster)
        heights = []
        rows = []
        cols = []
        angles = []
        points = 0

        for angle in [-1, 0]:
            for height in range(2):
                for col_b in range(3):
                    for row_a in range(4):
                        points += 1
                        heights.append(height)
                        rows.append(row_a)
                        cols.append(col_b)
                        angles.append(angle)
        ca.caput(pb + 'A:Positions', rows)
        ca.caput(pb + 'B:Positions', cols)
        ca.caput(pb + 'X:Positions', heights)
        ca.caput(pb + 'Y:Positions', angles)

        times = [100000] * points
        modes = [0] * points
        for i in range(0, points, 5):
            modes[i] = 2

        ca.caput(pb + 'ProfileTimeArray', times)
        ca.caput(pb + 'VelocityMode', modes)

        # setup and execute the scan
        ca.caput(pb + 'ProfilePointsToBuild', points)
        ca.caput(pb + 'ProfileNumPoints', points)
        ca.caput(pb + 'ProfileCsName', 'CS3')
        ca.caput(pb + 'ProfileBuild', 1, wait=True, timeout=2)
        assert(ca .caget(pb + 'ProfileBuildStatus_RBV') != 2)

        ca.caput(pb + 'ProfileExecute', 1, wait=True, timeout=30)

        test.assertEquals(
            ca.caget(pb + 'ProfileBuildStatus_RBV', datatype=ca.DBR_STRING),
            'Success')
        test.assertEquals(
            ca.caget(pb + 'ProfileExecuteMessage_RBV', datatype=ca.DBR_CHAR_STR),
            'Trajectory scan complete')
        test.assertEquals(tb.m1.pos, rows[-1])
        test.assertEquals(tb.m2.pos, cols[-1])
        test.assertEquals(tb.height.pos, heights[-1])
        test.assertEquals(tb.angle.pos, angles[-1])
