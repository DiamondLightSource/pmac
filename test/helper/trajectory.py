#!/bin/env python
from unittest import TestCase
from cothread import catools as ca

PB = 'BRICK1:'
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
        # switch to correct CS mappings
        ca.caput(PB + 'COORDINATE_SYS_GROUP', 'MIXED')

        # set up the axis parameters
        ca.caput(PB + 'A:Resolution', "1")
        ca.caput(PB + 'B:Resolution', "1")
        ca.caput(PB + 'X:Resolution', "1")
        ca.caput(PB + 'Y:Resolution', "1")
        cls.set_list(PB, 'Offset', axes, 0)
        cls.set_list(PB, 'UseAxis', axes, 'Yes')
        cls.set_list(PB, 'UseAxis', ALL_AXES - axes, 'No')

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
        ca.caput(PB + 'A:Positions', rows)
        ca.caput(PB + 'B:Positions', cols)
        ca.caput(PB + 'X:Positions', heights)
        ca.caput(PB + 'Y:Positions', angles)

        times = [100000] * points
        modes = [0] * points
        for i in range(0, points, 5):
            modes[i] = 2

        ca.caput(PB + 'ProfileTimeArray', times)
        ca.caput(PB + 'VelocityMode', modes)

        # setup and execute the scan
        ca.caput(PB + 'ProfilePointsToBuild', points)
        ca.caput(PB + 'ProfileNumPoints', points)
        ca.caput(PB + 'ProfileCsName', 'CS3')
        ca.caput(PB + 'ProfileBuild', 1, wait=True, timeout=2)
        assert(ca .caget(PB + 'ProfileBuildStatus_RBV') != 2)

        ca.caput(PB + 'ProfileExecute', 1, wait=True, timeout=30)

        test.assertEquals(
            ca.caget(PB + 'ProfileBuildStatus_RBV', datatype=ca.DBR_STRING),
            'Success')
        test.assertEquals(
            ca.caget(PB + 'ProfileExecuteMessage_RBV', datatype=ca.DBR_CHAR_STR),
            'Trajectory scan complete')
        test.assertEquals(ca.caget('PMAC_BRICK_TEST:MOTOR1.RBV'), rows[-1])
        test.assertEquals(ca.caget('PMAC_BRICK_TEST:MOTOR2.RBV'), cols[-1])
        test.assertEquals(ca.caget('BRICK1CS3:X.RBV'), heights[-1])
        test.assertEquals(ca.caget('BRICK1CS3:Y.RBV'), angles[-1])
