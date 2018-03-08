from unittest import TestCase
from test.brick.testbrick import DECIMALS
from test.brick.trajectory import Trajectory
from cothread import Sleep


def trajectory_quick_scan(test, test_brick):
    """
    Do a short 4D grid scan involving 2 1-1 motors and 2 virtual axes of two jack CS
    :param test_brick: the test brick instance to run against
    :param (TestCase) test: the calling test object, used to make assertions
    """
    tr = test_brick.trajectory
    assert isinstance(tr, Trajectory)
    # switch to correct CS mappings
    test_brick.set_cs_group(test_brick.g3)

    # set up the axis parameters
    tr.axisA.use = 'Yes'
    tr.axisB.use = 'Yes'
    tr.axisX.use = 'Yes'
    tr.axisY.use = 'Yes'

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

    tr.axisA.positions = rows
    tr.axisB.positions = cols
    tr.axisX.positions = heights
    tr.axisY.positions = angles

    times = [100000] * points
    modes = [0] * points
    for i in range(0, points, 5):
        modes[i] = 2

    tr.setup_scan(times, modes, points, points, 'CS3')

    tr.ProfileExecute(timeout=30)
    test.assertTrue(tr.execute_OK)

    test.assertEquals(test_brick.m1.pos, rows[-1])
    test.assertEquals(test_brick.m2.pos, cols[-1])
    test.assertEquals(test_brick.height.pos, heights[-1])
    test.assertEquals(test_brick.angle.pos, angles[-1])


def trajectory_fast_scan(test, test_brick):
    """
    Do a short 4D grid scan involving 2 1-1 motors and 2 virtual axes of two jack CS
    :param test_brick: the test brick instance to run against
    :param (TestCase) test: the calling test object, used to make assertions
    """
    tr = test_brick.trajectory
    assert isinstance(tr, Trajectory)
    # switch to correct CS mappings
    test_brick.set_cs_group(test_brick.g3)

    # set up the axis parameters
    tr.axisX.use = 'Yes'

    # build trajectory
    heights = []
    points = 0
    for height in range(100):
        points += 1
        heights.append(height/100.0)

    tr.axisX.positions = heights

    # each point takes 5 milli sec
    times = [5000] * points
    # all points are interpolated
    modes = [0] * points

    tr.setup_scan(times, modes, points, points, 'CS3')

    tr.ProfileExecute(timeout=30)
    test.assertTrue(tr.execute_OK)

    test.assertAlmostEqual(test_brick.height.pos, heights[-1], 1)


def trajectory_scan_appending(test, test_brick):
    """
    Do a short 4D grid scan involving 2 1-1 motors and 2 virtual axes of two jack CS
    :param test_brick: the test brick instance to run against
    :param (TestCase) test: the calling test object, used to make assertions
    """
    tr = test_brick.trajectory
    assert isinstance(tr, Trajectory)
    # switch to correct CS mappings
    test_brick.set_cs_group(test_brick.g3)

    # set up the axis parameters
    tr.axisX.use = 'Yes'

    # build trajectory
    ascend = []
    descend = []
    points = 500
    total_points = 4000
    for height in range(points):
        ascend.append(height/100.0)
        descend.append((points-height)/100.0)

    tr.axisX.positions = ascend

    # each point takes 50 milli sec
    times = [50000] * points
    # all points are interpolated
    modes = [0] * points

    tr.setup_scan(times, modes, points, total_points, 'CS3')
    tr.ProfileExecute(wait=False)

    for iteration in range(total_points/points/2):
        if iteration > 0:
            tr.axisX.positions = ascend
            tr.configure_axes()
            tr.AppendPoints()
        tr.axisX.positions = descend
        tr.configure_axes()
        tr.AppendPoints()
        Sleep(.5)

    while not tr.execute_done:
        Sleep(.5)

    test.assertTrue(tr.execute_OK)

    test.assertAlmostEqual(test_brick.height.pos, descend[-1], 1)
