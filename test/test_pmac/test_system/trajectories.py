from unittest import TestCase
from test.brick.testbrick import DECIMALS
from test.brick.trajectory import Trajectory
from cothread import Sleep
from datetime import datetime


def trajectory_quick_scan(test, test_brick):
    """
    Do a short 4D grid scan involving 2 1-1 motors and 2 virtual axes of two jack CS
    :param test_brick: the test brick instance to run against
    :param (TestCase) test: the calling test object, used to make assertions
    """
    tr = test_brick.trajectory
    assert isinstance(tr, Trajectory)

    # set up the axis parameters
    tr.axisA.use = "Yes"
    tr.axisB.use = "Yes"
    tr.axisX.use = "Yes"
    tr.axisY.use = "Yes"

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

    # each step is 50 mill secs
    times = [50000] * points
    modes = [0] * points
    for i in range(0, points, 5):
        modes[i] = 2

    tr.setup_scan(times, modes, points, points, "CS3")

    tr.ProfileExecute(timeout=30)
    test.assertTrue(tr.execute_OK)
    while test_brick.height.moving:
        Sleep(0.01)  # allow deceleration todo need to put this in the driver itself

    test.assertEquals(test_brick.m1.pos, rows[-1])
    test.assertEquals(test_brick.m2.pos, cols[-1])
    test.assertEquals(test_brick.height.pos, heights[-1])
    test.assertEquals(test_brick.angle.pos, angles[-1])


def trajectory_fast_scan(
    test, test_brick, n_axes, cs="CS3", microsecs=5000, distance=1
):
    """
    Do a fast scan involving n_axes motors
    :param n_axes: no. of axes to include in trajectory
    :param test_brick: the test brick instance to run against
    :param cs: the coordinate system name
    :param microsecs: time interval per step
    """
    tr = test_brick.trajectory
    assert isinstance(tr, Trajectory)
    steps = 10
    stepsize = distance / steps

    # build simple trajectory
    heights = []
    points = 0
    for height in range(steps + 1):
        points += 1
        heights.append(height * stepsize)

    # set up the axis parameters
    axis_count = 0
    while axis_count < n_axes:
        tr.axes[axis_count].use = "Yes"
        tr.axes[axis_count].positions = heights
        axis_count += 1

    # each point takes 5 milli sec per step
    times = [microsecs] * points
    # all points are interpolated
    modes = [0] * points

    tr.setup_scan(times, modes, points, points, cs)

    tr.ProfileExecute(timeout=30)

    while test_brick.m1.moving:
        Sleep(0.01)  # allow deceleration todo need to put this in the driver itself

    if not tr.execute_OK:
        raise RuntimeError


def trajectory_scan_appending(test, test_brick):
    """
    Do a short 4D grid scan involving 2 1-1 motors and 2 virtual axes of two jack CS
    :param test_brick: the test brick instance to run against
    :param (TestCase) test: the calling test object, used to make assertions
    """
    tr = test_brick.trajectory
    buff_len = tr.BufferLength

    assert isinstance(tr, Trajectory)
    # switch to correct CS mappings
    test_brick.set_cs_group(test_brick.g3)

    # set up the axis parameters
    tr.axisX.use = "Yes"

    # build trajectory
    ascend = []
    descend = []
    points = int(buff_len * 1.1)  # must start with > 1 buffer of points
    total_points = points * 4
    for height in range(points):
        ascend.append(height / 100.0)
        descend.append((points - height) / 100.0)

    tr.axisX.positions = ascend

    is_clipper = (
        True  # don't have a good test for this (was  is_clipper = buff_len < 1000)
    )
    time_period = 5000 if is_clipper else 2000
    # each point takes 2 milli sec for brick and 3 for clipper
    times = [time_period] * points

    # all points are interpolated
    modes = [0] * points

    tr.setup_scan(times, modes, points, total_points, "CS3")
    tr.axisX.positions = descend
    tr.configure_axes()
    tr.AppendPoints()
    tr.ProfileExecute(wait=False)
    Sleep(0)

    for iteration in range(int(total_points / points / 2 - 1)):
        tr.axisX.positions = ascend
        tr.configure_axes()
        tr.AppendPoints()
        Sleep(1)
        tr.axisX.positions = descend
        tr.configure_axes()
        tr.AppendPoints()
        Sleep(1)

    start = datetime.now()
    while not tr.execute_done:
        Sleep(0.5)
        elapsed = datetime.now() - start
        test.assertLess(elapsed.seconds, 120)

    test.assertTrue(tr.execute_OK)
    while test_brick.height.moving:
        Sleep(0.01)  # allow deceleration todo need to put this in the driver itself

    test.assertAlmostEqual(test_brick.height.pos, descend[-1], 1)
