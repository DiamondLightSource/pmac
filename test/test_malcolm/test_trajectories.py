from pathlib import Path
from unittest import TestCase

import cothread.catools as ca
import numpy as np
from cothread import Sleep
from dls_pmaclib.dls_pmacremote import PmacEthernetInterface
from dls_pmaclib.pmacgather import PmacGather
from scanpointgenerator import SpiralGenerator, CompoundGenerator, \
    LineGenerator

from malcolm.core import Process, Block
from malcolm.yamlutil import make_include_creator
from .plot_trajectories import plot_velocities
from ..brick.testbrick import TBrick

SAMPLE_RATE = 10  # how many between each point to gather
BRICK_CLOCK = 5000  # servo loop speed in Hz
FUDGE = .2  # no. secs to add to gather time as a safe buffer
AXES = [7, 8]  # always use axes x, y which are mapped to 7, 8

# triggering modes
TRIGGER_NONE = 0
TRIGGER_ROW = 1
TRIGGER_EVERY = 2


class TestTrajectories(TestCase):
    def setUp(self) -> None:
        self.proc: Process = None
        self.scan_block: Block = None
        self.traj_block: Block = None
        self.test_brick: TBrick = None
        self.pmac: PmacEthernetInterface = None
        self.pmac_gather: PmacGather = None
        self.gather_points = None
        self.start_x = 0
        self.start_y = 0
        self.step_time = 0
        self.m_res = []
        self.axes = []
        self.total_time = 0
        self.skip_run = False  # set true to not send trajectory to pmac

        self.min_interval = None
        self.min_index = None

        for axis in AXES:
            self.axes.append("stage{}".format(axis))

        self.brick_connect()

    def tearDown(self) -> None:
        self.proc.stop()
        self.pmac.disconnect()

    def setup_brick_malcolm(self, xv, yv, xa, ya):
        self.test_brick.m7.set_speed(xv)
        self.test_brick.m8.set_speed(yv)
        self.test_brick.m7.set_acceleration(xa)
        self.test_brick.m8.set_acceleration(ya)

        Sleep(.1)

        # create a malcolm scan from a YAML definition
        yaml_file = Path(
            __file__).parent / '../../etc/malcolm/PMAC-ML-TEST-01.yaml'
        self.proc = Process("Process")
        controllers, parts = make_include_creator(str(yaml_file))()
        for controller in controllers:
            self.proc.add_controller(controller)
        self.scan_block = self.proc.block_view('PMAC_TEST_SCAN')
        self.traj_block = self.proc.block_view('PMAC-ML-BRICK-01:TRAJ')
        self.trigger_block = self.proc.block_view('PMAC_TEST_SCAN:TRIG')

        # prepare the scan
        self.proc.start()
        self.scan_block.simultaneousAxes.put_value(self.axes)
        self.trigger_block.rowTrigger.put_value(TRIGGER_EVERY)

    def brick_connect(self):
        # create test brick object to communicate with pmac IOC
        self.test_brick = TBrick()
        # set up coordinate system mappings group 1 (CS2 1 to 1 mappings)

        self.test_brick.set_cs_group(self.test_brick.g1)
        # setup the pmac for gathering via the dls-pmaclib module
        self.pmac = PmacEthernetInterface(verbose=True)
        self.pmac.setConnectionParams('172.23.240.97', 1025)
        self.pmac.connect()
        self.pmac_gather = PmacGather(self.pmac)

        self.m_res.append(float(ca.caget(self.test_brick.m7.mres)))
        self.m_res.append(float(ca.caget(self.test_brick.m8.mres)))

    def do_a_scan(self, gen: CompoundGenerator):
        # configure the Malcolm scan
        self.scan_block.configure(gen)
        self.start_x = self.test_brick.m7.pos
        self.start_y = self.test_brick.m8.pos
        self.step_time = gen.duration
        times = self.test_brick.trajectory.getProfileTimeArray()
        self.total_time = np.sum(times)
        self.min_interval = np.min(times)
        self.min_index = np.where(self.min_interval == times)[0]

        # time array is in microseconds
        self.total_time /= 1000000

        # configure the brick position gather
        samples_per_sec = SAMPLE_RATE / gen.duration
        ticks_per_sample = BRICK_CLOCK / samples_per_sec
        samples = (self.total_time + FUDGE) * samples_per_sec
        self.pmac_gather.gatherConfig(AXES, samples, ticks_per_sample)

        if not self.skip_run:
            # start gathering and scanning
            self.pmac_gather.gatherTrigger(wait=False)
            self.scan_block.run()
            # make sure the gather period has expired
            self.pmac_gather.gatherWait()

            # extract the pmac gather info, disabling IOC polling while
            # doing so
            self.test_brick.disable_polling()
            data = self.pmac_gather.collectData()
            self.test_brick.disable_polling(False)

            self.gather_points = []
            self.pmac_gather.parseData(data)
            for i, c in enumerate(self.pmac_gather.channels):
                egu_points = np.multiply(c.scaledData, self.m_res[i])
                self.gather_points.append(egu_points)

    def plot_scan(self, title, failed=False):
        p = np.insert(np.array(self.traj_block.positionsX.value), 0,
                      self.start_x), \
            np.insert(np.array(self.traj_block.positionsY.value), 0,
                      self.start_y), \
            np.insert(np.array(self.traj_block.timeArray.value), 0, 0), \
            np.insert(np.array(self.traj_block.velocityMode.value), 0, 0), \
            np.insert(np.array(self.traj_block.userPrograms.value), 0, 0)
        print('trajectory arrays:-\n', p)

        title += '\n MinInterval={}us at {}'.format(
            self.min_interval, self.min_index
        )
        if failed:
            title = 'FAILED ' + title
        plot_velocities(p, title=title, step_time=self.step_time,
                        overlay=self.gather_points)
        # return the position arrays including start point to the caller
        xp, yp, _, _, _ = p
        return xp, yp

    def scan_and_plot(self, gen, title):
        try:
            self.do_a_scan(gen)
        except AssertionError:
            self.plot_scan(title, failed=True)
            raise
        return self.plot_scan(title)

    def test_spiral(self, trigger=TRIGGER_EVERY):
        step_time = .5
        # create a set of scan points in a spiral
        s = SpiralGenerator(self.axes, "mm", [0.0, 0.0],
                            5.0, scale=5)
        gen = CompoundGenerator([s], [], [], step_time)
        gen.prepare()

        self.setup_brick_malcolm(xv=100, yv=100, xa=.2, ya=.2)
        self.trigger_block.rowTrigger.put_value(trigger)

        self.scan_and_plot(gen, 'Live Spiral')

    def test_raster(self):
        self.lines(False, 'Live Raster')

    def test_snake(self):
        self.lines(True, 'Live Snake')

    def lines(self, snake=False, name='lines'):
        step_time = .2
        xs = LineGenerator(self.axes[0], "mm", 0, 10, 3, alternate=snake)
        ys = LineGenerator(self.axes[1], "mm", 0, 8, 3)

        gen = CompoundGenerator([ys, xs], [], [], step_time)
        gen.prepare()

        self.setup_brick_malcolm(xv=100, yv=100, xa=.5, ya=.5)

        xp, yp = self.scan_and_plot(gen, name)
        self.check_bounds(xp, '{} array x'.format(name))
        self.check_bounds(yp, '{} array y'.format(name))

    TITLE_PATTERN = '{} xv={} yv={} xa={} ya={}'

    def Interpolation_checker(self, xv=200., yv=400., xa=1., ya=80.,
                              skip=False, snake=False, name='',
                              trigger=TRIGGER_EVERY):
        xs = LineGenerator("stage7", "mm", 0, 4, 5, alternate=snake)
        ys = LineGenerator("stage8", "mm", 0, 2, 3)

        gen = CompoundGenerator([ys, xs], [], [], 0.15)
        gen.prepare()

        self.setup_brick_malcolm(xv=xv, yv=yv, xa=xa, ya=ya)
        self.trigger_block.rowTrigger.put_value(trigger)

        title = self.TITLE_PATTERN.format(name, xv, yv, xa, ya)
        xp, yp = self.scan_and_plot(gen, title)
        self.check_bounds(xp, '{} array x'.format(name))
        self.check_bounds(yp, '{} array y'.format(name))
        return xp, yp

    def check_bounds(self, a, name):
        # small amounts of overshoot are acceptable
        npa = np.array(a)
        less_start = np.argmax((npa[0] - npa) > 0.001)
        greater_end = np.argmax((npa - npa[-1]) > 0.001)
        self.assertEqual(
            less_start, 0, "Position {} < start for {}\n{}".format(
                less_start, name, a))
        self.assertEqual(
            greater_end, 0, "Position {} > end for {}\n{}".format(
                greater_end, name, a))

    def test_high_acceleration(self):
        # this system test performs the same trajectory as the malcolm unit
        # test pmacchildpart_test.test_turnaround_overshoot
        self.Interpolation_checker(17, 1, .1, .2,
                                   name='test_high_acceleration')

    def test_profile_point_interpolation(self):
        # test combinations of velocity profile points

        # x=6 and y=3 combined = 7
        self.Interpolation_checker(name='x6 y3 interpolation')
        self.scan_block.reset()
        # x=6 and y=4 combined = 8
        self.Interpolation_checker(ya=1, name='x6 y4 interpolation')
        self.scan_block.reset()
        # x=4 and y=3 combined = 5
        self.Interpolation_checker(snake=True, name='x4 y3 interpolation')
        self.scan_block.reset()
        # x=3 and y=4 combined = 5
        self.Interpolation_checker(
            ya=.01, snake=True, name='x3 y4 interpolation')

    def test_sparse_trajectory(self):
        # todo validate that these 1st two create same trajectory
        # x=6 and y=3 combined = 7. Trigggering at start of row only
        self.Interpolation_checker(xv=200, yv=200, xa=.1, skip=False,
                                   name='Row trigger', trigger=TRIGGER_ROW)

        # x=6 and y=3 combined = 7. Triggering all points
        self.Interpolation_checker(xv=200, yv=200, xa=.1, skip=True,
                                   name='Every trigger')
        # self.test_spiral(trigger=TRIGGER_ROW)

    def test_dummy(self):
        try:
            self.Interpolation_checker(
                xv=200, yv=200, xa=1, skip=False, name='Every trigger',
                trigger=TRIGGER_ROW
            )
        finally:
            print('min interval {} at position {}'.format(
                self.min_interval, self.min_index))
