from pathlib import Path
from unittest import TestCase

import cothread.catools as ca
import numpy as np
from dls_pmaclib.dls_pmacremote import PmacEthernetInterface
from dls_pmaclib.pmacgather import PmacGather
from malcolm.core import Process, Block
from malcolm.yamlutil import make_include_creator
from scanpointgenerator import SpiralGenerator, CompoundGenerator, \
    LineGenerator
from ..brick.testbrick import TBrick
from .plot_trajectories import plot_velocities

SAMPLE_RATE = 20  # how many between each point to gather
BRICK_CLOCK = 5000  # servo loop speed in Hz
FUDGE = .1  # no. secs to add to gather time as a safe buffer
AXES = [7, 8]  # always use axes x, y which are mapped to 7, 8


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
        self.m_res = []
        self.axes = []
        self.total_time = 0

        for axis in AXES:
            self.axes.append("stage{}".format(axis))

        self.make_malcolm()
        self.brick_connect()

    def tearDown(self) -> None:
        self.proc.stop()
        self.pmac.disconnect()
        pass

    def make_malcolm(self):
        # create a malcolm scan from a YAML definition
        yaml_file = Path(
            __file__).parent / '../../etc/malcolm/PMAC-ML-TEST-01.yaml'
        self.proc = Process("Process")
        controllers, parts = make_include_creator(str(yaml_file))()
        for controller in controllers:
            self.proc.add_controller(controller)
        self.scan_block = self.proc.block_view('PMAC_TEST_SCAN')
        self.traj_block = self.proc.block_view('PMAC-ML-BRICK-01:TRAJ')

        # prepare the scan
        self.proc.start()
        self.scan_block.simultaneousAxes.put_value(self.axes)

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
        self.total_time = np.sum(self.test_brick.trajectory.getProfileTimeArray())
        # time array is in microseconds
        self.total_time /= 1000000

        # configure the brick position gather
        samples_per_sec = SAMPLE_RATE / gen.duration
        ticks_per_sample = BRICK_CLOCK / samples_per_sec
        samples = (self.total_time + FUDGE) * samples_per_sec
        self.pmac_gather.gatherConfig(AXES, samples, ticks_per_sample)

        # start gathering and scanning
        self.pmac_gather.gatherTrigger(wait=False)
        self.scan_block.run()
        # make sure the gather period has expired
        self.pmac_gather.gatherWait()

        # extract the pmac gather info, disabling IOC polling while doing so
        self.test_brick.disable_polling()
        data = self.pmac_gather.collectData()
        self.test_brick.disable_polling(False)

        self.gather_points = []
        self.pmac_gather.parseData(data)
        for i, c in enumerate(self.pmac_gather.channels):
            egu_points = np.multiply(c.scaledData, self.m_res[i])
            self.gather_points.append(egu_points)

    def plot_scan(self, title, step_time):
        p = np.insert(np.array(self.traj_block.positionsX.value), 0,
                      self.start_x), \
            np.insert(np.array(self.traj_block.positionsY.value), 0,
                      self.start_y), \
            np.insert(np.array(self.traj_block.timeArray.value), 0, 0), \
            np.insert(np.array(self.traj_block.velocityMode.value), 0, 0), \
            np.insert(np.array(self.traj_block.userPrograms.value), 0, 0)
        print('trajectory arrays:-\n', p)
        plot_velocities(p, title=title, step_time=step_time,
                        overlay=self.gather_points)

    def test_spiral(self):
        step_time = .1
        # create a set of scan points in a spiral
        s = SpiralGenerator(self.axes, "mm", [0.0, 0.0],
                            5.0, scale=5)
        gen = CompoundGenerator([s], [], [], step_time)
        gen.prepare()

        self.test_brick.m7.set_speed(10 / step_time)
        self.test_brick.m8.set_speed(10 / step_time)
        self.test_brick.m7.set_acceleration(.01)
        self.test_brick.m8.set_acceleration(.01)

        self.do_a_scan(gen)
        self.plot_scan('Live Spiral', step_time)

    def test_snake(self):
        step_time = .1
        xs = LineGenerator(self.axes[0], "mm", 0, 10, 3)
        ys = LineGenerator(self.axes[1], "mm", 0, 8, 3)

        gen = CompoundGenerator([ys, xs], [], [], step_time)
        gen.prepare()

        self.test_brick.m7.set_speed(10 / step_time)
        self.test_brick.m8.set_speed(5 / step_time)
        self.test_brick.m7.set_acceleration(.1)
        self.test_brick.m8.set_acceleration(.3)

        self.do_a_scan(gen)
        self.plot_scan('Live Snake', step_time)
