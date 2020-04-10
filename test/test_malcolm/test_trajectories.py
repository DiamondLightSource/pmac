from datetime import datetime, timedelta
from pathlib import Path
from unittest import TestCase
import logging

import cothread.catools as ca
import numpy as np
import json
from cothread import Sleep
from dls_pmaclib.dls_pmacremote import PmacEthernetInterface
from dls_pmaclib.pmacgather import PmacGather
from scanpointgenerator import SpiralGenerator, CompoundGenerator, LineGenerator

from malcolm.core import Process, Block
from malcolm.yamlutil import make_include_creator
from test.test_malcolm.plot_trajectories import plot_velocities, plot_pos_time
from test.brick.testbrick import TBrick

SAMPLE_RATE = 50  # gather samples / second
BRICK_CLOCK = 5000  # servo loop speed in Hz
FUDGE = 0.2  # no. secs to add to gather time as a safe buffer
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
        self.x_scale = None
        self.y_scale = None

        self.min_interval = None
        self.min_index = None

        self.brick_connect()

        for axis in AXES:
            self.axes.append("stage{}".format(axis))

    def tearDown(self) -> None:
        self.proc.stop()
        self.pmac.disconnect()

    def setup_brick_malcolm(self, xv, yv, xa, ya, mres=None):
        self.pmac_gather = PmacGather(self.pmac)

        if mres:
            ca.caput(self.test_brick.m7.mres, mres)
            ca.caput(self.test_brick.m8.mres, mres)
        self.test_brick.m7.set_speed(xv)
        self.test_brick.m8.set_speed(yv)
        self.test_brick.m7.set_acceleration(xa)
        self.test_brick.m8.set_acceleration(ya)

        self.m_res.append(float(ca.caget(self.test_brick.m7.mres)))
        self.m_res.append(float(ca.caget(self.test_brick.m8.mres)))

        # yield cothread so Malcolm can see the changes
        Sleep(0.001)

        # create a malcolm scan from a YAML definition
        yaml_file = Path(__file__).parent / "../../etc/malcolm/PMAC-ML-TEST-01.yaml"
        self.proc = Process("Process")
        controllers, parts = make_include_creator(str(yaml_file))()
        for controller in controllers:
            self.proc.add_controller(controller)
        self.scan_block = self.proc.block_view("PMAC_TEST_SCAN")
        self.traj_block = self.proc.block_view("PMAC-ML-BRICK-01:TRAJ")
        self.pmac_block = self.proc.block_view("PMAC-ML-BRICK-01")
        self.trigger_block = self.proc.block_view("PMAC_TEST_SCAN:TRIG")

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
        self.pmac.setConnectionParams("172.23.240.59", 1025)
        self.pmac.connect()

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
        self.total_time = np.round(self.total_time)

        # configure the brick position gather
        ticks_per_sample = BRICK_CLOCK / SAMPLE_RATE
        samples = (np.round(self.total_time) + FUDGE) * SAMPLE_RATE
        self.pmac_gather.gatherConfig(AXES, samples, ticks_per_sample)

        elapsed_time = timedelta(0)
        if not self.skip_run:
            # start gathering and scanning
            self.pmac_gather.gatherTrigger(wait=False)
            start_time = datetime.now()
            self.scan_block.run()
            elapsed_time = datetime.now() - start_time
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
                if i < len(self.m_res):
                    egu_points = np.multiply(c.scaledData, self.m_res[i])
                    self.gather_points.append(egu_points)
                else:
                    print("WARNING - gather data has extra channels??")
        return elapsed_time

    def plot_scan(self, title, failed=False, elapsed=None, duration=0):
        p = (
            np.insert(np.array(self.traj_block.positionsX.value), 0, self.start_x),
            np.insert(np.array(self.traj_block.positionsY.value), 0, self.start_y),
            np.insert(np.array(self.traj_block.timeArray.value), 0, 0),
            np.insert(np.array(self.traj_block.velocityMode.value), 0, 0),
            np.insert(np.array(self.traj_block.userPrograms.value), 0, 0),
        )
        print("trajectory arrays:-\n", p)

        title += "\nPointDuration={}s Time={}s\nMinInterval={}ms" " at {}".format(
            duration,
            elapsed.total_seconds(),
            self.min_interval / 1000,
            self.min_index[:5],
        )
        if failed:
            title = "FAILED " + title
        plot_velocities(
            p,
            title=title,
            step_time=self.step_time,
            overlay=self.gather_points,
            x_scale=self.x_scale,
            y_scale=self.y_scale,
        )
        # this is a bit naff
        self.x_scale = None
        self.y_scale = None
        # return the position arrays including start point to the caller
        xp, yp, _, _, _ = p
        return xp, yp

    def scan_and_plot(self, gen, title):
        try:
            elapsed = self.do_a_scan(gen)
        except AssertionError:
            self.plot_scan(title, failed=True)
            raise
        return self.plot_scan(title, elapsed=elapsed, duration=gen.duration)

    def do_spiral(
        self, trigger=TRIGGER_EVERY, name="Spiral", interval=0.5, continuous=True
    ):
        # create a set of scan points in a spiral
        s = SpiralGenerator(self.axes, "mm", [0.0, 0.0], 5.0, scale=5)
        gen = CompoundGenerator([s], [], [], interval, continuous=continuous)
        gen.prepare()

        self.setup_brick_malcolm(xv=100, yv=100, xa=0.2, ya=0.2)
        self.trigger_block.rowTrigger.put_value(trigger)

        self.x_scale = [-5, 8]
        self.y_scale = [-3, 6]
        self.scan_and_plot(gen, name)

    TITLE_PATTERN = "{} xv={} yv={} xa={} ya={}"

    def Interpolation_checker(
        self,
        xv=200.0,
        yv=400.0,
        xa=1.0,
        ya=80.0,
        snake=False,
        name="",
        trigger=TRIGGER_EVERY,
        interval=0.15,
        continuous=True,
        generator=None,
        mres=None,
    ):
        if not generator:
            xs = LineGenerator("stage7", "mm", 0, 4, 5, alternate=snake)
            ys = LineGenerator("stage8", "mm", 0, 2, 3)

            gen = CompoundGenerator([ys, xs], [], [], interval, continuous=continuous)
            gen.prepare()
        else:
            gen = generator

        self.setup_brick_malcolm(xv=xv, yv=yv, xa=xa, ya=ya, mres=mres)
        self.trigger_block.rowTrigger.put_value(trigger)

        title = self.TITLE_PATTERN.format(name, xv, yv, xa, ya)
        xp, yp = self.scan_and_plot(gen, title)

        self.check_bounds(xp, "{} array x".format(name))
        self.check_bounds(yp, "{} array y".format(name))
        return xp, yp

    def check_bounds(self, a, name):
        # small amounts of overshoot are acceptable
        # UPDATE - I increased the tolerance on this because the
        # quatizing will reduce acceleration in a turnaround and therefore
        # it is possible for a motor to swing outside of the original run-up
        # distance which uses max acceleration
        npa = np.array(a)
        less_start = np.argmax((npa[0] - npa) > 0.05)
        greater_end = np.argmax((npa - npa[-1]) > 0.05)
        self.assertEqual(
            less_start, 0, "Position {} < start for {}\n{}".format(less_start, name, a)
        )
        self.assertEqual(
            greater_end, 0, "Position {} > end for {}\n{}".format(greater_end, name, a)
        )

    def test_high_acceleration(self):
        # this system test performs the same trajectory as the malcolm unit
        # test pmacchildpart_test.test_turnaround_overshoot
        self.Interpolation_checker(
            xv=17, yv=1, xa=0.1, ya=0.2, name="test_high_acceleration"
        )

    def test_stretch_sparse_points(self):
        # x=6 and y=3 combined = 7
        self.Interpolation_checker(name="Slow Sparse", trigger=TRIGGER_ROW, interval=2)
        self.Interpolation_checker(
            name="Slow Every Point", trigger=TRIGGER_EVERY, interval=2
        )

    # these tests choose parameters that create all the combinations of
    # numbers of velocity profile points in a turnaround and verify that they
    # create the same trajectory for ROW triggering and every point
    # triggering
    # todo validate that these create same trajectories (how?)
    #  note I have done so visually using the plots
    def test_trigger_x6_y3(self):
        # x=6 and y=3 combined = 7
        self.Interpolation_checker(name="Sparse x6 y3", trigger=TRIGGER_ROW)
        self.Interpolation_checker(name="Every Point x6 y3", trigger=TRIGGER_EVERY)

    def test_trigger_x6_y4(self):
        # x=6 and y=4 combined = 8
        self.Interpolation_checker(ya=1, name="Sparse x6 y4", trigger=TRIGGER_ROW)
        self.Interpolation_checker(
            ya=1, name="Every Point x6 y4", trigger=TRIGGER_EVERY
        )

    def test_trigger_x4_y3(self):
        # x=4 and y=3 combined = 5
        self.Interpolation_checker(snake=True, name="Sparse x4 y3", trigger=TRIGGER_ROW)
        self.Interpolation_checker(
            snake=True, name="Every Point x4 y3", trigger=TRIGGER_EVERY
        )

    def test_trigger_x3_y4(self):
        # x=3 and y=4 combined = 5
        self.Interpolation_checker(
            ya=0.01, snake=True, name="Sparse x3 y4", trigger=TRIGGER_ROW
        )
        self.Interpolation_checker(
            ya=0.01, snake=True, name="Every Point x3 y4", trigger=TRIGGER_EVERY
        )

    def test_trigger_spiral(self):
        self.do_spiral(trigger=TRIGGER_ROW, name="Sparse Spiral")
        self.do_spiral(trigger=TRIGGER_EVERY, name="Every Point Spiral")

    # Sharks Fin
    def test_sharks_fin(self):
        """
        attemmpt to reproduce an issue seen by Bryan where snake scan looks
        like sharks' fins rather than a saw tooth
        """
        gen_dict = json.loads(self.bryans_generator)
        gen = CompoundGenerator.from_dict(gen_dict)
        gen.prepare()

        xv, yv, xa, ya, mres, trig = 17, 17, 0.01, 0.01, 2e-5, TRIGGER_ROW
        self.setup_brick_malcolm(xv=xv, yv=yv, xa=xa, ya=ya, mres=mres)
        self.trigger_block.rowTrigger.put_value(trig)

        elapsed = self.do_a_scan(gen)

        x_demands = np.insert(
            np.array(self.traj_block.positionsX.value), 0, self.start_x
        )
        y_demands = np.insert(
            np.array(self.traj_block.positionsY.value), 0, self.start_y
        )
        times = np.insert(np.array(self.traj_block.timeArray.value), 0, 0)
        velocity_modes = np.insert(np.array(self.traj_block.velocityMode.value), 0, 0)

        title = f"parameters: xv={xv}, yv={yv}, xa={xa}, ya={ya}, mres={mres}"
        logging.info(title)
        logging.info("times, " + np.array2string(times, separator=","))
        logging.info(
            "x_demands, " + np.array2string(x_demands, precision=5, separator=",")
        )
        logging.info(
            "y_demands, " + np.array2string(y_demands, precision=5, separator=",")
        )
        logging.info(
            "velocity_modes, " + np.array2string(velocity_modes, separator=",")
        )

        self.plot_scan(title, failed=False, elapsed=elapsed)
        plot_pos_time(
            title,
            self.gather_points[0],
            self.gather_points[1],
            x_demands,
            y_demands,
            times,
        )

    bryans_generator = """
        {
            "typeid": "scanpointgenerator:generator/CompoundGenerator:1.0",
            "generators": [
                {
                    "typeid": "scanpointgenerator:generator/LineGenerator:1.0",
                    "axes": [
                        "stage8"
                    ],
                    "units": [
                        "mm"
                    ],
                    "start": [
                        0.05
                    ],
                    "stop": [
                        0.2
                    ],
                    "size": 4,
                    "alternate": true
                },
                {
                    "typeid": "scanpointgenerator:generator/LineGenerator:1.0",
                    "axes": [
                        "stage7"
                    ],
                    "units": [
                        "mm"
                    ],
                    "start": [
                        0.02
                    ],
                    "stop": [
                        0.98
                    ],
                    "size": 25,
                    "alternate": true
                }
            ],
            "excluders": [
                {
                    "typeid": "scanpointgenerator:excluder/ROIExcluder:1.0",
                    "rois": [
                        {
                            "typeid": "scanpointgenerator:roi/RectangularROI:1.0",
                            "start": [
                                0,
                                0
                            ],
                            "width": 1,
                            "height": 1,
                            "angle": 0
                        }
                    ],
                    "axes": [
                        "stage8",
                        "stage7"
                    ]
                }
            ],
            "mutators": [],
            "duration": 0.1,
            "continuous": true,
            "delay_after": 0
        }
        """
