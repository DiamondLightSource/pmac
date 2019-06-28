from unittest import TestCase
from dls_pmaclib.dls_pmacremote import PmacEthernetInterface
from dls_pmaclib.pmacgather import PmacGather


class TestTrajectories(TestCase):
    def test_spiral(self):
        pmac1 = PmacEthernetInterface(verbose=True)
        pmac1.setConnectionParams('172.23.240.97', 1025)
        pmac1.connect()

        g = PmacGather(pmac1)
        axes = [3, 2]
        g.gatherConfig(axes, 400, 10)
        g.gatherTrigger()
        data = g.collectData()
        g.parseData(data)
        for c in g.channels:
            print(c.scaledData)
