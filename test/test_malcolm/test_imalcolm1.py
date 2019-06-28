from scanpointgenerator import SpiralGenerator, CompoundGenerator

s = SpiralGenerator(["x", "y"], "mm", [0.0, 0.0], 5.0, scale=5)
gen = CompoundGenerator([s], [], [], 0.5)

b = self.block_view("PMAC_TEST_SCAN")

b.simultaneousAxes.put_value(["x", "y"])
b.configure(gen)
b.run()
