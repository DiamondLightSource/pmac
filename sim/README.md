# Simulated PMAC 8 axis controller #

This is a simple PMAC hardware simulator that is compatible with the pmac module.
Individual motors can be moved.  Trajectory scan interface is supported, 1 to 1 mapping between motor and axis:

- #1->A
- #2->B
- #3->C
- #4->U
- #5->V
- #6->W
- #7->X
- #8->Y

Motions are simplified, there is no servo loop.

## Execution ##

Execution of the simulator is simple:

	# Setup your virtual python environment and activate it
	virtualenv --no-site-packages -p /path/to/python2.7 venv27
	source venv27/bin/activate
	
	# Install npyscreen which is required for the simulator
	pip install npyscreen
	
	# Run the simulator
	python ./SimPMAC.py

