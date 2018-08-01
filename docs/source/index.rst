.. pmac documentation master file, created by
   sphinx-quickstart on Thu Sep 15 10:05:40 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

EPICS PMAC Driver
=================

The EPICS PMAC driver is an asyn type 3 driver that supports the motor record for individual motors and coordinate system axes.  The module also provides a trajectory scan interface for executing arbitrary complex continuous scanning, and provides support for placing motors into (and taking motors out of) coordinate systems from within an EPICS application.

This documentation provides the following sections:

* An :ref:`introduction` to the module.
* A :ref:`migration` document, how to migrate from tpmac, pmacCoord and pmacUtil.
* A :ref:`user_guide` which describes how to use the pmac module, and how to execute various motions.
* A :ref:`developer_guide` which describes how to setup an IOC, and how to build on the driver capabilities.
* Detailed :ref:`design_doc` documentation.
* :ref:`code_doc`.

Contents:

.. toctree::
   :maxdepth: 2

   introduction
   migration
   user
   developer
   design
   code


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

