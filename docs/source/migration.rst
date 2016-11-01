.. _migration:

Migrating from tpmac, pmacCoord and pmacUtil
============================================

The pmac module has combined the existing tpmac, pmacUtil and pmacCoord modules into a single module.  This document describes the steps required to convert an existing IOC to make use of the new pmac module.
This guide does not describe any of the new features of the pmac module.  These features are described in the :ref:`user_guide`.

Setup
-----

This guide assumes an IOC setup with the following:

* Delta Tau geobrick with 8 motors.
* Single coordinate system with kinematics.
