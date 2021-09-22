pmac Releases
=============

The latest untagged master branch can be obtained at
https://github.com/dls-controls/pmac.

Tagged source code releases from 2-0 onward can be obtained at
https://github.com/dls-controls/pmac/releases (earlier versions are not
recommended)

The versions of EPICS base, asyn, and motor modules used for
each release can be obtained from configure/RELEASE.local. The file
configure/RELEASE.linux-x86_64.Common should be modified to reflect
site location of dependencies.

Release Notes
=============

R2-5-6 (September 22, 2021)
==========================
### Changes
* Fix iocbuilder support for RunPlc when used for single digit PLC numbers

R2-5-5 (September 1, 2021)
==========================
### Changes
* Fix servo interrupt frequency for Power PMAC
* Add motion stop template

R2-5-4 (June 25, 2021)
======================
### Changes
* Updated trajectory scan PMC to use latest demand Q variables for motor starting points

R2-5-3 (June 4, 2021)
=====================
### Changes
* Various fixes for symetrie hexapod template (#89)
* add template and EDM/CSS screens for running commands on a pmac

R2-5-2 (June 3, 2021)
=====================
### Changes
* fix power pmac trajectory scan prog 1 script
  * now includes changes applied to tpmac prog
* add etc/bootstrap with details of how to setup the system test hardware

R2-5-1 (June 1, 2021)
=====================
### Changes
* make build compatible with EPICS base 7.0.5
* this involved minor changes in the configure folder

R2-5 (June 1, 2021)
===================
### Changes
* Added back one of the original velocity modes as mode 4.
* The full set of velocity modes are now:
    * 0: Velocity = Average Prev -> Next
    * 1: Velocity = Real Prev -> Current
    * 2: Velocity = Average Prev -> Current
    * 3: Velocity = 0
    * 4: Velocity = Average Current -> Next
* Implemented trajectory scan offset and resolution for virtual motors
* Improved the direct motion interface

R2-4-22 (October 14 , 2020)
===========================
### Changes
* Add template for enabling a PLC on a pmac controller

R2-4-21 (October 06 , 2020)
===========================
### Changes
* Add template for reading just the encoder position of an axis

R2-4-20 (September 08, 2020)
============================
### Changes
* Add CSS screens for dls-pmac-control.py and pmac status
    * Copy style of EDM screens
    * Trajectory, CS, Axes and Debugging screens included

R2-4-19 (Apr 16, 2020)
======================
### Changes
* Updated asyn to 4-39
* Updated motor to 7-0dls5
* Updated busy to 1-7-2dls3

R2-4-18 (Apr 15, 2020)
======================
### Changes
* Updates to support VME pmac devices
* Avoid long at startup pause waiting to list kinematics on Power Pmac
* Fix incorrect NAXES column in xeb
* New PROG1.pmc with "Average Previous to Current" velocity mode 2
  * This supports changes in Malcolm (post version 4-2b5) to avoid accumulation of errors in long sparse trajectories

R2-4-17 (Feb 6, 2020)
=====================
### Changes
* Stop manual axes CS defs clearing previously defined groups
* Improved connection logic for PMAC controllers
    * Driver can now start in disconnected state
    * Driver can connect when hardware is available
    * Driver can disconnect and reconnect
* Fixed missing colon in motor record name
* Fixed CS bug, was looping over raw axis total instead of CS total
* Improved trajectory scan error reporting
* Updated motor builder object PORT field description

R2-1 (May 23, 2018)
===================
### Changes
* restore continuous integration (new synapps dependencies)
* Fix seg fault on startup with no brick connected
* clean up compiler warnings
* Add Power Pmac IDE project as example configuration
* Get all system tests working against PPmac
    * except trajectory scan tests - not yet supported
* Fix coordinate system control features for ppmac
* Added $(PMAC):PollAllNow PV -
    * put value 1 with callback
    * synchronizes the current brick status
* fix issues with feedrate check (now compatible with PPMAC)

R2-0-1 (May 23, 2018)
=====================
### Changes
* fix example IOC to build outside of Diamond

R2-0 (May 23, 2018)
===================
### Changes
* fixes to issues with Coordinate System motors demand and read-back
inconsistency
* fixes to any issues with race conditions due to a lack of parameter
library locking
* now works with clipper
* supports 'direct demands'
    * for avoiding the pitfalls of the motor record (particularly with
    deferred moves)
* implements coordinated deferred moves:
    *  a set of motors can be commanded to start and complete motion at
    the same time.
* updated edm GUI
* simplified templates and builder support
* stop now always works
    * stop a trajectory scan or coordinate system motor by clicking
    stop on one of the real motors
* full suite of system tests using py.test

Note that I'm now making dls-master and master the SAME. To make
this friendly to external users I have:-

* I have fixed the etc/Makefile so that it skips the folder if builder
is not available
* I have commented the required configure files to say they need
changes for each site
* I have added a non-builder example IOC (with edm GUI)


