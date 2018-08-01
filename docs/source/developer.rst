.. _developer_guide:

Developer Guide
===============

This guide provides instructions for building the pmac module and creating and building an IOC with the pmac driver.  It also demonstrates how to make use of the pmac module features for monitoring additional PMAC variables and sending commands to the PMAC.

Installation
------------

Dependencies
************

The pmac module depends on the following EPICS modules to build.  Version numbers displayed are the module versions that this module has been developed against; earlier module versions may be sufficient.

* asyn (4-26)
* calc (3-1)
* motor (6-9)
* busy (1-6-1)

Create or update the configure/RELEASE.local file and add these dependencies.

Example RELEASE.local file::

  # The following definitions must be changed for each site
  EPICS_BASE      = /home/epics/R3.14.12.3/base
  SUPPORT         = /home/epics/R3.14.12.3/modules
  
  ASYN            = $(SUPPORT)/asyn/4-26
  CALC            = $(SUPPORT)/calc/3-1
  MOTOR           = $(SUPPORT)/motor/6-9dls2
  BUSY            = $(SUPPORT)/busy/1-6-1


Building the Module
*******************

There is no special configuration required to build the module other than the dependencies mentioned in the previous section.

To build the module type make in the top level directory of the module.

::

  cd $SUPPORT/pmac/1-0
  make
  
  
Creating an IOC
---------------

This guide assumes a knowledge of creating IOCs within EPICS, and only discusses the specifics of setting up the pmac module.
An example IOC is provided in the iocs directory.  Once the dependencies are referenced this example will build an IOC that contains standard and compound motor records, status records for the controller and motors, and trajectory scan records necessary to execute trajectory scans.

For an example IOC (called example)

Makefile
********

The DBD and LIB definitions are required in the exampleApp/src/Makefile.  An example is provided below:

::

  TOP = ../..
  include $(TOP)/configure/CONFIG
  
  PROD_IOC = example
  DBD += example.dbd
  example_DBD += base.dbd
  example_DBD += asyn.dbd
  example_DBD += pmacAsynIPPort.dbd
  example_DBD += motorSupport.dbd
  example_DBD += devSoftMotor.dbd
  example_DBD += pmacAsynMotorPort.dbd
  example_DBD += busySupport.dbd
  example_DBD += calcSupport.dbd
  example_SRCS += example_registerRecordDeviceDriver.cpp
  example_LIBS += calc
  example_LIBS += busy
  example_LIBS += pmacAsynMotorPort
  example_LIBS += softMotor
  example_LIBS += motor
  example_LIBS += pmacAsynIPPort
  example_LIBS += asyn
  example_LIBS += $(EPICS_BASE_IOC_LIBS)
  example_SRCS += exampleMain.cpp
  
  include $(TOP)/configure/RULES

Database Substitutions
**********************

There are several database template files that can be used with the driver, depending on the level of interaction required.  Create a substitutions file in the exampleApp/db directory.  To include the basic PMAC controller records the following substitutions example should be added:

::

  # Macros:
  #  PORT     Underlying PMAC or GeoBrick object
  #  P        PV Prefix
  #  R        PV Suffix
  #  TIMEOUT  Timeout for controller communication
  #  CSG0..7  Names for coordinate system groups
  file $(PMAC)/db/pmacController.template
  {
    pattern { PORT, P, R, TIMEOUT, CSG0, CSG1, CSG2, CSG3, CSG4, CSG5, CSG6, CSG7 }
      { "BRICK1", "PMAC_TEST:", "GB1:", "4", "TestGroup1", "TestGroup2", "", "", "", "", "", "" }
  }

The "PORT" substitution must match the name of the port provided to the startup script line.  "P" and "R" are used to form the record names, and "TIMEOUT" is the communications timeout for the hardware.  The "CSG0".."CSG7" are names given to the coordinate system groups that have been defined for this PMAC motion controller.  If there are any unused groups (or no groups at all) then these can be left blank.

The PMAC hardware status can be monitored using a combination of the global and individual axis status template.  For the global template the following example can be used:

::

  # Macros:
  #  DEVICE    Pmac/Geobrick name
  #  PLC       PLC for CPU load monitoring, e.g. 5
  #  PORT      Delta tau motor controller
  #  NAXES     Number of axes
  #  name      Object and gui association name
  #  TIMEOUT   Template argument
  #  FEEDRATE  Template argument
  file $(PMAC)/db/pmacStatus.template
  {
  pattern { DEVICE, PLC, PORT, NAXES, name, TIMEOUT, FEEDRATE }
    { "PMAC_TEST", "5", "BRICK1", "8", "BRICK1.STAT", "1", "100" }
  }

The following macros apply:

* DEVICE - Record names are formed using this macro.
* PLC - Specify the number of the PLC that is used to calculate the CPU load of the PMAC controller hardware.  For more details on the PMAC PLCs see section <INSERT SECTION>
* PORT - The name of the asyn port of the driver which must match the name of the port provided to the startup script line.
* NAXES - The number of real motors available on the PMAC controller.
* name - Assign a name to the controller for display purposes.
* TIMEOUT - Timeout for message exchange.
* FEEDRATE - Specify the feedrate limit (%).  If the feedrate falls below this on the controller then alarms are set.

For the individual axis status template the following example can be used:

::

  # Macros:
  #  DEVICE  Pmac/Geobrick name
  #  AXIS    Axis number
  #  PORT    Asyn port
  file $(PMAC)/db/pmacStatusAxis.template
  {
  pattern { DEVICE, AXIS, PORT }
    { "PMAC_TEST", "1", "BRICK1" }
    { "PMAC_TEST", "2", "BRICK1" }
    { "PMAC_TEST", "3", "BRICK1" }
    { "PMAC_TEST", "4", "BRICK1" }
    { "PMAC_TEST", "5", "BRICK1" }
    { "PMAC_TEST", "6", "BRICK1" }
    { "PMAC_TEST", "7", "BRICK1" }
    { "PMAC_TEST", "8", "BRICK1" }
  }

The following macros apply:

* DEVICE - Record names are formed using this macro.
* AXIS - Which motor is to be monitored.  Record names also use this macro.
* PORT - The name of the asyn port of the driver which must match the name of the port provided to the startup script line.

Motor records can be used to control both motors and coordinate system axes specified within the pmac module.  Some specific template files are provided with the module that contain not only the motor record itself, but other useful records.  A full explanation of the record API is presented in the section <INSERT SECTION>.  An example of substitutions for the standard motors is shown below:

::

  # Macros:
  #  P                Device Prefix
  #  M                Device Suffix
  #  PORT             Delta tau motor controller
  #  ADDR             Address on controller
  #  DESC             Description, displayed on EDM screen
  #  MRES             Motor Step Size (EGU)
  #  VELO             Velocity (EGU/s)
  #  PREC             Display Precision
  #  EGU              Engineering Units
  #  TWV              Tweak Step Size (EGU)
  #  DTYP             DTYP of record
  #  DIR              User Direction
  #  VBAS             Base Velocity (EGU/s)
  #  VMAX             Max Velocity (EGU/s), defaults to VELO
  #  ACCL             Seconds to Velocity
  #  BDST             BL Distance (EGU)
  #  BVEL             BL Velocity (EGU/s)
  #  BACC             BL Seconds to Veloc.
  #  DHLM             Dial High Limit
  #  DLLM             Dial Low Limit
  #  HLM              User High Limit
  #  LLM              User Low Limit
  #  HLSV             HW Lim. Violation Svr
  #  INIT             Startup commands
  #  SREV             Steps per Revolution
  #  RRES             Readback Step Size (EGU)
  #  ERES             Encoder Step Size (EGU)
  #  JAR              Jog Acceleration (EGU/s^2)
  #  UEIP             Use Encoder If Present
  #  URIP             Use RDBL If Present
  #  RDBL             Readback Location, set URIP = 1 if you specify this
  #  RTRY             Max retry count
  #  DLY              Readback settle time (s)
  #  OFF              User Offset (EGU)
  #  RDBD             Retry Deadband (EGU)
  #  FOFF             Freeze Offset, 0=variable, 1=frozen
  #  ADEL             Alarm monitor deadband (EGU)
  #  NTM              New Target Monitor, only set to 0 for soft motors
  #  FEHIGH           HIGH limit for following error
  #  FEHIHI           HIHI limit for following error
  #  FEHHSV           HIHI alarm severity for following error
  #  FEHSV            HIGH alarm severity for following error
  #  SCALE            Scale factor, if pmacSetAxisScale is used this should be set
  #  HOMEVIS          If 1 then home is visible on the gui
  #  HOMEVISSTR       If HOMEVIS=0, then display this text on the gui instead
  #  name             Object name and gui association name
  #  alh              Set this to alh to add the motor to the alarm handler and send emails, 
  #  gda_name         Name to export this as to GDA
  #  gda_desc         Description to export this as to GDA
  #  SPORT            Delta tau motor controller comms port
  #  HOME             Prefix for autohome instance. Defaults to $(P). If unspecified,
  #  PMAC             Prefix for pmacStatus instance. Needed to get axis descriptions
  #  ALLOW_HOMED_SET  Set to a blank to allow this axis to have its homed
  file $(PMAC)/db/dls_pmac_asyn_motor.template
  {
  pattern { P, M, PORT, ADDR, DESC, MRES, VELO, PREC, EGU, TWV, DTYP, DIR, VBAS, VMAX, ACCL, BDST, BVEL, BACC, DHLM, DLLM, HLM, LLM, HLSV, INIT, SREV, RRES, ERES, JAR, UEIP, URIP, RDBL, RTRY, DLY, OFF, RDBD, FOFF, ADEL, NTM, FEHIGH, FEHIHI, FEHHSV, FEHSV, SCALE, HOMEVIS, HOMEVISSTR, name, alh, gda_name, gda_desc, SPORT, HOME, PMAC, ALLOW_HOMED_SET }
    { "PMAC_TEST", ":M1", "BRICK1", "1", "Motor 1", "0.001", "20", "3", "mm", "1", "asynMotor", "0", "0", "$(VELO)", "0.5", "0", "0", "", "1000", "-1000", "", "", "MAJOR", "", "1000", "", "", "", "0", "0", "", "0", "0", "0", "", "0", "0", "1", "0", "0", "NO_ALARM", "NO_ALARM", "1", "1", "Use motor summary screen", "BRICK1.MOTORS.M1", "None", "", "$(DESC)", "BRICK1port", "$(P)", "$(P)", "#" }
    { "PMAC_TEST", ":M2", "BRICK1", "2", "Motor 2", "0.001", "20", "3", "mm", "1", "asynMotor", "0", "0", "$(VELO)", "0.5", "0", "0", "", "1000", "-1000", "", "", "MAJOR", "", "1000", "", "", "", "0", "0", "", "0", "0", "0", "", "0", "0", "1", "0", "0", "NO_ALARM", "NO_ALARM", "1", "1", "Use motor summary screen", "BRICK1.MOTORS.M2", "None", "", "$(DESC)", "BRICK1port", "$(P)", "$(P)", "#" }
    { "PMAC_TEST", ":M3", "BRICK1", "3", "Motor 3", "0.001", "1", "3", "mm", "1", "asynMotor", "0", "0", "$(VELO)", "0.5", "0", "0", "", "1000", "-1000", "", "", "MAJOR", "", "1000", "", "", "", "0", "0", "", "0", "0", "0", "", "0", "0", "1", "0", "0", "NO_ALARM", "NO_ALARM", "1", "1", "Use motor summary screen", "BRICK1.MOTORS.M3", "None", "", "$(DESC)", "BRICK1port", "$(P)", "$(P)", "#" }
    { "PMAC_TEST", ":M4", "BRICK1", "4", "Motor 4", "0.001", "1", "3", "mm", "1", "asynMotor", "0", "0", "$(VELO)", "0.5", "0", "0", "", "1000", "-1000", "", "", "MAJOR", "", "1000", "", "", "", "0", "0", "", "0", "0", "0", "", "0", "0", "1", "0", "0", "NO_ALARM", "NO_ALARM", "1", "1", "Use motor summary screen", "BRICK1.MOTORS.M4", "None", "", "$(DESC)", "BRICK1port", "$(P)", "$(P)", "#" }
    { "PMAC_TEST", ":M5", "BRICK1", "5", "Motor 5", "0.001", "1", "3", "mm", "1", "asynMotor", "0", "0", "$(VELO)", "0.5", "0", "0", "", "1000", "-1000", "", "", "MAJOR", "", "1000", "", "", "", "0", "0", "", "0", "0", "0", "", "0", "0", "1", "0", "0", "NO_ALARM", "NO_ALARM", "1", "1", "Use motor summary screen", "BRICK1.MOTORS.M5", "None", "", "$(DESC)", "BRICK1port", "$(P)", "$(P)", "#" }
    { "PMAC_TEST", ":M6", "BRICK1", "6", "Motor 6", "0.001", "1", "3", "mm", "1", "asynMotor", "0", "0", "$(VELO)", "0.5", "0", "0", "", "1000", "-1000", "", "", "MAJOR", "", "1000", "", "", "", "0", "0", "", "0", "0", "0", "", "0", "0", "1", "0", "0", "NO_ALARM", "NO_ALARM", "1", "1", "Use motor summary screen", "BRICK1.MOTORS.M6", "None", "", "$(DESC)", "BRICK1port", "$(P)", "$(P)", "#" }
    { "PMAC_TEST", ":M7", "BRICK1", "7", "Motor 7", "0.001", "1", "3", "mm", "1", "asynMotor", "0", "0", "$(VELO)", "0.5", "0", "0", "", "1000", "-1000", "", "", "MAJOR", "", "1000", "", "", "", "0", "0", "", "0", "0", "0", "", "0", "0", "1", "0", "0", "NO_ALARM", "NO_ALARM", "1", "1", "Use motor summary screen", "BRICK1.MOTORS.M7", "None", "", "$(DESC)", "BRICK1port", "$(P)", "$(P)", "#" }
    { "PMAC_TEST", ":M8", "BRICK1", "8", "Motor 8", "0.001", "1", "3", "mm", "1", "asynMotor", "0", "0", "$(VELO)", "0.5", "0", "0", "", "1000", "-1000", "", "", "MAJOR", "", "1000", "", "", "", "0", "0", "", "0", "0", "0", "", "0", "0", "1", "0", "0", "NO_ALARM", "NO_ALARM", "1", "1", "Use motor summary screen", "BRICK1.MOTORS.M8", "None", "", "$(DESC)", "BRICK1port", "$(P)", "$(P)", "#" }
  }

For a description of the motor record fields see http://www.aps.anl.gov/bcda/synApps/motor/

IOC Boot Script
***************

The following IOC startup script calls are available:

* pmacAsynIPConfigure

::

  # Create IP Port (PortName, IPAddr)
  pmacAsynIPConfigure("BRICK1port", "192.168.0.1:1025")
  
This call takes a name to assign to the port, and the address of the PMAC hardware controller (with optional port number).

* pmacCreateController

::

  # Configure Model 3 Controller Driver (Controler Port,Asyn Motor Port, ADDR, Axes, MOVE_POLL, IDLE_POLL)
  pmacCreateController("BRICK1", "BRICK1port", 0, 8, 100, 1000)

The pmacCreateController call requires a name for the asyn port, the name of the low level asyn port (created previously), the address of the low level port (0), the number of motors present on the PMAC motion controller, plus a move and idle poll time (in ms).  The poll time is used to decide how often high priority status messages are requested from the controller, when all motors are either idle or if any motors are moving.

* pmacCreateAxes

::

  # Configure Model 3 Axes Driver (Controler Port, Axis Count)
  pmacCreateAxes("BRICK1", 8)

A motor axis object is created for each physical motor using this call.  The name of the controller asyn port is required along with the number of motors present.

* pmacCreateAxis

::

  # Configure Model 3 Axis Driver (Controler Port, Axis Number)
  pmacCreateAxis("BRICK1", 3)

A motor axis object is created for the specified physical motor using this call.  The name of the controller asyn port is required along with the number of the motor.

* pmacDisableLimitsCheck

::

  # Disable the limits of an axis (Controller Port, Motor Number, All axes disabled)
  pmacDisableLimitsCheck("BRICK1", 1, 1)

This will disable limit checking within the driver.  If limit checking is not disabled and the motor is found to have its limits disabled then the motor record will enter an error state.  If the motor does not have limits connected then the limit checking behaviour can be overridden with this call.  If all axes disabled is set to 1 then the motor number is ignored and all limit checking is turned off.

* pmacSetAxisScale

::

  # Set the PMAC axis scale factor (Controller Port, Motor Number, Scale factor)
  pmacSetAxisScale("BRICK1", 1, 10)
  
This call can be made to increase resolution in the motor record for a specific motor. Default value is 1.

* pmacSetOpenLoopEncoderAxis

::

  # Set an axis to use the encoder feedback from another axis (Controller Port, Motor Number, Encoder motor number)
  pmacSetOpenLoopEncoderAxis("BRICK1", 2, 4)
  
If there is an open loop axis that has an encoder coming back on a different channel then the encoder readback axis number can be set here. This ensures that the encoder will be used for the position readback. It will also ensure that the encoder axis is set correctly when performing a set position on the open loop axis. To use this function, the axis number used for the encoder must have been configured already using pmacCreateAxis.

* pmacDebug

::

  # Set the debug level (Controller Port, Debug level, Motor number (or 0 for controller), CS number (or 0 for real motors))
  pmacDebug("BRICK1", 2, 0, 0)
  
Sets the debug level for the driver.  Setting both motor number and CS number to 0 applies the debug level to the controller.

* pmacCreateCS

::

  # Create CS (CS Port, Controller Port, CSNumber, Prog)
  pmacCreateCS("CS1", "BRICK1", 1, 10)

Configure a coordinate system for the PMAC.  Takes a port name for the coordinate system as well as the name of the controller port, the coordinate system number and the motion program to execute whenever a compound axis is requested to move.

* pmacCreateCSAxes

::

  # Configure Model 3 CS Axes Driver (Controler Port, Axis Count)
  pmacCreateCSAxes("CS1", 9)

A coordinate system motor axis object is created for each axis (ABCUVWXYZ) using this call.  The name of the coordinate system asyn port is required along with the number of axes.

* pmacCreateCSAxis

::

  # pmacCreateCSAxis 'Controller port name' 'Axis number'
  pmacCreateCSAxis("CS1", 1)

A coordinate system motor axis object is created for the specified axis (ABCUVWXYZ) using this call.  The name of the coordinate system asyn port is required along with the number of the axis (1=A, 2=B, 3=C, 4=U, 5=V, 6=W, 7=X, 8=Y, 9=Z).

* pmacCreateCsGroup

::

  # Create a new CS group of axes (Controller Port, Group Number, Group Name, Axis Count)
  pmacCreateCsGroup("BRICK1", 0, "TestGroup1", 2)
  
* pmacCsGroupAddAxis

::

  # Add an axis definition to a CS group (Controller Port, Group Number, Axis Number, Axis CS Definition, CS Number)
  pmacCsGroupAddAxis(BRICK1, 0, 1, X, 1)

* pmacSetCoordStepsPerUnit

::

  # Set the PMAC CS axis scale factor (CS Port, Axis Number, Scale factor)
  pmacSetCoordStepsPerUnit("CS1", 3, 5000)


Adding autohome records
***********************

Database Substitutions
++++++++++++++++++++++

IOC Boot Script
+++++++++++++++

Adding coordinate system motors
*******************************

Database Substitutions
++++++++++++++++++++++

IOC Boot Script
+++++++++++++++

Adding trajectory scan records
******************************

Database Substitutions
++++++++++++++++++++++

IOC Boot Script
+++++++++++++++

Adding PMAC variables for monitoring
------------------------------------


