.. _design_doc:

Design
======

NOTE
----

2nd November 2016 - TODO Check design document

This design document has been taken from the original confluence page and may be out of date with the module implementation.


1. Introduction
---------------

This document describes the design of the type 3 PMAC EPICS module, and the design of the programs/PLCs running on the PMAC for the EPICS module to make use of the additional capabilities.  The document is split into sections, with the current design ideas presented and a list of tasks necessary to either implement the design, or to investigate the feasibility of the current design.  As the tasks are completed the design can be updated to reflect the findings of any tests.

In general, the new functionality proposed here should not limit the module from being used on currently deployed hardware throughout Diamond.  It must be possible to simply replace the current tpmac (and associated) modules with the new module and still be able to make use of the current simple motor record EPICS interface and virtual axes EPICS interface if required.  If the additional scanning capability is not required then no additional software should need to be downloaded to the PMAC (TBD).  Most importantly the module must still compile for VxWorks and so must not contain any STL or C++ features that are not available for the VxWorks platform.


2. PMAC Trajectory Scans
------------------------

For each axis (X,Y etc, not motor) within a coordinate system, a specified block of user memory shall be reserved on the PMAC.  Two additional memory blocks are required, one to store times and one to store arbitrary data (used for triggering).  This means that a total of 11 blocks of memory are required for PMAC trajectory scanning, although not all memory may be used at any particular time.

Each memory block shall be split in to two halves, forming buffer A and buffer B.


.. image:: PMAC_Buffer_Layout.png

Each buffer of memory shall be filled with either demand positions, times or triggers for a trajectory scan.  A single PMAC command will fill as much of a buffer as possible, but it is expected that the entire buffer (A or B) cannot be filled by one single PMAC command due to limitations on the size of messages (assumed 1024 bytes).

A pointer variable is used to keep track of the current position within the buffer that is being used for the current scan.

A pointer variable is used to keep track of the current buffer.

Each buffer will always be indexed at the same entry, so for example if buffer A index 4 is in use, then the same index is in use for all buffers (X,Y,Z,U,V,W,A,B,C,Time,User).

By reading the two pointers higher level software should be able to determine when it is safe to write to a particular buffer (A is in use, so write to B).

A motion program can be executed that follows the specified positions using a PMAC PVT move and by controlling the time between positions.  The motion program looks at the buffer pointers, collects the corresponding position demand for each axis and the delta time, then executes the necessary moves and increments the pointers.  If the end of one buffer is reached then the buffer pointer is reset and the next buffer is used, ensuring there will always be one buffer that is currently not being used by the PMAC.  The motion program is configurable to provide either point to point moves (with DWELL in between) or to follow the trajectory in a smooth manner (no discontinuities of velocity or position).

The motion program can trigger output signals after certain trajectory points have been reached.  This is acheived by reading the User buffer (which will be filled with integers) and calling an appropriate subroutine.  If the user buffer has a zero entry then this means do not call a subroutine.

2.1 EPICS Software Interface
****************************

A set of M variables have been defined that can be used by the EPICS driver to control and monitor trajectory scanning.  The M variable numbers and a description are presented below:

+-------------------+-------------------+-----------------------------------------------------------------------------------+
| PMAC Definition   | M Variable        | Description                                                                       |
+===================+===================+===================================================================================+
| Status            | M4034             | Status of motion program for EPICS - 0: Initialised, 1: Active, 2: Idle, 3: Error |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| Abort             | M4035             | Abort trigger for EPICS                                                           |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| Axes              | M4036             | An int between 1 and 511 specifying which axes to use                             |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| BufferLength      | M4037             | Length of a single buffer e.g. AX, AY                                             |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| TotalPoints       | M4038             | Total number of points scanned through                                            |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| CurrentIndex      | M4039             | Current index position in buffers                                                 |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| CurrentBuffer     | M4040             | Current buffer specifier - 0: A, 1: B                                             |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| BufferAdr_A       | M4041             | Start index of buffer A                                                           |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| BufferAdr_B       | M4042             | Start index of buffer B                                                           |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| CurrentBufferAdr  | M4043             | A or B buffer address                                                             |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| BufferFill_A      | M4044             | Fill level of buffer A                                                            |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| BufferFill_B      | M4045             | Fill level of buffer B                                                            |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| CurrentBufferFill | M4046             | The indexes that current buffer has been filled up to                             |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| PrevBufferFill    | M4047             | Fill of previous buffer to decide whether to end scan (if it wasn't full)         |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| Error             | M4048             | Error message 0: None, 1: Invalid axes value, 2: Move time of 0                   |
+-------------------+-------------------+-----------------------------------------------------------------------------------+
| Version           | M4049             | Version of the code executing on the PMAC.                                        |
+-------------------+-------------------+-----------------------------------------------------------------------------------+


The User and VelocityMode variables exist in the X memory and the time in the Y memory of the same address. They will be all be set by writing a single L value. Time is just written as required and will be the same when the Y memory is read. User and VelMode will be written to bits 25-28 and 29-32 by adding the following values to the time:

+------------+-------------+-------------+--------------------------------------------------------+
| Setting    | Bits to Set | Value       | Description                                            |
+============+=============+=============+========================================================+
| Time       | 1-24        | 0x00xxxxxxx | Time in microseconds (0 - 16777215)                    |
+------------+-------------+-------------+--------------------------------------------------------+
| VelMode 0  | None        | 0x000000000 | (Default) Calculate velocity using Prev->Next section  |
+------------+-------------+-------------+--------------------------------------------------------+
| VelMode 1  | 29          | 0x100000000 | Calculate velocity using Prev->Current section         |
+------------+-------------+-------------+--------------------------------------------------------+
| VelMode 2  | 30          | 0x200000000 | Calculate velocity using Current->Next section         |
+------------+-------------+-------------+--------------------------------------------------------+
| GoSub 1    | 25-28       | 0x010000000 | Run Subroutine 1                                       |
+------------+-------------+-------------+--------------------------------------------------------+
| GoSub 2    | 25-28       | 0x020000000 | Run Subroutine 2                                       |
+------------+-------------+-------------+--------------------------------------------------------+
| GoSub 3    | 25-28       | 0x030000000 | Run Subroutine 3                                       |
+------------+-------------+-------------+--------------------------------------------------------+
| GoSub 4    | 25-28       | 0x040000000 | Run Subroutine 4                                       |
+------------+-------------+-------------+--------------------------------------------------------+
| GoSub 5    | 25-28       | 0x050000000 | Run Subroutine 5                                       |
+------------+-------------+-------------+--------------------------------------------------------+
| GoSub 6    | 25-28       | 0x060000000 | Run Subroutine 6                                       |
+------------+-------------+-------------+--------------------------------------------------------+
| GoSub 7    | 25-28       | 0x070000000 | Run Subroutine 7                                       |
+------------+-------------+-------------+--------------------------------------------------------+

Example - Move Time of 100ms, VelMode 2, Run Subroutine 5

+---------+-------------+--------------+-------------------+
| Time    | VelMode     | GoSub        | Value to write    |
+=========+=============+==============+===================+
| 0x186A0 | 0x200000000 | 0x050000000  | 0x2500186A0       |
+---------+-------------+--------------+-------------------+

Example - Move Time of 2.5ms, VelMode 0, Run Subroutine 1

+---------+-------------+--------------+-------------------+
| Time    | VelMode     | GoSub        | Value to write    |
+=========+=============+==============+===================+
| 0x9C4   | 0x000000000 | 0x010000000  | 0x0100009C4       |
+---------+-------------+--------------+-------------------+


This interface is not yet complete, and will evolve as testing is carried out on the PMAC code.  Eventually this should present a table of specific items where appropriate (P,Q numbers, memory address writes WX, WY, WD, WI etc).

The EPICS driver reads the following data items:

* Pointer to the current half buffer (A=1, B=2)
* Pointer to the current position within the buffer that the PMAC is using
* Size of a buffer
* Starting address of the half buffers
* Motion program status
* Number of points that have been executed since starting

The EPICS driver writes the following data items:

* Specify which axes are to be used in a scan (X,Y,Z,U,V,W,A,B,C)
* Specify if a user routine to execute between each position update
* Start and stop the motion program
* Write new values into a buffer (it is not the responsibility of the PMAC to know whether position writes are correct)
* Write the value of the latest current valid index within the buffer (the PMAC understands if it has caught up/overrun)
    
3. PMAC Module
--------------

The new PMAC module contains all required code, templates, screens and documentation that supersedes the tpmac, pmacCoord and pmacUtil EPICS modules.  The motor module's interface into the tpmac module is replaced with an interface into the PMAC module.  The public interface between the motor module is not changed, but the PMAC module will restrict message flow to the PMAC hardware when necessary (specifically not allowing spurious stop commands from the EPICS motor record when certain configurations have been setup).


[Alan Greer > PMAC Design > PMAC_current.png] diagram above illustrates the currently available EPICS modules that are replaced by the PMAC module.  The three previously used EPICS modules would make multiple connections to the same asyn port, but now all messages for the PMAC are managed by the broker described below.  A set of parameters present in the pmacController class of the PMAC module are provided so that the EPICS records can query this object and the communications to the PMAC hardware are managed by the broker.


[Alan Greer > PMAC Design > PMACBlockDiagram.png]


Below is a class diagram for the PMAC driver code.

[Alan Greer > PMAC Design > PMAC_Class_Diagram.png]

The main points from the diagram:

* A single main controller class (pmacController) that inherits from the asynMotorController class.
* The pmacController class contains one instance each of the pmacBroker and pmacTrajectory classes.
* The pmacController is associated with eight instances of the pmacAxis class.
* A number of pmacCSController instances can be created (up to one for each additional coordinate system), which also inherit from the asynMotorController class and provide the motor record interface required for the CS virtual axes.
* Each pmacCSController class is associated with up to nine instances of the pmacCSAxis class.
* Both the pmacAxis and pmacCSAxis classes inherit from the asynMotorAxis class.


The table below contains the mapping from EPICS record to parameter in the pmacController class.

    This needs some discussion to check we are happy with the implementation details.


4. Communication Broker
-----------------------

Low level communications between the EPICS driver (tpmac/pmacApp/pmacAsynMotorPortSrc/pmacController.cpp) has already been implemented under for the type 3 PMAC driver.  The aim is to implement a broker on top of the existing low level methods that can be used to tightly control the traffic between the PMAC and the controlling application.

4.1 Reading
***********

The broker can read from the PMAC at three polling rates, slow, medium and fast.  The current global status method of the controller class shall be maintained with its polling rate.  This method will no longer have a hardcoded set of messages to send to the PMAC, and instead it will decide if it is necessary to call one of three new methods, slow, medium, fast read.  A corresponding store (not std::map (sad)) shall be maintained for each read method, and the store contains data items that should be read whenever that method is called.  These data requests are batched into a single (or minimal) request and sent to the PMAC.  The response is received and decoded.  Higher level software can add or remove(?) items from the stores, and register for updates when the item has been read from the PMAC.

After discussion with Giles we think the following proposed polling is better:

    Standard polling (which runs at one rate when motors are not moving, and the faster rate when motors are moving)
    Slow polling (which always at a slow rate, and the rate does not need to change if motors are moving)

4.2 Writing
***********

Currently messages are simply sent to the PMAC as they arrive, often individually.  The broker still accepts individual messages for immediate despatch to the PMAC, but it also provides a batch mode which allows multiple PMAC commands to be sent as a single batched message.

4.3 Locking
***********

The existing controller class low level write read method already uses the pasynOctetSyncIO asyn interface, which provides locking on the specified port.  This only locks from the point of view of the application, no external locking mechanism is required for the PMAC.

4.4 Proposed Methods
********************

The following methods are added to the existing type 3 pmacController class to provide the necessary interface as described above.
Method  Parameters  Description 
addReadVariable Type (SLOW | MEDIUM | FAST) PMAC Variable (string) Adds the PMAC variable to the specified container ready for reading.
deleteReadVariable Type (SLOW | MEDIUM | FAST) PMAC Variable (string) Deletes the PMAC variable from the specified container.
registerForRead Type (SLOW | MEDIUM | FAST) Callback (ptr to callback method) User data (void * used to access calling object) Register interest in data from one of the polling loops.  The callback is called whenever the data container has been updated by the PMAC.  A copy of data items returned by the PMAC is passed to the callback.
immediateWriteRead  PMAC command (string) This method will result in the supplied message being sent to the PMAC as soon as is possible.  Any response will be returned from the method.
startBatchWrite Begin a new batch of write messages.
addBatchWrite PMAC command (string) Add a new message to the batch.
sendBatch All currently batched messages shall be sent as a single PMAC message.  Responses from the messages will be returned from this method.

5. PMAC Controller
------------------

5.1 Status Polling
******************

The current type 3 pmacController inherits from the asynMotorController class, which provides single thread polling of status at one of two predetermined rates (the slow rate when nothing is moving and the fast rate when something is moving).  The pmacController continues to use this polling method, but it polls different variables during different poll method calls.  The pmacController contains three storage containers for PMAC polled status items.  During a poll call one or more of the containers keys are built into a request string that is sent to the PMAC.  The response is received from the PMAC and the return values stored in the container as the value of the corresponding key.  Once the PMAC has been interrogated for the status information then any registered callbacks are notified of the new data arrival.  The three containers are

Fast container.  Items in this container are requested at every poll.

Medium container.  Items in this container are requested once every two polls.

Slow container.  Items in this container are requested once every five polls.

This method of tiered polling reduces the number of messages that are sent to the PMAC for status items.  With the necessity for sending possibly large batches of data points to the PMAC it will be useful to keep the general status write/reads in once place and cutting down on messages sent to the PMAC should offer better performance.
Poll Rate Items Read from PMAC
Slow (0.1 Hz always)  

Motor coordinate system assignments

Kinematics
Medium (1 Hz always)  

Custom motion program status

Encoder loss status

Global status

Q  variable status
Fast (Either 1 Hz or 10 Hz) 

Motor positions

Motor following error

Motor status

PMAC buffer pointers

CS status


5.2 Trajectory Interface
************************

The current type 3 asyn motor controller class already provides trajectory interface code.  The following parameters are provided at the controller level.  Note that many of these parameters are not used in the base class, and will be utilised by the PMAC specific child class to provide the required functionality.
Parameter Type  

Description
profileNumAxes  Int32 Not currently used
profileNumPoints  Int32 Number of points in the current profile
profileCurrentPoint Int32 Currently executing profile point
profileNumPulses  Int32 
profileStartPulses  Int32 
profileEndPulses  Int32 
profileActualPulses Int32 
profileNumReadbacks Int32 
profileTimeMode Int32 PROFILE_TIME_MODE_FIXED | PROFILE_TIME_MODE_ARRAY.  Used to specify either an array of delta time values corresponding to the array of positions, or a single fixed time to be used for each point.
profileFixedTime  Float64 This parameter is used as the fixed time in between each point if PROFILE_TIME_MODE_FIXED is selected.
profileTimeArray  Float64Array  Array to store profile positions.  There is a maximum specified that is used for allocation when the controller is created.
profileAcceleration Float64 
profileMoveMode Int32 

profileBuild (cmd /

state /

status /

message)
  

Int32

Int32

Int32

String
  The command is used to call buildProfile method in controller.

profileExecute (cmd /

state /

status /

message)
  

Int32

Int32

Int32

String
  The command is used to call executeProfile method in controller.

profileReadback (cmd /

state /

status /

message)
  

Int32

Int32

Int32

String
  The command is used to call readbackProfile method in controller.

profileAbort (cmd /

state /

status /

message)
  

Int32

Int32

Int32

String
The command is used to call abortProfile method in controller.
profileUseAxis  Int32 Axis specific.  Switch to turn on or off trajectory scan for this axis.
profilePositions  Float64Array  Axis specific.  This array contains the trajectory points for the axis.
profileReadbacks  Float64Array  Axis specific.
profileFollowingErrors  Float64Array  Axis specific.
profileMotorResolution  Float64 Axis specific.
profileMotorDirection Int32 Axis specific.
profileMotorOffset  Float64 Axis specific.

The controller class is currently setup to build and execute profiles on a per axis basis.  This results in several calls to executeProfile, one for each axis which has been selected to be included for the profile move.  For the PMAC controller the design implements a slightly different pattern:

A trajectory move thread is created for the controller object.  The trajectory move thread is responsible for sending down motion program execution statements, aborting running motion programs if necessary, and for sending batches of positions to write into the PMAC memory.  By sending batches of positions from the trajectory move thread, all required axis positions can be combined into a single PMAC write and sent simultaneously.  Only one single counter for the current half buffer and position is required on board the PMAC as there is no individual thread for each axis, which reduces the complexity of the trajectory thread.

The implementation of trajectory scanning for the PMAC Controller is summarised below:

    For each axis, the profileUseAxis parameter is switched to "use" or "not use".
    For each axis to be included in the trajectory move, the position array is passed into the controller (profilePositions parameter).
    The profileTimeMode parameter is set to PROFILE_TIME_MODE_FIXED or PROFILE_TIME_MODE_ARRAY.
    Either the profileFixedTime parameter is set to the required demand (if profileTimeMode is PROFILE_TIME_MODE_FIXED), or the time array is passed into the controller (profileTimeArray parameter).
    The profileBuild command parameter is executed to build all relevant axis profiles.
    The profileExecute command parameter is issued.  This sends an EPICS event to wake up the profile thread.
    On the first iteration, the profile thread sends down the position and time demands for the first half-buffer.
    On subsequent iterations, the profile thread sends down the position and time demands (if necessary) for the half buffer that is not currently under execution in the PMAC.
    To abort a profile move the profileAbort command parameter is issued.  The profile thread will be checking for the abort signal and stops sending half-buffer updates if the signal is received.
    The profileAbort command also sends the abort command to the PMAC.


5.3 Deferred Moves
******************

Deferred moves are now implemented by execution of a single point trajectory scan.  This removes the higher level EPICS code complexities and associated problems that are present in the current driver code.  As explained in the EPICS Level Control section below, control of axes by the motor record will be revoked by the driver during trajectory scans.  This removes the problem caused by the EPICS motor record issuing stops when soft limits are reached (especially for coordinate system axes not currently in control).  It is TBD if a deferred move should have its own set of parameters.

5.4 PMAC Axis Groups
********************

The grouping of axes is defined during initialisation of the controller class.  An axes group is specified through the pmacCreateCsGroup IOC shell command, passing the name of the controller, the name and number of the CS group and the number of axes in the group.  Axes are added to individual groups by calling pmacCsGroupAddAxis and supplying the name of the controller, the CS group number, the axis number, the string mapping to be supplied to the PMAC and the coordinate system number.  Axes can be switched into any specified group by setting the PMAC_C_CoordSysGroup parameter.

There is a single predefined group that will always be available (group 1) in the pmacController.  This group places all axes into coordinate system 1 with the following mappings:

&1 #1->A

&1 #2->B

&1 #3->C

&1 #4->U

&1 #5->V

&1 #6->W

&1 #7->X

&1 #8->Y

6. PMAC Axis
------------

The current type 3 pmacAxis class inherits from the asynMotorAxis class.  Methods within this class are called by the controller class when axis specific parameters are set.  For the initial implementation of the new trajectory scan capable classes minimal changes are required to the pmacAxis class:

    Wherever the pmacAxis class needs make direct calls to the low level write/read methods, these calls are serviced through the broker, and additional checks are made to ensure high level motor controller commands are not sent when the axes are not active (this could be due to another group selected or a trajectory scan executing).
    The axis status polling is now carried out through the controller poll methods, with the axis registering a callback for required updates.  This cuts down on the number of messages sent to the PMAC hardware.


7. PMAC Coordinate System Axis
------------------------------

The pmacCSAxis class is a new addition to the type 3 PMAC module and is a subclass of the asynMotorAxis class.  The class is very similar to the pmacAxis class, but executes commands on a coordinate system axis rather than a raw motor.  The coordinate system is specified when the class is instantiated along with the axis within the coordinate system.  The same public API is provided by the coordinate system axis class, including the control and status parameters and the ability to trajectory scan the axis.  The interface provided allows an EPICS motor record to drive the axis in exactly the same way as the current type 2 implementation, and is fully backwards compatible to any higher level software with the exception of deferred moves that are now executed by using a single point trajectory scan.

8. EPICS Level Control
----------------------

Below is the block diagram for the current design.

[Alan Greer > PMAC Design > PMACBlockDiagram.png]

For setting up a geobrick with eight motors the following startup script shell commands are required:

    Single call to create the pmac controller.  Only one controller is required for the entire setup, controller contains the broker.
    Single call to create eight real motors.  Motors are numbered 1 to 8, all communications pass through the controller.
    Exactly one call for CS 1.  TBD how this is different from the call below.
    One call for each additional coordinate system, creating nine axes X,Y,Z,U,V,W,A,B,C.  Each axis is numbered sequentially starting from 9, and all communicate through the controller.
    Calls made as necessary to create the CS groups.
    Calls made as necessary to add motors to CS group axes.

For setting up a geobrick with eight motors the following records are required:

    Each real motor has a corresponding template, containing motor record and records that currently reside in pmacUtil.  All records access data through parameters (from the controller)
    Each CS has a template that contains records to select the CS for trajectory scanning.
    Each CS has a template that contains a time series record for that coordinate system.  Uses address to notify controller which CS time series has been set?
    Each CS axis has a corresponding template, containing a motor record and specific axis records for the CS axis, along with a position array record, and status records.  Uses address to notify controller which axis. 

The table below can be filled as the records are defined that will form the PMAC API.
Template Name Record Name Record Type Record Description

autohome.vdb
  
  
  
brake.template  
  
  
compensationtable.vdb 
  
  
deltatauRotaryHack.template 
  
  
dls_pmac_asyn_motor_no_coord.template 
  
  
dls_pmac_asyn_motor.template  
  
  
dls_pmac_cs_asyn_motor.template 
  
  
dls_pmac_patch_asyn_motor.template  
  
  
eloss_kill_autohome_records.template  
  
  
encoderReadback.template  
  
  
energise_streams.vdb  
  
  
energise.vdb  
  
  
energyModes.vdb 
  
  
gather.vdb  
  
  
motion_protection.template  
  
  
motion_stop.template  
  
  
motorstatus.vdb 
  
  
nanomotor_axis.template 
  
  
nanomotor_pmac.template 
  
  
pmacDeferMoves.template 
  
  
pmacStatus32Axes.vdb  
  
  
pmacStatus8Axes.vdb 
  
  
pmacStatusAxis.vdb  
  
  
pmacStatus.vdb  
  
  
pmacVariableWriteRecords.vdb  
  
  
pmacVariableWrite.vdb 
  
  
positionCompare_nojitter.vdb  
  
  
positionCompare.vdb 
  
  
translated_motor.template 
  
  
trigger_pause.vdb 
  
