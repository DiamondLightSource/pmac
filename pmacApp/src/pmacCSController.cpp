/*
 * pmacCSController.cpp
 *
 *  Created on: 24 Feb 2016
 *      Author: Alan Greer
 *
 *
\file

This is a driver for Delta-Tau PMAC Coordinate Systems,
designed to interface with the "motor API" style interface that
allows drivers to be used with the standard asyn device and driver
support for the motor record

To use this driver, you should create a co-ordinate system with kinematics and a
position reporting PLC as detailed in the
<a href="http://www.deltatau.com/manuals/pdfs/TURBO%20PMAC%20USER%20MANUAL.pdf">
Delta Tau Turbo PMAC User Manual</a>, along with a position reporting PLC. You
should follow the conventions set out in the
<a href="../BLS-GEN-CTRL-0005.doc">DLS PMAC Programming Standards</a>

The driver uses the following interface:
- CS Axes A, B, C, U, V, W, X, Y, Z are mapped to Q1..9 in kinematic code by the
PMAC
- The EPICS driver expects the position reporting PLC to report the positions of
these axes on Q81..89, in EGUs
- The driver applies a scale factor (10000.0 by default, modified by
pmacSetCoordStepsPerUnit). This is needed because the motor record can only
drive in integer "steps". The motor record needs to have the inverse of this
scale factor in MRES (0.0001 by default)
- The driver is numbered from 0, so the motor record with address 0 connects
to axis A which has readback in Q81
- When a move is demanded, the following is done:
 - The CS is aborted to put the motors in closed loop mode
 - Isx87 (Acceleration) is set to VELO / ACCL * 1000
 - Isx89 (Feedrate) is set to the motor record VELO
 - Demand value in EGUs is written to the relevant Q Variable in Q71..79
 - Motion program specified in the startup script (probably
PROG_10_CS_motion.pmc) is run.

The following code will open an IP Port to an ethernet PMAC, then wrap CS2
in a driver with ref 0 running prog 10, polling for position every 0.5s
when the motors aren't moving, and every 0.1s when they are, and set the scale
factor for X on CS2 to 100000.0, so the relevant motor record will need an MRES
of 1.0/100000.0
\code
# Create IP Port (IPPort, IPAddr)
# This function lives in tpmac
pmacAsynIPConfigure("BRICK1port", "172.23.243.156:1025")

# Create CS (ControllerPort, Addr, CSNumber, CSRef, Prog)
pmacAsynCoordCreate("BRICK1port", 0, 2, 0, 10)
# Configure CS (PortName, DriverName, CSRef, NAxes)
drvAsynMotorConfigure("BRICK1CS2", "pmacAsynCoord", 0, 9)
# Set scale factor (CS_Ref, axis, stepsPerUnit)
pmacSetCoordStepsPerUnit(0, 6, 100000.0)
# Set Idle and Moving poll periods (CS_Ref, PeriodMilliSeconds)
pmacSetCoordIdlePollPeriod(0, 500)
pmacSetCoordMovingPollPeriod(0, 100)
\endcode
 */

#include <iocsh.h>
#include <drvSup.h>
#include <registryFunction.h>
#include <epicsExport.h>
#include <sstream>
#include "pmacCSController.h"
#include "pmacController.h"

const epicsUInt32 pmacCSController::PMAC_ERROR_PRINT_TIME_ = 600; //seconds

const epicsUInt32 pmacCSController::CS_STATUS1_RUNNING_PROG = (0x1 << 0);
const epicsUInt32 pmacCSController::CS_STATUS1_SINGLE_STEP_MODE = (0x1 << 1);
const epicsUInt32 pmacCSController::CS_STATUS1_CONTINUOUS_MODE = (0x1 << 2);
const epicsUInt32 pmacCSController::CS_STATUS1_MOVE_BY_TIME_MODE = (0x1 << 3);
const epicsUInt32 pmacCSController::CS_STATUS1_CONTINUOUS_REQUEST = (0x1 << 4);
const epicsUInt32 pmacCSController::CS_STATUS1_RADIUS_INC_MODE = (0x1 << 5);
const epicsUInt32 pmacCSController::CS_STATUS1_A_INC = (0x1 << 6);
const epicsUInt32 pmacCSController::CS_STATUS1_A_FEEDRATE = (0x1 << 7);
const epicsUInt32 pmacCSController::CS_STATUS1_B_INC = (0x1 << 8);
const epicsUInt32 pmacCSController::CS_STATUS1_B_FEEDRATE = (0x1 << 9);
const epicsUInt32 pmacCSController::CS_STATUS1_C_INC = (0x1 << 10);
const epicsUInt32 pmacCSController::CS_STATUS1_C_FEEDRATE = (0x1 << 11);
const epicsUInt32 pmacCSController::CS_STATUS1_U_INC = (0x1 << 12);
const epicsUInt32 pmacCSController::CS_STATUS1_U_FEEDRATE = (0x1 << 13);
const epicsUInt32 pmacCSController::CS_STATUS1_V_INC = (0x1 << 14);
const epicsUInt32 pmacCSController::CS_STATUS1_V_FEEDRATE = (0x1 << 15);
const epicsUInt32 pmacCSController::CS_STATUS1_W_INC = (0x1 << 16);
const epicsUInt32 pmacCSController::CS_STATUS1_W_FEEDRATE = (0x1 << 17);
const epicsUInt32 pmacCSController::CS_STATUS1_X_INC = (0x1 << 18);
const epicsUInt32 pmacCSController::CS_STATUS1_X_FEEDRATE = (0x1 << 19);
const epicsUInt32 pmacCSController::CS_STATUS1_Y_INC = (0x1 << 20);
const epicsUInt32 pmacCSController::CS_STATUS1_Y_FEEDRATE = (0x1 << 21);
const epicsUInt32 pmacCSController::CS_STATUS1_Z_INC = (0x1 << 22);
const epicsUInt32 pmacCSController::CS_STATUS1_Z_FEEDRATE = (0x1 << 23);

const epicsUInt32 pmacCSController::CS_STATUS2_CIRCLE_SPLINE_MODE = (0x1 << 0);
const epicsUInt32 pmacCSController::CS_STATUS2_CCW_RAPID_MODE = (0x1 << 1);
const epicsUInt32 pmacCSController::CS_STATUS2_2D_CUTTER_COMP = (0x1 << 2);
const epicsUInt32 pmacCSController::CS_STATUS2_2D_LEFT_3D_CUTTER = (0x1 << 3);
const epicsUInt32 pmacCSController::CS_STATUS2_PVT_SPLINE_MODE = (0x1 << 4);
const epicsUInt32 pmacCSController::CS_STATUS2_SEG_STOPPING = (0x1 << 5);
const epicsUInt32 pmacCSController::CS_STATUS2_SEG_ACCEL = (0x1 << 6);
const epicsUInt32 pmacCSController::CS_STATUS2_SEG_MOVING = (0x1 << 7);
const epicsUInt32 pmacCSController::CS_STATUS2_PRE_JOG = (0x1 << 8);
const epicsUInt32 pmacCSController::CS_STATUS2_CUTTER_MOVE_BUFFD = (0x1 << 9);
const epicsUInt32 pmacCSController::CS_STATUS2_CUTTER_STOP = (0x1 << 10);
const epicsUInt32 pmacCSController::CS_STATUS2_CUTTER_COMP_OUTSIDE = (0x1 << 11);
const epicsUInt32 pmacCSController::CS_STATUS2_DWELL_MOVE_BUFFD = (0x1 << 12);
const epicsUInt32 pmacCSController::CS_STATUS2_SYNCH_M_ONESHOT = (0x1 << 13);
const epicsUInt32 pmacCSController::CS_STATUS2_EOB_STOP = (0x1 << 14);
const epicsUInt32 pmacCSController::CS_STATUS2_DELAYED_CALC = (0x1 << 15);
const epicsUInt32 pmacCSController::CS_STATUS2_ROTARY_BUFF = (0x1 << 16);
const epicsUInt32 pmacCSController::CS_STATUS2_IN_POSITION = (0x1 << 17);
const epicsUInt32 pmacCSController::CS_STATUS2_FOLLOW_WARN = (0x1 << 18);
const epicsUInt32 pmacCSController::CS_STATUS2_FOLLOW_ERR = (0x1 << 19);
const epicsUInt32 pmacCSController::CS_STATUS2_AMP_FAULT = (0x1 << 20);
const epicsUInt32 pmacCSController::CS_STATUS2_MOVE_IN_STACK = (0x1 << 21);
const epicsUInt32 pmacCSController::CS_STATUS2_RUNTIME_ERR = (0x1 << 22);
const epicsUInt32 pmacCSController::CS_STATUS2_LOOKAHEAD = (0x1 << 23);

const epicsUInt32 pmacCSController::CS_STATUS3_LIMIT = (0x1 << 1);

const std::string pmacCSController::CS_RUNTIME_ERRORS[] = {
  "No run-time error",
  "Insufficient calculation time",
  "Program counter out of range (too low)",
  "Program counter out of range (too high)",
  "Unlinked conditional statement",
  "Subroutine stack overflow",
  "Jump to non-existant label",
  "Cutter compensation interference error",
  "Forward kinematic execution error",
  "Inverse kinematic execution error",
  "No axes remaining in CS"
};

/**
 * Create a driver instance to communicate with a given coordinate system
 *
 * @param portName The Asyn port used to communicate with the PMAC card
 * @param controllerPortName
 * @param addr The Asyn address of the PMAC (usually 0)
 * @param csNo The co-ordinate system to connect to
 * @param ref A unique reference, used by the higher layer software to reference this C.S.
 * @param program The PMAC program number to run to move the C.S.
 */
pmacCSController::pmacCSController(const char *portName, const char *controllerPortName, int csNo,
                                   int program)
        : asynMotorController(portName, 10, (int)NUM_MOTOR_DRIVER_PARAMS + (int)NUM_PMAC_CS_PARAMS,
                              asynInt32ArrayMask, // For user mode and velocity mode
                              0, // No addition interrupt interfaces
                              ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                              1, // autoconnect
                              0, 0),  // Default priority and stack size
          pmacDebugger("pmacCSController"),
          portName_(portName),
          csNumber_(csNo),
          progNumber_(program),
          movesDeferred_(0),
          csMoveTime_(0) {
  static const char *functionName = "pmacCSController";

  // Init the status
  status_[0] = 0;
  status_[1] = 0;
  status_[2] = 0;

  setIntegerParam(profileBuild_, 0);

  pAxes_ = (pmacCSAxis **) (asynMotorController::pAxes_);

  pC_ = (pmacController*) findAsynPortDriver(controllerPortName);
  if (!pC_) {
    debug(DEBUG_ERROR, functionName, "ERROR port not found", controllerPortName);
  }

  // tell the broker to lock me when polling the brick
  pC_->registerForLock(this);

  //Create controller-specific parameters
  bool paramStatus = true;
  createParam(PMAC_CS_FirstParamString, asynParamInt32, &PMAC_CS_FirstParam_);
  createParam(PMAC_CS_CsMoveTimeString, asynParamFloat64, &PMAC_CS_CsMoveTime_);
  createParam(PMAC_CS_RealMotorNumberString, asynParamInt32, &PMAC_CS_RealMotorNumber_);
  createParam(PMAC_CS_MotorScaleString, asynParamInt32, &PMAC_CS_MotorScale_);
  createParam(PMAC_CS_MotorResString, asynParamFloat64, &PMAC_CS_MotorRes_);
  createParam(PMAC_CS_MotorOffsetString, asynParamFloat64, &PMAC_CS_MotorOffset_);
  createParam(PMAC_CS_CsAbortString, asynParamInt32, &PMAC_CS_Abort_);
  createParam(PMAC_CS_ForwardKinematicString, asynParamOctet, &PMAC_CS_ForwardKinematic_);
  createParam(PMAC_CS_InverseKinematicString, asynParamOctet, &PMAC_CS_InverseKinematic_);
  createParam(PMAC_CS_QVariablesString, asynParamOctet, &PMAC_CS_QVariables_);
  createParam(PMAC_CS_DirectMoveString, asynParamFloat64, &PMAC_CS_DirectMove_);
  createParam(PMAC_CS_DirectResString, asynParamFloat64, &PMAC_CS_DirectRes_);
  createParam(PMAC_CS_DirectOffsetString, asynParamFloat64, &PMAC_CS_DirectOffset_);
  createParam(PMAC_CS_LastParamString, asynParamInt32, &PMAC_CS_LastParam_);
  paramStatus = ((setDoubleParam(PMAC_CS_CsMoveTime_, csMoveTime_) == asynSuccess) && paramStatus);
  for(int index=0; index<=PMAC_CS_AXES_COUNT; index++) {
    paramStatus = ((setIntegerParam(
            index, PMAC_CS_RealMotorNumber_, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(
            index, PMAC_CS_MotorScale_, 10000)  == asynSuccess) &&paramStatus);
  }

  if (!paramStatus) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s Unable To Set Driver Parameters In Constructor.\n", functionName);
  }

  pC_->registerCS(this, portName, csNumber_);
}

pmacCSController::~pmacCSController() {
}

void pmacCSController::initComplete() {
  storeKinematics();
  pmacCSAxis *pAxis = NULL;
  // Notify all registered axes of the good connection
  for (int axis = 0; axis < numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (pAxis != NULL) {
      pAxis->goodConnection();
    }
  }
}

void pmacCSController::badConnection() {
  pmacCSAxis *pAxis = NULL;
  // Notify all registered axes of the bad connection
  for (int axis = 0; axis < numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (pAxis != NULL) {
      pAxis->badConnection();
    }
  }
}

bool pmacCSController::initialised(void) {
  return pC_->initialised();
}

std::string pmacCSController::getPortName() {
  return portName_;
}

/**
 * Deal with controller specific epicsInt32 params.
 * @param pasynUser
 * @param value
 * @param asynStatus
 */
asynStatus pmacCSController::writeInt32(asynUser *pasynUser, epicsInt32 value) {
  int function = pasynUser->reason;
  bool status = true;
  pmacCSAxis *pAxis = NULL;
  const char *name[128];
  char command[PMAC_CS_MAXBUF] = {0};
  char response[PMAC_CS_MAXBUF] = {0};
  static const char *functionName = "writeInt32";

  debug(DEBUG_FLOW, functionName);

  getParamName(function, name);
  debug(DEBUG_VARIABLE, functionName, "Parameter Updated", *name);
  pAxis = this->getAxis(pasynUser);
  if (!pAxis) {
    return asynError;
  }

  status = (pAxis->setIntegerParam(function, value) == asynSuccess) && status;

  if (function == motorDeferMoves_) {
    debug(DEBUG_VARIABLE, functionName, "Motor defer value", value);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s: Setting deferred move mode on PMAC %s to %d\n", functionName, portName, value);
    if (value == 0) {
      status = (this->processDeferredMoves() == asynSuccess) && status;
    }
    this->movesDeferred_ = value;
  } else if (function == PMAC_CS_MotorScale_) {
    pAxis->scale_ = value;
  } else if (function == PMAC_CS_Abort_) {
    sprintf(command, "&%dA", csNumber_);
    debug(DEBUG_VARIABLE, functionName, "Command sent to PMAC", command);
    status = (this->immediateWriteRead(command, response) == asynSuccess) && status;
    pC_->csResetAllDemands = true;
  }

  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = (asynMotorController::writeInt32(pasynUser, value) == asynSuccess) && status;

  if (!status) {
    return asynError;
  }
  return asynSuccess;
}

/**
 * Deal with controller specific epicsInt32 params.
 * @param pasynUser
 * @param value
 * @param asynStatus
 */
asynStatus pmacCSController::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
  int function = pasynUser->reason;
  bool status = true;
  pmacCSAxis *pAxis = NULL;
  const char *name[128];
  static const char *functionName = "writeFloat64";
  char command[PMAC_CS_MAXBUF] = {0};
  char response[PMAC_CS_MAXBUF] = {0};

  debug(DEBUG_FLOW, functionName);

  getParamName(function, name);
  debug(DEBUG_VARIABLE, functionName, "Parameter Updated", *name);
  pAxis = this->getAxis(pasynUser);
  if (!pAxis) {
    return asynError;
  }

  if ((function == PMAC_CS_CsMoveTime_) ) {
    csMoveTime_ = value;
    if (this->pC_->useCsVelocity) {
      sprintf(command, "&%dQ70=%f", csNumber_, value);
      debug(DEBUG_VARIABLE, functionName, "Command sent to PMAC", command);
      status = (this->immediateWriteRead(command, response) == asynSuccess) && status;
    }
  } else if (function == PMAC_CS_MotorRes_){
    pAxis->setKinematicResolution(value);
  } else if (function == PMAC_CS_MotorOffset_){
    pAxis->setKinematicOffset(value);
  } else if (function == PMAC_CS_DirectMove_){
    double baseVelocity = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    getDoubleParam(pAxis->axisNo_, motorVelBase_, &baseVelocity);
    getDoubleParam(pAxis->axisNo_, motorVelocity_, &velocity);
    getDoubleParam(pAxis->axisNo_, motorAccel_, &acceleration);
    pAxis->directMove(value, baseVelocity, velocity, acceleration);
    pAxis->setIntegerParam(motorStatusDone_, 0);
    pAxis->callParamCallbacks();
    wakeupPoller();
  }

  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = (asynMotorController::writeFloat64(pasynUser, value) == asynSuccess) && status;

  return status ? asynSuccess : asynError;
}


asynStatus pmacCSController::processDeferredMoves(void) {
  asynStatus status = asynSuccess;
  char command[PMAC_MAXBUF] = {0};
  char fullCommand[PMAC_MAXBUF] = {0};
  char response[PMAC_MAXBUF] = {0};
  pmacCSAxis *pAxis = NULL;
  int executeDeferred = 0;
  static const char *functionName = "processDeferredMoves";

  debug(DEBUG_FLOW, functionName);
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  //Build up combined move command for all axes involved in the deferred move.
  for (int axis = 0; axis < numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (pAxis != NULL) {
      if (pAxis->deferredMove_) {
        sprintf(command, "%s %s", command, pAxis->deferredCommand_);
        executeDeferred = 1;
      }
    }
  }

  if (executeDeferred == 1) {
    this->makeCSDemandsConsistent();
    if (this->getProgramNumber() != 0) {
      // Abort current move to make sure axes are enabled
      status = this->immediateWriteRead(
              ((pmacController *) pC_)->pHardware_->getCSEnableCommand(csNumber_).c_str(),
              response);

      sprintf(fullCommand, "&%d%s Q70=%f B%dR", this->getCSNumber(), command, this->csMoveTime_,
              this->getProgramNumber());
      if(status == asynSuccess) {
        status = this->immediateWriteRead(fullCommand, response);
      }
    }
  }

  //Clear deferred move flag for the axes involved.
  for (int axis = 0; axis < numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (pAxis != NULL) {
      if (pAxis->deferredMove_) {
        pAxis->deferredMove_ = 0;
      }
    }
  }

  return status;
}

void pmacCSController::setDebugLevel(int level, int axis) {
  // Check if an axis or controller wide debug is to be set
  if (axis == 0) {
    printf("Setting PMAC CS controller debug level to %d\n", level);
    // Set the level for the controller
    this->setLevel(level);
  } else {
    if (this->getAxis(axis) != NULL) {
      printf("Setting PMAC CS axis %d debug level to %d\n", axis, level);
      this->getAxis(axis)->setLevel(level);
    }
  }
}

bool pmacCSController::getMoving() {
  pmacCSAxis *pA = NULL;
  bool anyMoving = false;
  static const char *functionName = "getMoving";
  debug(DEBUG_TRACE, functionName, "Called");

  this->lock();
  for (int i = 0; i < numAxes_; i++) {
    pA = getAxis(i);
    if (!pA) continue;
    anyMoving |= pA->getMoving();
  }
  this->unlock();
  debug(DEBUG_VARIABLE, functionName, "Any axes moving", (int) anyMoving);
  return anyMoving;
}

int pmacCSController::getCSNumber() {
  return csNumber_;
}

int pmacCSController::getProgramNumber() {
  return progNumber_;
}

csStatus pmacCSController::getStatus() {
  return cStatus_;
}

std::string pmacCSController::getVelocityCmd(double velocity, double steps) {
  return ((pmacController *) pC_)->pHardware_->getCSVelocityCmd(csNumber_, velocity,
                                                                steps);
}

std::string pmacCSController::getCSAccTimeCmd(double time) {
  return ((pmacController *) pC_)->pHardware_->getCSAccTimeCmd(csNumber_, time);
}

void pmacCSController::callback(pmacCommandStore *sPtr, int type) {
  std::string value;
  char remove[PMAC_CS_MAXBUF];
  char key[PMAC_CS_MAXBUF];
  static const char *functionName = "callback";
  debug(DEBUG_TRACE, functionName, "Coordinate system status callback");

  if(type == pmacMessageBroker::PMAC_PRE_FAST_READ) {
    // Parse the status
    ((pmacController *) pC_)->pHardware_->parseCSStatus(csNumber_, sPtr, cStatus_);
    status_[0] = cStatus_.stat1_;
    status_[1] = cStatus_.stat2_;
    status_[2] = cStatus_.stat3_;
  }
  else if (type == pmacMessageBroker::PMAC_SLOW_READ) {
    // ask the command store to extract all Q variables from the SLOW_READ hash table
    sprintf(remove, "&%d", csNumber_);
    sprintf(key, "&%dQ", csNumber_);
    value = sPtr->getVariablesList(key, remove);
    setStringParam(PMAC_CS_QVariables_, value.c_str());
    callParamCallbacks();
  }
}

asynStatus pmacCSController::immediateWriteRead(const char *command, char *response) {
  static const char *functionName = "immediateWriteRead";
  asynStatus status = asynSuccess;

  if (!pC_) {
    debug(DEBUG_ERROR, functionName, "ERROR PMAC controller not found");
    status = asynError;
  }

  // Send the write/read demand to the PMAC controller
  if (status == asynSuccess) {
    ((pmacController *) pC_)->immediateWriteRead(command, response);
  }

  return status;
}

asynStatus pmacCSController::axisWriteRead(const char *command, char *response) {
  static const char *functionName = "axisWriteRead";
  asynStatus status = asynSuccess;

  if (!pC_) {
    debug(DEBUG_ERROR, functionName, "ERROR PMAC controller not found");
    status = asynError;
  }

  // Send the write/read demand to the PMAC controller
  if (status == asynSuccess) {
    ((pmacController *) pC_)->axisWriteRead(command, response);
  }

  return status;
}

pmacAxis *pmacCSController::getRawAxis(int axisNo)
{
  return pC_->getAxis(axisNo);
}

/** Returns a pointer to an pmacAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
pmacCSAxis *pmacCSController::getAxis(asynUser *pasynUser) {
  int axisNo = 0;

  getAddress(pasynUser, &axisNo);
  return getAxis(axisNo);
}

/** Returns a pointer to an pmacAxis object.
  * Returns NULL if the axis number is invalid.
  * \param[in] axisNo Axis index number. */
pmacCSAxis *pmacCSController::getAxis(int axisNo) {
  if ((axisNo < 0) || (axisNo >= numAxes_)) return NULL;
  return pAxes_[axisNo];
}

// Registration for callbacks
asynStatus pmacCSController::registerForCallbacks(pmacCallbackInterface *cbPtr, int type) {
  // Simply forward the request to the main controller
  return ((pmacController *) pC_)->registerForCallbacks(cbPtr, type);
}


asynStatus pmacCSController::monitorPMACVariable(int poll_speed, const char *var) {
  // Simply forward the request to the main controller
  return ((pmacController *) pC_)->monitorPMACVariable(poll_speed, var);
}

asynStatus pmacCSController::tScanCheckForErrors() {
  asynStatus status = asynSuccess;
  static const char *functionName = "tScanCheckForErrors";
  debug(DEBUG_FLOW, functionName);

  if ((status_[2] & CS_STATUS3_LIMIT) != 0) {
    status = asynError;
  }
  if ((status_[1] & CS_STATUS2_FOLLOW_ERR) != 0) {
    status = asynError;
  }
  if ((status_[1] & CS_STATUS2_AMP_FAULT) != 0) {
    status = asynError;
  }
  if ((status_[1] & CS_STATUS2_RUNTIME_ERR) != 0) {
    status = asynError;
  }
  return status;
}

/** Returns a string containing an appropriate trajectory scan error
  * message if the scan has failed or else an empty string.
  * Returns empty string if there is no error found.
  */
std::string pmacCSController::tScanGetErrorMessage()
{
  static const char *functionName = "tScanGetErrorMessage";
  std::stringstream ss;
  char reply[PMAC_MAXBUF];
  int reason;
  debug(DEBUG_FLOW, functionName);

  if ((status_[2] & CS_STATUS3_LIMIT) != 0) {
    std::stringstream err;
    err << "Trajectory scan failed - CS " << this->csNumber_ << " reports limit active";
    ss << "CS " << this->csNumber_ << " reports limit active";
    debug(DEBUG_ERROR, functionName, err.str());
  }
  if ((status_[1] & CS_STATUS2_FOLLOW_ERR) != 0) {
    std::stringstream err;
    err << "Trajectory scan failed - CS " << this->csNumber_ << " reports following error";
    ss << "CS " << this->csNumber_ << " reports following error";
    debug(DEBUG_ERROR, functionName, err.str());
  }
  if ((status_[1] & CS_STATUS2_AMP_FAULT) != 0) {
    std::stringstream err;
    err << "Trajectory scan failed - CS " << this->csNumber_ << " reports amplifier fault";
    ss << "CS " << this->csNumber_ << " reports amplifier fault";
    debug(DEBUG_ERROR, functionName, err.str());
  }
  if ((status_[1] & CS_STATUS2_RUNTIME_ERR) != 0) {
    std::stringstream err;
    err << "Trajectory scan failed - CS " << this->csNumber_ << " Runtime Error (";
    // Read the runtime error value out of the motion controller
    ss << "RY:$002" << (this->csNumber_ - 1) << "14";
    // Construct a full error message for logging to console
    // and a status message that can be displayed to the user
    err << ss.str();
    this->immediateWriteRead(ss.str().c_str(), reply);
    sscanf(reply, "%d", &reason);
    err << " = " << reason << ") : ";
    err << CS_RUNTIME_ERRORS[reason];
    debug(DEBUG_ERROR, functionName, err.str());
    ss.str(std::string());
    ss << "CS " << this->csNumber_ << " Runtime Error: " << CS_RUNTIME_ERRORS[reason];
  }
  return ss.str();
}

asynStatus pmacCSController::tScanCheckProgramRunning(int *running) {
  asynStatus status = asynSuccess;
  static const char *functionName = "tScanCheckProgramRunning";

  debug(DEBUG_FLOW, functionName);

  *running = cStatus_.running_;
  return status;
}

asynStatus pmacCSController::makeCSDemandsConsistent() {
  // Simply forward the request to the main controller
  return ((pmacController *) pC_)->makeCSDemandsConsistent();
}

/**
 * Set the PMAC axis scale factor to increase resolution in the motor record.
 * Default value is 1.
 * @param axis Axis number to set the PMAC axis scale factor.
 * @param scale Scale factor to set
 */
asynStatus pmacCSController::pmacSetAxisScale(int axis, int scale) {
  asynStatus result = asynSuccess;
  pmacCSAxis *pA = NULL;
  static const char *functionName = "pmacCSController::pmacSetAxisScale";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (scale < 1) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: Error: scale factor must be >=1.\n",
              functionName);
    result = asynError;
  } else {
    this->lock();
    pA = getAxis(axis);
    if (pA) {
      pA->scale_ = scale;
      setIntegerParam(axis, PMAC_CS_MotorScale_, scale);
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s. Setting scale factor of %d on axis %d, on controller %s.\n",
                functionName, pA->scale_, pA->axisNo_, portName);

    } else {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s: Error: axis %d has not been configured using pmacCreateAxis.\n", functionName,
                axis);
      result = asynError;
    }
    this->unlock();
  }
  return result;
}

asynStatus pmacCSController::wakeupPoller() {
  // We need to wake up the real motor controller polling task
  return ((pmacController *) pC_)->wakeupPoller();
}


/**
 * Set the real axis number that this virtual axis is directly mapped to
 * This should be zero if there is no direct mapping
 *
 * @param axis Axis number to set the PMAC axis scale factor.
 * @param mappedAxis the real axis this is mapped tp
 */
asynStatus pmacCSController::pmacCSSetAxisDirectMapping(int axis, int mappedAxis) {
  asynStatus status;
  static const char *functionName = "pmacCSSetAxisDirectMapping";

  debug(DEBUG_FLOW, functionName);

  this->lock();
  status = setIntegerParam(axis, PMAC_CS_RealMotorNumber_, mappedAxis);
  this->unlock();

  callParamCallbacks();
  return status;
}

int pmacCSController::pmacCSGetAxisDirectMapping(int axis) {
  int rawAxis = 0;
  static const char *functionName = "pmacCSGetAxisDirectMapping";

  debug(DEBUG_FLOW, functionName);
  getIntegerParam(axis, PMAC_CS_RealMotorNumber_, &rawAxis);

  return rawAxis;
}

double pmacCSController::getAxisResolution(int axis) {
  double resolution = 0;

  //getDoubleParam(axis, PMAC_CS_MotorRes_, &resolution);

  resolution = this->getAxis(axis)->getResolution();
  if(resolution == 0) {
    resolution = 1;  // guard against asyn issues causing div by zero
  }
  return resolution;
}

double pmacCSController::getAxisOffset(int axis) {
  //getDoubleParam(axis, PMAC_CS_MotorOffset_, &offset);

  double offset = this->getAxis(axis)->getOffset();
  return offset;
}

asynStatus pmacCSController::storeKinematics() {
  asynStatus status = asynSuccess;
  int buffer_size = 20000;
  char* buffer = (char*)malloc(buffer_size);
  static const char *functionName = "storeKinematics";

  startTimer(DEBUG_TIMING, functionName);
  debug(DEBUG_FLOW, functionName);
  // Read forward kinematic
  status = listKinematic(csNumber_ , "forward", buffer, buffer_size);
  // Store into the appropriate parameter
  setStringParam(PMAC_CS_ForwardKinematic_, buffer);
  // Read inverse kinematic
  status = listKinematic(csNumber_, "inverse", buffer, buffer_size);
  // Store into the appropriate parameter
  setStringParam(PMAC_CS_InverseKinematic_, buffer);

  callParamCallbacks();
  if (status != asynSuccess) {
    debug(DEBUG_ERROR, functionName, "Failed to read all Kinematics");
  }
  stopTimer(DEBUG_TIMING, functionName, "Time taken to store kinematics");

  free(buffer);
  return status;
}

asynStatus
pmacCSController::listKinematic(int csNo, const std::string &type, char *buffer, size_t size) {
  int word = 0;
  char reply[PMAC_MAXBUF];
  char cmd[PMAC_MAXBUF];
  char line[PMAC_MAXBUF];
  int cword = 0;
  int running = 1;
  asynStatus status = asynSuccess;
  static const char *functionName = "listKinematic";

  startTimer(DEBUG_TIMING, functionName);
  debug(DEBUG_FLOW, functionName);
  debug(DEBUG_VARIABLE, functionName, "Listing kinematics for CS", csNo);

  if (type != "forward" && type != "inverse") {
    status = asynError;
    debug(DEBUG_ERROR, functionName, "Unknown kinematic type", type);
  }

  if (status == asynSuccess) {
    // Setup the list command
    // List by 1 word at a time, throw away duplicates
    // This is very inefficient, but currently necessary due
    // to the PMAC driver
    strcpy(buffer, "");
    while (running == 1 && word < 10000) {
      sprintf(cmd, "&%d list %s,%d,1", csNo, type.c_str(), word);
      this->immediateWriteRead(cmd, reply);
      if (
          reply[0] == 0x7 || reply[0] == 0 ||
          strncmp(reply, " ", 5) == 0 ||
          strstr(reply, "NOT IN BUFFER") != NULL)
      {
        running = 0;
      } else {
        sscanf(reply, "%d:%s", &cword, line);
        if (cword == word) {
          if (strlen(buffer) + strlen(line) + 1 > size) {
            // We cannot add the next line as the buffer would be full
            // Report the error
            running = 0;
            status = asynError;
            debug(DEBUG_ERROR, functionName, "Buffer not large enough for kinematic", (int) size);
          } else {
            strcat(buffer, line);
            strcat(buffer, " ");
          }
        }
      }
      word++;
    }
    debug(DEBUG_VARIABLE, functionName, "Kinematic", buffer);
  }
  stopTimer(DEBUG_TIMING, functionName, "Time taken to list kinematic");

  return status;
}

/*************************************************************************************/
/** The following functions have C linkage, and can be called directly or from iocsh */

extern "C" {

/**
 * C wrapper for the pmacCSController constructor.
 * See pmacCSController::pmacCSController.
 *
 */
asynStatus pmacCreateCS(const char *portName,
                        const char *controllerPortName,
                        int csNo,
                        int program) {
  new pmacCSController(portName,
                       controllerPortName,
                       csNo,
                       program);

  return asynSuccess;
}

/**
 * C wrapper for the pmacCSAxis constructor.
 * See pmacCSAxis::pmacCSAxis.
 *
 */
asynStatus pmacCreateCSAxis(const char *pmacName, /* specify which controller by port name */
                            int axis)             /* axis number (start from 1). */
{
  pmacCSController *pC;

  static const char *functionName = "pmacCreateCSAxis";

  pC = (pmacCSController *) findAsynPortDriver(pmacName);
  if (!pC) {
    printf("%s: ERROR Port %s Not Found.\n", functionName, pmacName);
    return asynError;
  }

  if (axis == 0) {
    printf("%s: ERROR Axis Number 0 Not Allowed. This Asyn Address Is Reserved For Controller Specific Parameters.\n",
           functionName);
    return asynError;
  }

  pC->lock();
  new pmacCSAxis(pC, axis);
  pC->unlock();
  return asynSuccess;
}

/**
 * C Wrapper function for pmacAxis constructor.
 * See pmacCSAxis::pmacCSAxis.
 * This function allows creation of multiple pmacAxis objects with axis numbers 1 to numAxes.
 * @param pmacName Asyn port name for the controller (const char *)
 * @param numAxes The number of axes to create, starting at 1.
 *
 */
asynStatus pmacCreateCSAxes(const char *pmacName, /* specify which controller by port name */
                            int numAxes)          /* Number of axes to create */
{
  pmacCSController *pC;

  static const char *functionName = "pmacCreateCSAxis";

  pC = (pmacCSController *) findAsynPortDriver(pmacName);
  if (!pC) {
    printf("%s: Error port %s not found\n", functionName, pmacName);
    return asynError;
  }

  for (int axis = 0; axis <= numAxes; axis++) {
    new pmacCSAxis(pC, axis);
  }

  return asynSuccess;
}

/**
 * Set the PMAC axis scale factor to increase resolution in the motor record.
 * Default value is 10000.
 * @param controller The Asyn port name for the PMAC controller.
 * @param axis Axis number to set the PMAC axis scale factor.
 * @param scale Scale factor to set
 */
asynStatus pmacCSSetAxisScale(const char *pmacCSName, int axis, int scale) {
  pmacCSController *pC;
  static const char *functionName = "pmacSetCSAxisScale";

  pC = (pmacCSController *) findAsynPortDriver(pmacCSName);
  if (!pC) {
    printf("%s: Error port %s not found\n", functionName, pmacCSName);
    return asynError;
  }

  return pC->pmacSetAxisScale(axis, scale);
}

/* Code for iocsh registration */

/* pmacCreateController */
static const iocshArg pmacCreateCSControllerArg0 = {"Coordinate system port name", iocshArgString};
static const iocshArg pmacCreateCSControllerArg1 = {"PMAC controller port name", iocshArgString};
static const iocshArg pmacCreateCSControllerArg2 = {"Coordinate system number", iocshArgInt};
static const iocshArg pmacCreateCSControllerArg3 = {"Motion program execution number", iocshArgInt};
static const iocshArg *const pmacCreateCSControllerArgs[] = {&pmacCreateCSControllerArg0,
                                                             &pmacCreateCSControllerArg1,
                                                             &pmacCreateCSControllerArg2,
                                                             &pmacCreateCSControllerArg3};
static const iocshFuncDef configpmacCreateController = {"pmacCreateCS", 4,
                                                        pmacCreateCSControllerArgs};
static void configpmacCreateCSControllerCallFunc(const iocshArgBuf *args) {
  pmacCreateCS(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}

/* pmacCreateCSAxis */
static const iocshArg pmacCreateCSAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateCSAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg *const pmacCreateCSAxisArgs[] = {&pmacCreateCSAxisArg0,
                                                       &pmacCreateCSAxisArg1};
static const iocshFuncDef configpmacCSAxis = {"pmacCreateCSAxis", 2, pmacCreateCSAxisArgs};

static void configpmacCSAxisCallFunc(const iocshArgBuf *args) {
  pmacCreateCSAxis(args[0].sval, args[1].ival);
}

/* pmacCreateAxes */
static const iocshArg pmacCreateCSAxesArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateCSAxesArg1 = {"Num Axes", iocshArgInt};
static const iocshArg *const pmacCreateCSAxesArgs[] = {&pmacCreateCSAxesArg0,
                                                       &pmacCreateCSAxesArg1};
static const iocshFuncDef configpmacCSAxes = {"pmacCreateCSAxes", 2, pmacCreateCSAxesArgs};

static void configpmacCSAxesCallFunc(const iocshArgBuf *args) {
  pmacCreateCSAxes(args[0].sval, args[1].ival);
}

/* pmacCSSetAxisScale */
static const iocshArg pmacCSSetAxisScaleArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCSSetAxisScaleArg1 = {"Axis number", iocshArgInt};
static const iocshArg pmacCSSetAxisScaleArg2 = {"Scale factor", iocshArgInt};
static const iocshArg *const pmacCSSetAxisScaleArgs[] = {&pmacCSSetAxisScaleArg0,
                                                         &pmacCSSetAxisScaleArg1,
                                                         &pmacCSSetAxisScaleArg2};
static const iocshFuncDef configpmacCSSetAxisScale = {"pmacSetCoordStepsPerUnit", 3,
                                                      pmacCSSetAxisScaleArgs};

static void configpmacCSSetAxisScaleFunc(const iocshArgBuf *args) {
  pmacCSSetAxisScale(args[0].sval, args[1].ival, args[2].ival);
}

static void pmacCSControllerRegister(void) {
  iocshRegister(&configpmacCreateController, configpmacCreateCSControllerCallFunc);
  iocshRegister(&configpmacCSAxis, configpmacCSAxisCallFunc);
  iocshRegister(&configpmacCSAxes, configpmacCSAxesCallFunc);
  iocshRegister(&configpmacCSSetAxisScale, configpmacCSSetAxisScaleFunc);
}
epicsExportRegistrar(pmacCSControllerRegister);

#ifdef vxWorks
//VxWorks register functions
epicsRegisterFunction(pmacCreateCS);
epicsRegisterFunction(pmacCreateCSAxis);
epicsRegisterFunction(pmacCreateCSAxes);
epicsRegisterFunction(pmacCSSetAxisScale);
#endif
} // extern "C"



