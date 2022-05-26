/********************************************
 *  pmacController.cpp
 *
 *  PMAC Asyn motor based on the
 *  asynMotorController class.
 *
 *  Matthew Pearson
 *  23 May 2012
 *
 ********************************************/



#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <sstream>
#include <iostream>
#include <cmath>

using std::cout;
using std::endl;
using std::dec;

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <postfix.h>
#include <iocsh.h>
#include <drvSup.h>
#include <registryFunction.h>

#include "asynOctetSyncIO.h"

#include "pmacController.h"
#include "pmacCSController.h"
#include "pmacCSMonitor.h"

#include <epicsExport.h>

static const char *driverName = "pmacController";

const epicsUInt32 pmacController::PMAC_MAXBUF_ = PMAC_MAXBUF;
const epicsFloat64 pmacController::PMAC_TIMEOUT_ = 2.0;
const epicsUInt32 pmacController::PMAC_FEEDRATE_LIM_ = 100;
const epicsUInt32 pmacController::PMAC_ERROR_PRINT_TIME_ = 600; //seconds
const epicsUInt32 pmacController::PMAC_FORCED_FAST_POLLS_ = 10;
const epicsUInt32 pmacController::PMAC_OK_ = 0;
const epicsUInt32 pmacController::PMAC_ERROR_ = 1;
const epicsUInt32 pmacController::PMAC_FEEDRATE_DEADBAND_ = 1;
const epicsInt32 pmacController::PMAC_CID_PMAC_ = 602413;
const epicsInt32 pmacController::PMAC_CID_GEOBRICK_ = 603382;
const epicsInt32 pmacController::PMAC_CID_CLIPPER_ = 602404;
const epicsInt32 pmacController::PMAC_CID_POWER_ = 604020;

const epicsUInt32 pmacController::PMAC_STATUS1_MAXRAPID_SPEED = (0x1 << 0);
const epicsUInt32 pmacController::PMAC_STATUS1_ALT_CMNDOUT_MODE = (0x1 << 1);
const epicsUInt32 pmacController::PMAC_STATUS1_SOFT_POS_CAPTURE = (0x1 << 2);
const epicsUInt32 pmacController::PMAC_STATUS1_ERROR_TRIGGER = (0x1 << 3);
const epicsUInt32 pmacController::PMAC_STATUS1_FOLLOW_ENABLE = (0x1 << 4);
const epicsUInt32 pmacController::PMAC_STATUS1_FOLLOW_OFFSET = (0x1 << 5);
const epicsUInt32 pmacController::PMAC_STATUS1_PHASED_MOTOR = (0x1 << 6);
const epicsUInt32 pmacController::PMAC_STATUS1_ALT_SRC_DEST = (0x1 << 7);
const epicsUInt32 pmacController::PMAC_STATUS1_USER_SERVO = (0x1 << 8);
const epicsUInt32 pmacController::PMAC_STATUS1_USER_PHASE = (0x1 << 9);
const epicsUInt32 pmacController::PMAC_STATUS1_HOMING = (0x1 << 10);
const epicsUInt32 pmacController::PMAC_STATUS1_BLOCK_REQUEST = (0x1 << 11);
const epicsUInt32 pmacController::PMAC_STATUS1_DECEL_ABORT = (0x1 << 12);
const epicsUInt32 pmacController::PMAC_STATUS1_DESIRED_VELOCITY_ZERO = (0x1 << 13);
const epicsUInt32 pmacController::PMAC_STATUS1_DATABLKERR = (0x1 << 14);
const epicsUInt32 pmacController::PMAC_STATUS1_DWELL = (0x1 << 15);
const epicsUInt32 pmacController::PMAC_STATUS1_INTEGRATE_MODE = (0x1 << 16);
const epicsUInt32 pmacController::PMAC_STATUS1_MOVE_TIME_ON = (0x1 << 17);
const epicsUInt32 pmacController::PMAC_STATUS1_OPEN_LOOP = (0x1 << 18);
const epicsUInt32 pmacController::PMAC_STATUS1_AMP_ENABLED = (0x1 << 19);
const epicsUInt32 pmacController::PMAC_STATUS1_X_SERVO_ON = (0x1 << 20);
const epicsUInt32 pmacController::PMAC_STATUS1_POS_LIMIT_SET = (0x1 << 21);
const epicsUInt32 pmacController::PMAC_STATUS1_NEG_LIMIT_SET = (0x1 << 22);
const epicsUInt32 pmacController::PMAC_STATUS1_MOTOR_ON = (0x1 << 23);

const epicsUInt32 pmacController::PMAC_STATUS2_IN_POSITION = (0x1 << 0);
const epicsUInt32 pmacController::PMAC_STATUS2_WARN_FOLLOW_ERR = (0x1 << 1);
const epicsUInt32 pmacController::PMAC_STATUS2_ERR_FOLLOW_ERR = (0x1 << 2);
const epicsUInt32 pmacController::PMAC_STATUS2_AMP_FAULT = (0x1 << 3);
const epicsUInt32 pmacController::PMAC_STATUS2_NEG_BACKLASH = (0x1 << 4);
const epicsUInt32 pmacController::PMAC_STATUS2_I2T_AMP_FAULT = (0x1 << 5);
const epicsUInt32 pmacController::PMAC_STATUS2_I2_FOLLOW_ERR = (0x1 << 6);
const epicsUInt32 pmacController::PMAC_STATUS2_TRIGGER_MOVE = (0x1 << 7);
const epicsUInt32 pmacController::PMAC_STATUS2_PHASE_REF_ERR = (0x1 << 8);
const epicsUInt32 pmacController::PMAC_STATUS2_PHASE_SEARCH = (0x1 << 9);
const epicsUInt32 pmacController::PMAC_STATUS2_HOME_COMPLETE = (0x1 << 10);
const epicsUInt32 pmacController::PMAC_STATUS2_POS_LIMIT_STOP = (0x1 << 11);
const epicsUInt32 pmacController::PMAC_STATUS2_DESIRED_STOP = (0x1 << 12);
const epicsUInt32 pmacController::PMAC_STATUS2_FORE_IN_POS = (0x1 << 13);
const epicsUInt32 pmacController::PMAC_STATUS2_NA14 = (0x1 << 14);
const epicsUInt32 pmacController::PMAC_STATUS2_ASSIGNED_CS = (0x1 << 15);

/*Global status ???*/
const epicsUInt32 pmacController::PMAC_GSTATUS_CARD_ADDR = (0x1 << 0);
const epicsUInt32 pmacController::PMAC_GSTATUS_ALL_CARD_ADDR = (0x1 << 1);
const epicsUInt32 pmacController::PMAC_GSTATUS_RESERVED = (0x1 << 2);
const epicsUInt32 pmacController::PMAC_GSTATUS_PHASE_CLK_MISS = (0x1 << 3);
const epicsUInt32 pmacController::PMAC_GSTATUS_MACRO_RING_ERRORCHECK = (0x1 << 4);
const epicsUInt32 pmacController::PMAC_GSTATUS_MACRO_RING_COMMS = (0x1 << 5);
const epicsUInt32 pmacController::PMAC_GSTATUS_TWS_PARITY_ERROR = (0x1 << 6);
const epicsUInt32 pmacController::PMAC_GSTATUS_CONFIG_ERROR = (0x1 << 7);
const epicsUInt32 pmacController::PMAC_GSTATUS_ILLEGAL_LVAR = (0x1 << 8);
const epicsUInt32 pmacController::PMAC_GSTATUS_REALTIME_INTR = (0x1 << 9);
const epicsUInt32 pmacController::PMAC_GSTATUS_FLASH_ERROR = (0x1 << 10);
const epicsUInt32 pmacController::PMAC_GSTATUS_DPRAM_ERROR = (0x1 << 11);
const epicsUInt32 pmacController::PMAC_GSTATUS_CKSUM_ACTIVE = (0x1 << 12);
const epicsUInt32 pmacController::PMAC_GSTATUS_CKSUM_ERROR = (0x1 << 13);
const epicsUInt32 pmacController::PMAC_GSTATUS_LEADSCREW_COMP = (0x1 << 14);
const epicsUInt32 pmacController::PMAC_GSTATUS_WATCHDOG = (0x1 << 15);
const epicsUInt32 pmacController::PMAC_GSTATUS_SERVO_REQ = (0x1 << 16);
const epicsUInt32 pmacController::PMAC_GSTATUS_DATA_GATHER_START = (0x1 << 17);
const epicsUInt32 pmacController::PMAC_GSTATUS_RESERVED2 = (0x1 << 18);
const epicsUInt32 pmacController::PMAC_GSTATUS_DATA_GATHER_ON = (0x1 << 19);
const epicsUInt32 pmacController::PMAC_GSTATUS_SERVO_ERROR = (0x1 << 20);
const epicsUInt32 pmacController::PMAC_GSTATUS_CPUTYPE = (0x1 << 21);
const epicsUInt32 pmacController::PMAC_GSTATUS_REALTIME_INTR_RE = (0x1 << 22);
const epicsUInt32 pmacController::PMAC_GSTATUS_RESERVED3 = (0x1 << 23);

const epicsUInt32 pmacController::PMAC_HARDWARE_PROB = (PMAC_GSTATUS_REALTIME_INTR |
                                                        PMAC_GSTATUS_FLASH_ERROR |
                                                        PMAC_GSTATUS_DPRAM_ERROR |
                                                        PMAC_GSTATUS_CKSUM_ERROR |
                                                        PMAC_GSTATUS_WATCHDOG |
                                                        PMAC_GSTATUS_SERVO_ERROR);

const epicsUInt32 pmacController::PMAX_AXIS_GENERAL_PROB1 = 0;
const epicsUInt32 pmacController::PMAX_AXIS_GENERAL_PROB2 = (PMAC_STATUS2_DESIRED_STOP |
                                                             PMAC_STATUS2_AMP_FAULT);

//C function prototypes, for the functions that can be called on IOC shell.
//Some of these functions are provided to ease transition to the model 3 driver. Some of these
//functions could be handled by the parameter library.
extern "C"
{
asynStatus
pmacCreateController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress,
                     int numAxes, int movingPollPeriod, int idlePollPeriod);
asynStatus pmacCreateAxis(const char *pmacName, int axis);
asynStatus pmacCreateAxes(const char *pmacName, int numAxes);
asynStatus pmacDisableLimitsCheck(const char *controller, int axis, int allAxes);
asynStatus pmacSetAxisScale(const char *controller, int axis, int scale);
asynStatus pmacSetOpenLoopEncoderAxis(const char *controller, int axis, int encoder_axis);
}

static void trajTaskC(void *drvPvt) {
  pmacController *pPvt = (pmacController *) drvPvt;
  pPvt->trajectoryTask();
}

/**
 * pmacController constructor.
 * @param portName The Asyn port name to use (that the motor record connects to).
 * @param lowLevelPortName The name of the low level port that has already been created, to enable comms to the controller.
 * @param lowLevelPortAddress The asyn address for the low level port
 * @param numAxes The number of axes on the controller (1 based)
 * @param movingPollPeriod The time (in milliseconds) between polling when axes are moving
 * @param movingPollPeriod The time (in milliseconds) between polling when axes are idle
 */
pmacController::pmacController(const char *portName, const char *lowLevelPortName,
                               int lowLevelPortAddress,
                               int numAxes, double movingPollPeriod, double idlePollPeriod)
        : asynMotorController(portName, numAxes + 1, NUM_MOTOR_DRIVER_PARAMS + NUM_PMAC_PARAMS,
                              asynEnumMask | asynInt32ArrayMask, // For user mode and velocity mode
                              asynEnumMask, // No addition interrupt interfaces
                              ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                              1, // autoconnect
                              0, 50000),  // Default priority and stack size
          pmacDebugger("pmacController") {
  int index = 0;
  static const char *functionName = "pmacController::pmacController";

  useCsVelocity = true;
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Constructor.\n", functionName);

  //Initialize non static data members
  pHardware_ = NULL;
  connected_ = 0;
  initialised_ = 0;
  cid_ = 0;
  cpu_ = "";
  parameterIndex_ = 0;
  lowLevelPortUser_ = NULL;
  movesDeferred_ = 0;
  nowTimeSecs_ = 0.0;
  lastTimeSecs_ = 0.0;
  epicsTimeGetCurrent(&lastMediumTime_);
  epicsTimeAddSeconds(&lastMediumTime_, PMAC_MEDIUM_LOOP_TIME / -1000.0);
  epicsTimeGetCurrent(&lastSlowTime_);
  epicsTimeAddSeconds(&lastSlowTime_, PMAC_SLOW_LOOP_TIME / -1000.0);
  printNextError_ = false;
  feedRatePoll_ = false;
  movingPollPeriod_ = movingPollPeriod;
  idlePollPeriod_ = idlePollPeriod;
  pvtTimeMode_ = 0;
  profileInitialized_ = false;
  profileBuilt_ = false;
  appendAvailable_ = false;
  tScanShortScan_ = false;
  tScanExecuting_ = 0;
  tScanCSNo_ = 0;
  tScanAxisMask_ = 0;
  tScanPointCtr_ = 0;
  tScanPmacBufferPtr_ = 0;
  tScanPmacTotalPts_ = 0;
  tScanPmacStatus_ = 0;
  tScanPmacBufferNumber_ = 0;
  tScanPmacBufferAddressA_ = 0;
  tScanPmacBufferAddressB_ = 0;
  tScanPmacBufferSize_ = 0;
  tScanPositions_ = NULL;
  tScanPmacProgVersion_ = 0.0;
  i8_ = 0;
  i7002_ = 0;
  csResetAllDemands = false;
  csCount = 0;

  // Create the message broker
  pBroker_ = new pmacMessageBroker(this->pasynUserSelf);

  // Create the hashtable for storing port to CS number mappings
  pPortToCs_ = new IntegerHashtable();

  // Create the parameter hashtables
  pIntParams_ = new IntegerHashtable();
  pHexParams_ = new IntegerHashtable();
  pDoubleParams_ = new IntegerHashtable();
  pStringParams_ = new IntegerHashtable();
  pWriteParams_ = new StringHashtable();

  pAxes_ = (pmacAxis **) (asynMotorController::pAxes_);

  // Initialise the table of CS controller pointers
  pCSControllers_ = (pmacCSController **) malloc(PMAC_MAX_CS * sizeof(pmacCSController *));
  for (index = 0; index < PMAC_MAX_CS; index++) {
    pCSControllers_[index] = NULL;
  }

  //Create dummy axis for asyn address 0. This is used for controller parameters.
  //pAxisZero = new pmacAxis(this, 0);
  pAxisZero = new pmacCSMonitor(this);
  pGroupList = new pmacCsGroups(this);


  // Create the trajectory store
  pTrajectory_ = new pmacTrajectory();

  createAsynParams();

  if (pBroker_->connect(lowLevelPortName, lowLevelPortAddress) != asynSuccess) {
    printf("%s: Failed to connect to low level asynOctetSyncIO port %s\n", functionName,
           lowLevelPortName);
    setIntegerParam(PMAC_C_CommsError_, PMAC_ERROR_);
  } else {
    setIntegerParam(PMAC_C_CommsError_, PMAC_OK_);
  }

  // Initialise the connection
  this->checkConnection();
  if (!connected_){
    debugf(DEBUG_ERROR, "pmacController", "FAILED TO CONNECT TO CONTROLLER '%s'", portName);
  }

  // Do nothing if we have failed to connect. Requires a restart once
  // the brick is restored
  initAsynParams();

  // Create the epicsEvents for signaling to start and stop scanning
  this->startEventId_ = epicsEventCreate(epicsEventEmpty);
  if (!this->startEventId_) {
    printf("%s:%s epicsEventCreate failure for start event\n", driverName, functionName);
  }
  this->stopEventId_ = epicsEventCreate(epicsEventEmpty);
  if (!this->stopEventId_) {
    printf("%s:%s epicsEventCreate failure for stop event\n", driverName, functionName);
  }

  // Create the thread that executes trajectory scans
  epicsThreadCreate("TrajScanTask",
                    epicsThreadPriorityMedium,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC) trajTaskC,
                    this);
}

pmacController::~pmacController(void) {
  //Destructor. Should never get here.
  delete pAxisZero;
}

bool pmacController::initialised() {
  return initialised_;
}

asynStatus pmacController::checkConnection() {
  asynStatus status = asynSuccess;
  int connected, newConnection;
  static const char *functionName = "checkConnection";
  debug(DEBUG_FLOW, functionName);

  if (pBroker_ != NULL) {
    status = pBroker_->getConnectedStatus(&connected, &newConnection);
  } else {
    status = asynError;
  }

  if (status == asynSuccess) {
    connected_ = connected;  //must be before initialSetup() call
    if (connected && newConnection)  initialSetup();  //config new gpascii session
    debug(DEBUG_VARIABLE, functionName, "Connection status", connected_);
  } else {
    connected_ = false;
    // Inform all motor axis objects that the connection is dropped
    for (int axis = 1; axis <= numAxes_; axis++) {
      if (this->getAxis(axis) != NULL) {
        this->getAxis(axis)->badConnection();
      }
    }
    // Inform all CS controller objects that the connection is dropped
    for (int index = 0; index < PMAC_MAX_CS; index++) {
      pmacCSController *csPtr = pCSControllers_[index];
      if (csPtr != NULL){
        csPtr->badConnection();
      }
    }
  }

  return status;
}

asynStatus pmacController::initialSetup() {
  asynStatus status;
  char response[1024];
  static const char *functionName = "initialiseConnection";
  debug(DEBUG_FLOW, functionName);
  status = this->readDeviceType();

  if (status == asynSuccess) {
    if(pHardware_ != NULL) {
      delete pHardware_;
    }

    // Check for powerPMAC connection
    if (cid_ == PMAC_CID_POWER_) {
      pHardware_ = new pmacHardwarePower();
      // Mark the connection
      pBroker_->markAsPowerPMAC();
      // set the echo to 7
      this->lowLevelWriteRead("echo 7", response);
    } else if (cid_ == PMAC_CID_GEOBRICK_ || cid_ == PMAC_CID_PMAC_ ||
               cid_ == PMAC_CID_CLIPPER_) {
      pHardware_ = new pmacHardwareTurbo();
    } else {
      // This is bad so output an error and return error status
      debug(DEBUG_ERROR, functionName, "Unknown hardware CID:", cid_);
      status = asynError;
    }
    if (status == asynSuccess) {
      // Register this controller with the hardware class
      pHardware_->registerController(this);
    }
  }

  if (status != asynSuccess) {
    debug(DEBUG_ERROR, functionName, "Unable to initialise connection to PMAC!");
  } else {
    // Initialisation successful
    initialised_ = 1;
    setupBrokerVariables();
  }

  if (status == asynSuccess)  pBroker_->clearNewConnection();

  // Inform all motor axis objects that the connection is good
  for (int axis = 1; axis <= numAxes_; axis++) {
    if (this->getAxis(axis) != NULL) {
      this->getAxis(axis)->goodConnection();
    }
  }

  // Now loop over and initialise all CS registered objects
  for (int index = 0; index < PMAC_MAX_CS; index++) {
    this->initCSHardware(index);
  }

  return status;
}

void pmacController::createAsynParams(void) {
  //Create controller-specific parameters
  createParam(PMAC_C_FirstParamString, asynParamInt32, &PMAC_C_FirstParam_);
  createParam(PMAC_C_PollAllNowString, asynParamInt32, &PMAC_C_PollAllNow_);
  createParam(PMAC_C_StopAllString, asynParamInt32, &PMAC_C_StopAll_);
  createParam(PMAC_C_KillAllString, asynParamInt32, &PMAC_C_KillAll_);
  createParam(PMAC_C_GlobalStatusString, asynParamInt32, &PMAC_C_GlobalStatus_);
  createParam(PMAC_C_CommsErrorString, asynParamInt32, &PMAC_C_CommsError_);
  createParam(PMAC_C_FeedRateString, asynParamInt32, &PMAC_C_FeedRate_);
  createParam(PMAC_C_FeedRateLimitString, asynParamInt32, &PMAC_C_FeedRateLimit_);
  createParam(PMAC_C_FeedRatePollString, asynParamInt32, &PMAC_C_FeedRatePoll_);
  createParam(PMAC_C_FeedRateProblemString, asynParamInt32, &PMAC_C_FeedRateProblem_);
  createParam(PMAC_C_FeedRateCSString, asynParamInt32, &PMAC_C_FeedRateCS_);
  createParam(PMAC_C_CoordSysGroup, asynParamInt32, &PMAC_C_CoordSysGroup_);
  createParam(PMAC_C_GroupCSPortString, asynParamInt32, &PMAC_C_GroupCSPort_);
  createParam(PMAC_C_GroupCSPortRBVString, asynParamInt32, &PMAC_C_GroupCSPortRBV_);
  createParam(PMAC_C_GroupAssignString, asynParamOctet, &PMAC_C_GroupAssign_);
  createParam(PMAC_C_GroupAssignRBVString, asynParamOctet, &PMAC_C_GroupAssignRBV_);
  createParam(PMAC_C_GroupExecuteString, asynParamInt32, &PMAC_C_GroupExecute_);
  createParam(PMAC_C_DebugLevelString, asynParamInt32, &PMAC_C_DebugLevel_);
  createParam(PMAC_C_DebugAxisString, asynParamInt32, &PMAC_C_DebugAxis_);
  createParam(PMAC_C_DebugCSString, asynParamInt32, &PMAC_C_DebugCS_);
  createParam(PMAC_C_DebugCmdString, asynParamInt32, &PMAC_C_DebugCmd_);
  createParam(PMAC_C_DisablePollingString, asynParamInt32, &PMAC_C_DisablePolling_);
  createParam(PMAC_C_FastUpdateTimeString, asynParamFloat64, &PMAC_C_FastUpdateTime_);
  createParam(PMAC_C_LastParamString, asynParamInt32, &PMAC_C_LastParam_);
  createParam(PMAC_C_CpuUsageString, asynParamFloat64, &PMAC_C_CpuUsage_);
  createParam(PMAC_C_AxisCSString, asynParamInt32, &PMAC_C_AxisCS_);
  createParam(PMAC_C_AxisReadonlyString, asynParamInt32, &PMAC_C_AxisReadonly_);
  createParam(PMAC_C_WriteCmdString, asynParamOctet, &PMAC_C_WriteCmd_);
  createParam(PMAC_C_KillAxisString, asynParamInt32, &PMAC_C_KillAxis_);
  createParam(PMAC_C_PLCBits00String, asynParamInt32, &PMAC_C_PLCBits00_);
  createParam(PMAC_C_PLCBits01String, asynParamInt32, &PMAC_C_PLCBits01_);
  createParam(PMAC_C_StatusBits01String, asynParamInt32, &PMAC_C_StatusBits01_);
  createParam(PMAC_C_StatusBits02String, asynParamInt32, &PMAC_C_StatusBits02_);
  createParam(PMAC_C_StatusBits03String, asynParamInt32, &PMAC_C_StatusBits03_);
  createParam(PMAC_C_GpioInputsString, asynParamInt32, &PMAC_C_GpioInputs_);
  createParam(PMAC_C_GpioOutputsString, asynParamInt32, &PMAC_C_GpioOutputs_);
  createParam(PMAC_C_ProgBitsString, asynParamInt32, &PMAC_C_ProgBits_);
  createParam(PMAC_C_AxisBits01String, asynParamInt32, &PMAC_C_AxisBits01_);
  createParam(PMAC_C_AxisBits02String, asynParamInt32, &PMAC_C_AxisBits02_);
  createParam(PMAC_C_AxisBits03String, asynParamInt32, &PMAC_C_AxisBits03_);
  createParam(PMAC_C_ProfileUseAxisAString, asynParamInt32, &PMAC_C_ProfileUseAxisA_);
  createParam(PMAC_C_ProfileUseAxisBString, asynParamInt32, &PMAC_C_ProfileUseAxisB_);
  createParam(PMAC_C_ProfileUseAxisCString, asynParamInt32, &PMAC_C_ProfileUseAxisC_);
  createParam(PMAC_C_ProfileUseAxisUString, asynParamInt32, &PMAC_C_ProfileUseAxisU_);
  createParam(PMAC_C_ProfileUseAxisVString, asynParamInt32, &PMAC_C_ProfileUseAxisV_);
  createParam(PMAC_C_ProfileUseAxisWString, asynParamInt32, &PMAC_C_ProfileUseAxisW_);
  createParam(PMAC_C_ProfileUseAxisXString, asynParamInt32, &PMAC_C_ProfileUseAxisX_);
  createParam(PMAC_C_ProfileUseAxisYString, asynParamInt32, &PMAC_C_ProfileUseAxisY_);
  createParam(PMAC_C_ProfileUseAxisZString, asynParamInt32, &PMAC_C_ProfileUseAxisZ_);
  createParam(PMAC_C_ProfilePositionsAString, asynParamFloat64Array, &PMAC_C_ProfilePositionsA_);
  createParam(PMAC_C_ProfilePositionsBString, asynParamFloat64Array, &PMAC_C_ProfilePositionsB_);
  createParam(PMAC_C_ProfilePositionsCString, asynParamFloat64Array, &PMAC_C_ProfilePositionsC_);
  createParam(PMAC_C_ProfilePositionsUString, asynParamFloat64Array, &PMAC_C_ProfilePositionsU_);
  createParam(PMAC_C_ProfilePositionsVString, asynParamFloat64Array, &PMAC_C_ProfilePositionsV_);
  createParam(PMAC_C_ProfilePositionsWString, asynParamFloat64Array, &PMAC_C_ProfilePositionsW_);
  createParam(PMAC_C_ProfilePositionsXString, asynParamFloat64Array, &PMAC_C_ProfilePositionsX_);
  createParam(PMAC_C_ProfilePositionsYString, asynParamFloat64Array, &PMAC_C_ProfilePositionsY_);
  createParam(PMAC_C_ProfilePositionsZString, asynParamFloat64Array, &PMAC_C_ProfilePositionsZ_);
  createParam(PMAC_C_ProfileAppendString, asynParamInt32, &PMAC_C_ProfileAppend_);
  createParam(PMAC_C_ProfileAppendStateString, asynParamInt32, &PMAC_C_ProfileAppendState_);
  createParam(PMAC_C_ProfileAppendStatusString, asynParamInt32, &PMAC_C_ProfileAppendStatus_);
  createParam(PMAC_C_ProfileAppendMessageString, asynParamOctet, &PMAC_C_ProfileAppendMessage_);
  createParam(PMAC_C_ProfileNumBuildString, asynParamInt32, &PMAC_C_ProfileNumBuild_);
  createParam(PMAC_C_ProfileBuiltPointsString, asynParamInt32, &PMAC_C_ProfileBuiltPoints_);
  createParam(PMAC_C_ProfileUserString, asynParamInt32Array, &PMAC_C_ProfileUser_);
  createParam(PMAC_C_ProfileVelModeString, asynParamInt32Array, &PMAC_C_ProfileVelMode_);
  createParam(PMAC_C_TrajBufferLengthString, asynParamInt32, &PMAC_C_TrajBufferLength_);
  createParam(PMAC_C_TrajTotalPointsString, asynParamInt32, &PMAC_C_TrajTotalPoints_);
  createParam(PMAC_C_TrajStatusString, asynParamInt32, &PMAC_C_TrajStatus_);
  createParam(PMAC_C_TrajCurrentIndexString, asynParamInt32, &PMAC_C_TrajCurrentIndex_);
  createParam(PMAC_C_TrajCurrentBufferString, asynParamInt32, &PMAC_C_TrajCurrentBuffer_);
  createParam(PMAC_C_TrajBuffAdrAString, asynParamInt32, &PMAC_C_TrajBuffAdrA_);
  createParam(PMAC_C_TrajBuffAdrBString, asynParamInt32, &PMAC_C_TrajBuffAdrB_);
  createParam(PMAC_C_TrajBuffFillAString, asynParamInt32, &PMAC_C_TrajBuffFillA_);
  createParam(PMAC_C_TrajBuffFillBString, asynParamInt32, &PMAC_C_TrajBuffFillB_);
  createParam(PMAC_C_TrajRunTimeString, asynParamFloat64, &PMAC_C_TrajRunTime_);
  createParam(PMAC_C_TrajCSNumberString, asynParamInt32, &PMAC_C_TrajCSNumber_);
  createParam(PMAC_C_TrajCSPortString, asynParamInt32, &PMAC_C_TrajCSPort_);
  createParam(PMAC_C_TrajPercentString, asynParamFloat64, &PMAC_C_TrajPercent_);
  createParam(PMAC_C_TrajEStatusString, asynParamInt32, &PMAC_C_TrajEStatus_);
  createParam(PMAC_C_TrajProgString, asynParamInt32, &PMAC_C_TrajProg_);
  createParam(PMAC_C_TrajProgVersionString, asynParamFloat64, &PMAC_C_TrajProgVersion_);
  createParam(PMAC_C_TrajCodeVersionString, asynParamFloat64, &PMAC_C_TrajCodeVersion_);
  createParam(PMAC_C_NoOfMsgsString, asynParamInt32, &PMAC_C_NoOfMsgs_);
  createParam(PMAC_C_TotalBytesWrittenString, asynParamInt32, &PMAC_C_TotalBytesWritten_);
  createParam(PMAC_C_TotalBytesReadString, asynParamInt32, &PMAC_C_TotalBytesRead_);
  createParam(PMAC_C_MsgBytesWrittenString, asynParamInt32, &PMAC_C_MsgBytesWritten_);
  createParam(PMAC_C_MsgBytesReadString, asynParamInt32, &PMAC_C_MsgBytesRead_);
  createParam(PMAC_C_MsgTimeString, asynParamInt32, &PMAC_C_MsgTime_);
  createParam(PMAC_C_MaxBytesWrittenString, asynParamInt32, &PMAC_C_MaxBytesWritten_);
  createParam(PMAC_C_MaxBytesReadString, asynParamInt32, &PMAC_C_MaxBytesRead_);
  createParam(PMAC_C_MaxTimeString, asynParamInt32, &PMAC_C_MaxTime_);
  createParam(PMAC_C_AveBytesWrittenString, asynParamInt32, &PMAC_C_AveBytesWritten_);
  createParam(PMAC_C_AveBytesReadString, asynParamInt32, &PMAC_C_AveBytesRead_);
  createParam(PMAC_C_AveTimeString, asynParamInt32, &PMAC_C_AveTime_);
  createParam(PMAC_C_FastStoreString, asynParamInt32, &PMAC_C_FastStore_);
  createParam(PMAC_C_MediumStoreString, asynParamInt32, &PMAC_C_MediumStore_);
  createParam(PMAC_C_SlowStoreString, asynParamInt32, &PMAC_C_SlowStore_);
  createParam(PMAC_C_ReportFastString, asynParamInt32, &PMAC_C_ReportFast_);
  createParam(PMAC_C_ReportMediumString, asynParamInt32, &PMAC_C_ReportMedium_);
  createParam(PMAC_C_ReportSlowString, asynParamInt32, &PMAC_C_ReportSlow_);
  createParam(PMAC_C_HomingStatusString, asynParamInt32, &PMAC_C_HomingStatus_);
  createParam(PMAC_C_RealMotorNumberString, asynParamInt32, &PMAC_C_RealMotorNumber_);
  createParam(PMAC_C_MotorScaleString, asynParamInt32, &PMAC_C_MotorScale_);
  createParam(PMAC_C_MotorResString, asynParamFloat64, &PMAC_C_MotorRes_);
  createParam(PMAC_C_MotorOffsetString, asynParamFloat64, &PMAC_C_MotorOffset_);
  createParam(PMAC_C_DirectMoveString, asynParamFloat64, &PMAC_C_DirectMove_);
  createParam(PMAC_C_DirectResString, asynParamFloat64, &PMAC_C_DirectRes_);
  createParam(PMAC_C_DirectOffsetString, asynParamFloat64, &PMAC_C_DirectOffset_);
  createParam(PMAC_C_IVariablesString, asynParamOctet, &PMAC_I_Variables_);
  createParam(PMAC_C_MVariablesString, asynParamOctet, &PMAC_M_Variables_);
  createParam(PMAC_C_PVariablesString, asynParamOctet, &PMAC_P_Variables_);
}

void pmacController::initAsynParams(void) {
  const char *functionName = "initAsynParams";

  bool paramStatus = true;
  paramStatus = ((setIntegerParam(PMAC_C_StopAll_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_KillAll_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_PollAllNow_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_GlobalStatus_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_FeedRateProblem_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_FeedRateCS_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_FeedRateLimit_, 100) == asynSuccess) && paramStatus);
  paramStatus = ((setDoubleParam(PMAC_C_FastUpdateTime_, 0.0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_AxisReadonly_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_CoordSysGroup_, 0) == asynSuccess) && paramStatus);
  for (int axis = 0; axis < numAxes_; axis++) {
    paramStatus = ((setIntegerParam(axis, PMAC_C_GroupCSPort_, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(axis, PMAC_C_GroupCSPortRBV_, 0) == asynSuccess) &&
                   paramStatus);
    paramStatus = ((setStringParam(axis, PMAC_C_GroupAssign_, "") == asynSuccess) && paramStatus);
    paramStatus = ((setStringParam(axis, PMAC_C_GroupAssignRBV_, "") == asynSuccess) &&
                   paramStatus);
  }
  // Initialise the trajectory interface
  paramStatus = ((setIntegerParam(profileBuildState_, PROFILE_BUILD_DONE) == asynSuccess) &&
                 paramStatus);
  paramStatus = ((setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_DONE) == asynSuccess) &&
                 paramStatus);
  paramStatus = (
          (setIntegerParam(PMAC_C_ProfileAppendState_, PROFILE_EXECUTE_DONE) == asynSuccess) &&
          paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_ProfileNumBuild_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_ProfileBuiltPoints_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setDoubleParam(PMAC_C_TrajRunTime_, 0.0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_TrajCSNumber_, tScanCSNo_) == asynSuccess) && paramStatus);
  //paramStatus = ((setStringParam(PMAC_C_TrajCSPort_, "") == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_TrajEStatus_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_TrajProg_, 1) == asynSuccess) && paramStatus);
  paramStatus = ((setDoubleParam(PMAC_C_TrajProgVersion_, 0.0) == asynSuccess) && paramStatus);
  paramStatus = (
          (setDoubleParam(PMAC_C_TrajCodeVersion_, PMAC_TRAJECTORY_VERSION) == asynSuccess) &&
          paramStatus);
  // Initialise the statistics
  paramStatus = ((setIntegerParam(PMAC_C_NoOfMsgs_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_TotalBytesWritten_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_TotalBytesRead_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_MsgBytesWritten_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_MsgBytesRead_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_MsgTime_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_MaxBytesWritten_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_MaxBytesRead_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_MaxTime_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_AveBytesWritten_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_AveBytesRead_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_AveTime_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_FastStore_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_MediumStore_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_SlowStore_, 0) == asynSuccess) && paramStatus);

  // Set individual axes parmeters
  for (int index = 0; index < numAxes_; index++) {
    paramStatus = ((setIntegerParam(index, PMAC_C_RealMotorNumber_, index) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(index, PMAC_C_MotorScale_, 1) == asynSuccess) && paramStatus);
    //paramStatus = ((setDoubleParam(index, PMAC_C_MotorRes_, 1.0) == asynSuccess) && paramStatus);
    //paramStatus = ((setDoubleParam(index, PMAC_C_MotorOffset_, 0.0) == asynSuccess) && paramStatus);
  }
  callParamCallbacks();

  if (!paramStatus) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s Unable To Set Driver Parameters In Constructor.\n", functionName);
  }
}

void pmacController::pollAllNow(void) {
  static const char *functionName = "pollAllNow";

  debug(DEBUG_FLOW, functionName);
  // Force updates of each loop
  pBroker_->updateVariables(pmacMessageBroker::PMAC_PRE_FAST_READ);
  pBroker_->updateVariables(pmacMessageBroker::PMAC_FAST_READ);
  pBroker_->updateVariables(pmacMessageBroker::PMAC_MEDIUM_READ);
  pBroker_->updateVariables(pmacMessageBroker::PMAC_SLOW_READ);

}

void pmacController::addBrokerVariables(const std::string &monitorVariables) {
  std::string var;
  char sep = ' ';
  unsigned long pos = 0;
  unsigned long next = 0;

  // add in slow poll monitoring of any variables added in the startup script
  // using pmacMonitorVariables
  while (next < monitorVariables.length()) {
    next = monitorVariables.find(sep, pos);
    if (next == std::string::npos) {
      next = monitorVariables.length();
    }
    var = monitorVariables.substr(pos, next - pos);
    pos = next + 1;
    pBroker_->addReadVariable(pmacMessageBroker::PMAC_SLOW_READ, var.c_str());
  }
}

void pmacController::setupBrokerVariables(void) {
  int plcNo = 0;
  int gpioNo = 0;
  int progNo = 0;
  char cmd[32];
  char response[64];
  static const char *functionName = "pmacController::setupBrokerVariables";

  // Add the items required for global status
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ,
                            pHardware_->getGlobalStatusCmd().c_str());
  debug(DEBUG_VARIABLE, functionName, "global status", pHardware_->getGlobalStatusCmd().c_str());

  // Add the feedrate values
  // first find out how many Coordinate Systems are enabled
  sprintf(cmd, "%s", pHardware_->getCSEnabledCountCmd().c_str());
  this->immediateWriteRead(cmd, response);
  sscanf(response, "%d", &csCount);

  if(cid_ == PMAC_CID_POWER_) {
      //Power pmac coordinate systems are from 0 - 15 so subtract 1
      csCount--;
  } else {
      // Turbo pmac I68 is one less than the count
      csCount++;
  }
  debug(DEBUG_VARIABLE, functionName, "Count of CSes enabled %d", csCount);
  for (int csNo = 1; csNo <= csCount; csNo++) {
    sprintf(cmd, "&%d%s", csNo, "%");
    debug(DEBUG_VARIABLE, functionName, "Adding feedrate check", cmd);
    pBroker_->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, cmd);
  }

  // Add I42 to the slow loop to monitor PVT time control mode
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_SLOW_READ, PMAC_PVT_TIME_MODE);

  // CPU Calculation requires a set of I and M variables
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, PMAC_CPU_PHASE_INTR);
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, PMAC_CPU_PHASE_TIME);
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, PMAC_CPU_SERVO_TIME);
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, PMAC_CPU_RTI_TIME);
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_SLOW_READ, PMAC_CPU_I8);
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_SLOW_READ, PMAC_CPU_I7002);

  // Add the PMAC P variables required for trajectory scanning
  // Fast readout required of these values
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_PRE_FAST_READ, PMAC_TRAJ_STATUS);
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, PMAC_TRAJ_CURRENT_INDEX);
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, PMAC_TRAJ_CURRENT_BUFFER);
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, PMAC_TRAJ_TOTAL_POINTS);

  // Medium readout of the PLC program status (same for both Geobrick and VME)
  for (plcNo = 0; plcNo < 32; plcNo++) {
    sprintf(cmd, "M%d", (plcNo + 5000));
    pBroker_->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, cmd);
  }

  // Medium readout of the motion program status (same for both Geobrick and VME)
  for (progNo = 0; progNo < 16; progNo++) {
    sprintf(cmd, "M%d", ((progNo * 100) + 5180));
    pBroker_->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, cmd);
  }

  // Medium readout of the GPIO status bits
  switch (cid_) {
    case PMAC_CID_GEOBRICK_:
    case PMAC_CID_CLIPPER_:
    case PMAC_CID_POWER_:
      // Outputs
      for (gpioNo = 0; gpioNo < 8; gpioNo++) {
        sprintf(cmd, "M%d", (gpioNo + 32));
        pBroker_->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, cmd);
      }
      // Inputs
      for (gpioNo = 0; gpioNo < 16; gpioNo++) {
        sprintf(cmd, "M%d", gpioNo);
        pBroker_->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, cmd);
      }
      break;

    case PMAC_CID_PMAC_:
      // Outputs
      for (gpioNo = 0; gpioNo < 8; gpioNo++) {
        sprintf(cmd, "M%d", (gpioNo + 7716));
        pBroker_->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, cmd);
        sprintf(cmd, "M%d", (gpioNo + 7740));
        pBroker_->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, cmd);
      }
      // Inputs
      for (gpioNo = 0; gpioNo < 8; gpioNo++) {
        sprintf(cmd, "M%d", (gpioNo + 7616));
        pBroker_->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, cmd);
        sprintf(cmd, "M%d", (gpioNo + 7640));
        pBroker_->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, cmd);
      }
      break;

    default:
      // As we couldn't read the cid from the PMAC we don't know which m-vars to read
      debug(DEBUG_ERROR, functionName, "Unable to set GPIO M-vars, unknown Card ID");
  }

  // Slow readout required of trajectory buffer setup
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_SLOW_READ, PMAC_TRAJ_BUFFER_LENGTH);
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_SLOW_READ, PMAC_TRAJ_BUFF_ADR_A);
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_SLOW_READ, PMAC_TRAJ_BUFF_ADR_B);
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_SLOW_READ, PMAC_TRAJ_PROG_VERSION);


  // Register this class for updates
  pBroker_->registerForLocks(this);
  pBroker_->registerForUpdates(this, pmacMessageBroker::PMAC_PRE_FAST_READ);
  pBroker_->registerForUpdates(this, pmacMessageBroker::PMAC_FAST_READ);
  pBroker_->registerForUpdates(this, pmacMessageBroker::PMAC_MEDIUM_READ);
  pBroker_->registerForUpdates(this, pmacMessageBroker::PMAC_SLOW_READ);
}

void pmacController::registerForLock(asynPortDriver *controller) {
  pBroker_->registerForLocks(controller);
}

void pmacController::startPMACPolling() {
  // Start the underlying polling thread (asynMotorController)
  startPoller(movingPollPeriod_, idlePollPeriod_, PMAC_FORCED_FAST_POLLS_);
}

void pmacController::setDebugLevel(int level, int axis, int csNo) {
  // If the cs number is greater than 0 then send the demand on to the CS
  if (csNo > 0) {
    if (pCSControllers_[csNo] != NULL) {
      pCSControllers_[csNo]->setDebugLevel(level, axis);
    } else {
      printf("Cannot set CS debug level, invalid CS %d\n", csNo);
    }
  } else {
    // Check if an axis or controller wide debug is to be set
    if (axis == 0) {
      printf("Setting PMAC controller debug level to %d\n", level);
      // Set the level for the controller
      this->setLevel(level);
      // Set the level for the broker
      pBroker_->setLevel(level);
      // Set the level for the groups container
      if (pGroupList != NULL) {
        pGroupList->setLevel(level);
      }
    } else {
      if (this->getAxis(axis) != NULL) {
        printf("Setting PMAC axis %d debug level to %d\n", axis, level);
        this->getAxis(axis)->setLevel(level);
      }
    }
  }
}

asynStatus
pmacController::drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName,
                              size_t *psize) {
  static const char *functionName = "drvUserCreate";
  asynStatus status = asynSuccess;
  int index;
  // Accepted parameter formats
  //
  // For reading variables
  // PMAC_VxF_...  => PMAC Variable Fast Loop
  // PMAC_VxM_...  => PMAC Variable Medium Loop
  // PMAC_VxS_...  => PMAC Variable Slow Loop
  //
  // x is I for int, H for hex, D for double or S for string
  //
  // There must be no j or = in a variable, these items will simply be polled for their current status
  //
  // For Writing only
  // PMAC_WI_... => Write Integer Value
  // PMAC_WD_... => Write Double Value
  // PMAC_WS_... => Write String Value
  //
  // Writing to these parameters will result in immediate writes to the PMAC


  // Check if we have already provided maximum number of custom parameters
  if (parameterIndex_ >= PMAC_MAX_PARAMETERS) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: Not enough space allocated to store a new custom parameter, \
                    nothing will be done for this parameter.\n",
              driverName, functionName);
    status = asynError;
  }

  if (status == asynSuccess) {

    if (findParam(drvInfo, &index) && strlen(drvInfo) > 9 && strncmp(drvInfo, "PMAC_V", 6) == 0 &&
        drvInfo[8] == '_') {

      // Retrieve the name of the variable
      char *rawPmacVariable = epicsStrDup(drvInfo + 9);
      char pmacVariable[128];
      this->processDrvInfo(rawPmacVariable, pmacVariable);

      debug(DEBUG_VARIABLE, functionName, "Creating new parameter", pmacVariable);
      // Check for I, D or S in drvInfo[6]
      switch (drvInfo[6]) {
        case 'I':
          // Create the parameter
          createParam(drvInfo, asynParamInt32, &(this->parameters[parameterIndex_]));
          setIntegerParam(this->parameters[parameterIndex_], 0);
          // Add variable to integer parameter hashtable
          this->pIntParams_->insert(pmacVariable, this->parameters[parameterIndex_]);
          parameterIndex_++;
          break;
        case 'H':
          // Create the parameter
          createParam(drvInfo, asynParamInt32, &(this->parameters[parameterIndex_]));
          setIntegerParam(this->parameters[parameterIndex_], 0);
          // Add variable to integer parameter hashtable
          this->pHexParams_->insert(pmacVariable, this->parameters[parameterIndex_]);
          parameterIndex_++;
          break;
        case 'D':
          createParam(drvInfo, asynParamFloat64, &(this->parameters[parameterIndex_]));
          // Add variable to double parameter hashtable
          this->pDoubleParams_->insert(pmacVariable, this->parameters[parameterIndex_]);
          parameterIndex_++;
          break;
        case 'S':
          createParam(drvInfo, asynParamOctet, &(this->parameters[parameterIndex_]));
          // Add variable to string parameter hashtable
          this->pStringParams_->insert(pmacVariable, this->parameters[parameterIndex_]);
          parameterIndex_++;
          break;
        default:
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Expected PMAC_Vtx_... where x is one of I, D or S. Got '%c'\n",
                    driverName, functionName, drvInfo[6]);
          status = asynError;
      }

      if (status == asynSuccess) {
        // Check for F, M or S in drvInfo[7]
        switch (drvInfo[7]) {
          case 'F':
            this->pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, pmacVariable);
            break;
          case 'M':
            this->pBroker_->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, pmacVariable);
            break;
          case 'S':
            this->pBroker_->addReadVariable(pmacMessageBroker::PMAC_SLOW_READ, pmacVariable);
            break;
          default:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: Expected PMAC_Vtx_... where t is one of F, M or S. Got '%c'\n",
                      driverName, functionName, drvInfo[7]);
            status = asynError;
        }

        if (status == asynSuccess) {
          this->pWriteParams_->insert(drvInfo, pmacVariable);
        }
      }



      // Create the parameter, store the param ID along with the var name in the hashtable

    }

    if (findParam(drvInfo, &index) && strlen(drvInfo) > 9 && strncmp(drvInfo, "PMAC_W", 6) == 0 &&
        drvInfo[7] == '_') {

      // Retrieve the name of the variable
      char *rawPmacVariable = epicsStrDup(drvInfo + 8);
      char pmacVariable[128];
      this->processDrvInfo(rawPmacVariable, pmacVariable);

      debug(DEBUG_VARIABLE, functionName, "Creating new write only parameter", pmacVariable);
      // Check for I, D or S in drvInfo[7]
      switch (drvInfo[6]) {
        case 'I':
          // Create the parameter
          createParam(drvInfo, asynParamInt32, &(this->parameters[parameterIndex_]));
          // Add variable to write only parameter hashtable
          this->pWriteParams_->insert(drvInfo, pmacVariable);
          parameterIndex_++;
          break;
        case 'D':
          createParam(drvInfo, asynParamFloat64, &(this->parameters[parameterIndex_]));
          // Add variable to double parameter hashtable
          this->pWriteParams_->insert(drvInfo, pmacVariable);
          parameterIndex_++;
          break;
        case 'S':
          createParam(drvInfo, asynParamOctet, &(this->parameters[parameterIndex_]));
          // Add variable to string parameter hashtable
          this->pWriteParams_->insert(drvInfo, pmacVariable);
          parameterIndex_++;
          break;
        default:
          asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Expected PMAC_Wx_... where x is one of I, D or S. Got '%c'\n",
                    driverName, functionName, drvInfo[6]);
          status = asynError;
      }
    }

  }

  if (status == asynSuccess) {
    // Now return baseclass result
    status = asynMotorController::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
  }
  return status;
}

asynStatus pmacController::processDrvInfo(char *input, char *output) {
  asynStatus status = asynSuccess;
  char ppmacStart[128];
  char ppmacEnd[128];
  char ppostfix[128];
  short perr = 0;
  double presult = 0.0;
  char *sqPtr;
  char *eqPtr;
  char *pinfix;
  int lstatus = 0;
  static const char *functionName = "processDrvInfo";

  debug(DEBUG_FLOW, functionName);

  // Search for any ` characters that represent an expression
  sqPtr = strchr(input, '`');
  if (sqPtr != NULL) {
    strncpy(ppmacStart, input, (sqPtr - input));
    ppmacStart[sqPtr - input] = '\0';
    debug(DEBUG_VARIABLE, functionName, "Pre expression", ppmacStart);
    sqPtr++;
    // We've found a quote character, search for the end quote
    eqPtr = strchr(sqPtr, '`');
    if (eqPtr != NULL) {
      strcpy(ppmacEnd, eqPtr + 1);
      debug(DEBUG_VARIABLE, functionName, "Post expression", ppmacEnd);
      // We've found an end quote so check the string
      int len = eqPtr - sqPtr;
      pinfix = (char *) malloc((len + 1) * sizeof(char));
      memcpy(pinfix, sqPtr, len);
      pinfix[len] = '\0';
      debug(DEBUG_VARIABLE, functionName, "Input expression", pinfix);
      // Now run the expression through the calc routines
      lstatus = postfix(pinfix, ppostfix, &perr);
      if (lstatus != 0) {
        // postfix failed, report error
        strcpy(output, input);
        debug(DEBUG_ERROR, functionName, "Postfix expression error", calcErrorStr(perr));
        debug(DEBUG_ERROR, functionName, "Postfix failed expression", pinfix);
        status = asynError;
      } else {
        lstatus = calcPerform(NULL, &presult, ppostfix);
        if (lstatus != 0) {
          // postfix failed, report error
          strcpy(output, input);
          debug(DEBUG_ERROR, functionName, "Failed to evaluate postfix expression", pinfix);
          status = asynError;
        } else {
          debug(DEBUG_VARIABLE, functionName, "Calculated value", (int) presult);
          sprintf(output, "%s%d%s", ppmacStart, (int) presult, ppmacEnd);
          debug(DEBUG_VARIABLE, functionName, "Updated PMAC variable", output);
        }
      }
    } else {
      // We found only 1` so this cannot be an expression
      // Simply copy the input into the output
      strcpy(output, input);
    }
  } else {
    // Simply copy the input into the output
    strcpy(output, input);
  }

  return status;
}

void pmacController::callback(pmacCommandStore *sPtr, int type) {
  static const char *functionName = "callback";
  debug(DEBUG_FLOW, functionName);

  if (type == pmacMessageBroker::PMAC_PRE_FAST_READ) {
    // Execute the pre-fast update loop
    this->prefastUpdate(sPtr);
  }

  if (type == pmacMessageBroker::PMAC_FAST_READ) {
    // Execute the fast update loop
    this->fastUpdate(sPtr);
  }

  // If this is a slow callback then execute the slow update
  if (type == pmacMessageBroker::PMAC_MEDIUM_READ) {
    // Execute the medium update loop
    this->mediumUpdate(sPtr);
  }

  // If this is a slow callback then execute the slow update
  if (type == pmacMessageBroker::PMAC_SLOW_READ) {
    // Parse PMAC global status
    this->slowUpdate(sPtr);
  }

  lock();
  // Loop over parameter list and search for values
  // Check for integer params
  std::string key = this->pIntParams_->firstKey();
  if (key != "") {
    if (sPtr->checkForItem(key)) {
      int val = 0;
      sscanf(sPtr->readValue(key).c_str(), "%d", &val);
      debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
      debug(DEBUG_VARIABLE, functionName, "      value", val);
      setIntegerParam(this->pIntParams_->lookup(key), val);
    }
    while (this->pIntParams_->hasNextKey()) {
      key = this->pIntParams_->nextKey();
      if (sPtr->checkForItem(key)) {
        int val = 0;
        sscanf(sPtr->readValue(key).c_str(), "%d", &val);
        debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
        debug(DEBUG_VARIABLE, functionName, "      value", val);
        setIntegerParam(this->pIntParams_->lookup(key), val);
      }
    }
  }

  // Check for hex integer params
  key = this->pHexParams_->firstKey();
  if (key != "") {
    if (sPtr->checkForItem(key)) {
      int val = 0;
      sscanf(sPtr->readValue(key).c_str(), "$%x", &val);
      debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
      debug(DEBUG_VARIABLE, functionName, "      value", val);
      setIntegerParam(this->pHexParams_->lookup(key), val);
    }
    while (this->pHexParams_->hasNextKey()) {
      key = this->pHexParams_->nextKey();
      if (sPtr->checkForItem(key)) {
        int val = 0;
        sscanf(sPtr->readValue(key).c_str(), "$%x", &val);
        debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
        debug(DEBUG_VARIABLE, functionName, "      value", val);
        setIntegerParam(this->pHexParams_->lookup(key), val);
      }
    }
  }

  // Check for double params
  key = this->pDoubleParams_->firstKey();
  if (key != "") {
    if (sPtr->checkForItem(key)) {
      double val = 0;
      sscanf(sPtr->readValue(key).c_str(), "%lf", &val);
      debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
      debug(DEBUG_VARIABLE, functionName, "      value", val);
      setDoubleParam(this->pDoubleParams_->lookup(key), val);
    }
    while (this->pDoubleParams_->hasNextKey()) {
      key = this->pDoubleParams_->nextKey();
      if (sPtr->checkForItem(key)) {
        double val = 0;
        sscanf(sPtr->readValue(key).c_str(), "%lf", &val);
        debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
        debug(DEBUG_VARIABLE, functionName, "      value", val);
        setDoubleParam(this->pDoubleParams_->lookup(key), val);
      }
    }
  }

  // Check for string params
  key = this->pStringParams_->firstKey();
  if (key != "") {
    if (sPtr->checkForItem(key)) {
      char val[MAX_STRING_SIZE];
      sscanf(sPtr->readValue(key).c_str(), "%s", val);
      debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
      debug(DEBUG_VARIABLE, functionName, "      value", (char *) val);
      setStringParam(this->pStringParams_->lookup(key), val);
    }
    while (this->pStringParams_->hasNextKey()) {
      key = this->pStringParams_->nextKey();
      if (sPtr->checkForItem(key)) {
        char val[MAX_STRING_SIZE];
        sscanf(sPtr->readValue(key).c_str(), "%s", val);
        debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
        debug(DEBUG_VARIABLE, functionName, "      value", (char *) val);
        setStringParam(this->pDoubleParams_->lookup(key), val);
      }
    }
  }
  unlock();
  callParamCallbacks();
}

asynStatus pmacController::slowUpdate(pmacCommandStore *sPtr) {
  asynStatus status = asynSuccess;
  int nvals;
  std::string trajPtr;
  int storeSize = 0;
  static const char *functionName = "slowUpdate";
  std::string value;
  debug(DEBUG_FLOW, functionName);

  // read in the combined variables values.
  value = sPtr->getVariablesList("I");
  setStringParam(PMAC_I_Variables_, value.c_str());
  value = sPtr->getVariablesList("P");
  setStringParam(PMAC_P_Variables_, value.c_str());
  value = sPtr->getVariablesList("M");
  setStringParam(PMAC_M_Variables_, value.c_str());

  // Read the length of avaialable trajectory buffers
  trajPtr = sPtr->readValue(PMAC_TRAJ_BUFFER_LENGTH);
  if (trajPtr == "") {
    debug(DEBUG_ERROR, functionName, "Problem reading trajectory buffer length",
          PMAC_TRAJ_BUFFER_LENGTH);
    status = asynError;
  } else {
    nvals = sscanf(trajPtr.c_str(), "%d", &tScanPmacBufferSize_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading trajectory buffer length",
            PMAC_TRAJ_BUFFER_LENGTH);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", trajPtr);
      status = asynError;
    } else {
      // Save the value into the parameter
      setIntegerParam(PMAC_C_TrajBufferLength_, tScanPmacBufferSize_);
      debugf(DEBUG_VARIABLE, functionName, "Slow read trajectory buffer length [%s] => %d",
             PMAC_TRAJ_BUFFER_LENGTH, tScanPmacBufferSize_);
    }
  }

  // Read the address of the A half buffer
  trajPtr = sPtr->readValue(PMAC_TRAJ_BUFF_ADR_A);
  if (trajPtr == "") {
    debug(DEBUG_ERROR, functionName, "Problem reading address of A buffer", PMAC_TRAJ_BUFF_ADR_A);
    status = asynError;
  } else {
    nvals = sscanf(trajPtr.c_str(), "%d", &tScanPmacBufferAddressA_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading address of A buffer", PMAC_TRAJ_BUFF_ADR_A);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", trajPtr);
      status = asynError;
    } else {
      // Save the value into the parameter
      setIntegerParam(PMAC_C_TrajBuffAdrA_, tScanPmacBufferAddressA_);
      debugf(DEBUG_VARIABLE, functionName, "Slow read trajectory address buffer A [%s] => %X",
             PMAC_TRAJ_BUFF_ADR_A, tScanPmacBufferAddressA_);
    }
  }

  // Read the address of the B half buffer
  trajPtr = sPtr->readValue(PMAC_TRAJ_BUFF_ADR_B);
  if (trajPtr == "") {
    debug(DEBUG_ERROR, functionName, "Problem reading address of B buffer", PMAC_TRAJ_BUFF_ADR_B);
    status = asynError;
  } else {
    nvals = sscanf(trajPtr.c_str(), "%d", &tScanPmacBufferAddressB_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading address of B buffer", PMAC_TRAJ_BUFF_ADR_B);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", trajPtr);
      status = asynError;
    } else {
      // Save the value into the parameter
      setIntegerParam(PMAC_C_TrajBuffAdrB_, tScanPmacBufferAddressB_);
      debugf(DEBUG_VARIABLE, functionName, "Slow read trajectory address buffer B [%s] => %X",
             PMAC_TRAJ_BUFF_ADR_B, tScanPmacBufferAddressB_);
    }
  }

  // Read the version number of the motion program
  trajPtr = sPtr->readValue(PMAC_TRAJ_PROG_VERSION);
  if (trajPtr == "") {
    debug(DEBUG_ERROR, functionName, "Problem reading motion program version",
          PMAC_TRAJ_PROG_VERSION);
    status = asynError;
  } else {
    nvals = sscanf(trajPtr.c_str(), "%lf", &tScanPmacProgVersion_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading motion program version",
            PMAC_TRAJ_PROG_VERSION);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", trajPtr);
      status = asynError;
    } else {
      // Save the value into the parameter
      setDoubleParam(PMAC_C_TrajProgVersion_, tScanPmacProgVersion_);
      debugf(DEBUG_VARIABLE, functionName, "Slow read trajectory motion program version [%s] => %X",
             PMAC_TRAJ_PROG_VERSION, tScanPmacProgVersion_);
    }
  }

  // Read the value of PVT time control mode
  trajPtr = sPtr->readValue(PMAC_PVT_TIME_MODE);
  if (trajPtr == "") {
    debug(DEBUG_ERROR, functionName, "Problem reading PVT time control mode", PMAC_PVT_TIME_MODE);
    status = asynError;
  } else {
    nvals = sscanf(trajPtr.c_str(), "%d", &pvtTimeMode_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading PVT time control mode", PMAC_PVT_TIME_MODE);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", trajPtr);
      status = asynError;
    } else {
      debugf(DEBUG_VARIABLE, functionName, "PVT time control mode [%s] => %X", PMAC_PVT_TIME_MODE,
             pvtTimeMode_);
    }
  }

  // Used for CPU calculation
  if (status == asynSuccess) {
    status = parseIntegerVariable(PMAC_CPU_I8, sPtr->readValue(PMAC_CPU_I8),
                                  "Real-time interrupt period", i8_);
  }
  if (status == asynSuccess) {
    status = parseIntegerVariable(PMAC_CPU_I7002, sPtr->readValue(PMAC_CPU_I7002),
                                  "Servo clock frequency", i7002_);
  }

  // Read out the size of the pmac command stores
  if (pBroker_->readStoreSize(pmacMessageBroker::PMAC_FAST_READ, &storeSize) == asynSuccess) {
    setIntegerParam(PMAC_C_FastStore_, storeSize);
  }
  if (pBroker_->readStoreSize(pmacMessageBroker::PMAC_MEDIUM_READ, &storeSize) == asynSuccess) {
    setIntegerParam(PMAC_C_MediumStore_, storeSize);
  }
  if (pBroker_->readStoreSize(pmacMessageBroker::PMAC_SLOW_READ, &storeSize) == asynSuccess) {
    setIntegerParam(PMAC_C_SlowStore_, storeSize);
  }

  return status;
}

asynStatus pmacController::mediumUpdate(pmacCommandStore *sPtr) {
  asynStatus status = asynSuccess;
  int nvals = 0;
  int plc = 0;
  int plcBit = 0;
  int plcBits00 = 0;
  int plcBits01 = 0;
  std::string plcString = "";
  int gpio = 0;
  int gpioBit = 0;
  int gpioOutputs = 0;
  int gpioInputs = 0;
  std::string gpioString = "";
  int prog = 0;
  int progBit = 0;
  int progBits = 0;
  std::string progString = "";
  int axisCs = 0;
  char command[8];
  int feedrate = 0;
  bool printErrors = 0;
  int feedrate_limit = 0;
  int min_feedrate = 0;
  int min_feedrate_cs = 0;
  static const char *functionName = "mediumUpdate";
  debug(DEBUG_FLOW, functionName);

  // Get the time and decide if we want to print errors.
  epicsTimeGetCurrent(&nowTime_);
  nowTimeSecs_ = nowTime_.secPastEpoch;
  if ((nowTimeSecs_ - lastTimeSecs_) < PMAC_ERROR_PRINT_TIME_) {
    printErrors = 0;
  } else {
    printErrors = 1;
    lastTimeSecs_ = nowTimeSecs_;
  }

  if (printNextError_) {
    printErrors = 1;
  }

  // Read the PLC program status variables
  for (plc = 0; plc < 32; plc++) {
    sprintf(command, "M%d", (plc + 5000));
    plcString = sPtr->readValue(command);
    if (plcString == "") {
      debug(DEBUG_VARIABLE, functionName, "Problem reading PLC program status", command);
      status = asynError;
    } else {
      nvals = sscanf(plcString.c_str(), "%d", &plcBit);
      if (nvals != 1) {
        debug(DEBUG_VARIABLE, functionName, "Error reading PLC program status", command);
        debug(DEBUG_VARIABLE, functionName, "    nvals", nvals);
        debug(DEBUG_VARIABLE, functionName, "    response", plcString);
        status = asynError;
      } else {
        if (plc < 16) {
          plcBits00 += plcBit << plc;
        } else {
          plcBits01 += plcBit << (plc - 16);
        }
      }
    }
  }

  // Read the GPIO status variables
  switch (cid_) {
    case PMAC_CID_GEOBRICK_:
    case PMAC_CID_CLIPPER_:
    case PMAC_CID_POWER_:
      // Outputs
      for (gpio = 0; gpio < 8; gpio++) {
        sprintf(command, "M%d", (gpio + 32));
        gpioString = sPtr->readValue(command);
        if (gpioString == "") {
          debug(DEBUG_VARIABLE, functionName, "Problem reading GPIO status", command);
          status = asynError;
        } else {
          nvals = sscanf(gpioString.c_str(), "%d", &gpioBit);
          if (nvals != 1) {
            debug(DEBUG_VARIABLE, functionName, "Error reading GPIO status", command);
            debug(DEBUG_VARIABLE, functionName, "    nvals", nvals);
            debug(DEBUG_VARIABLE, functionName, "    response", gpioString);
            status = asynError;
          } else {
            gpioOutputs += gpioBit << gpio;
          }
        }
      }
      // Inputs
      for (gpio = 0; gpio < 16; gpio++) {
        sprintf(command, "M%d", gpio);
        gpioString = sPtr->readValue(command);
        if (gpioString == "") {
          debug(DEBUG_VARIABLE, functionName, "Problem reading GPIO status", command);
          status = asynError;
        } else {
          nvals = sscanf(gpioString.c_str(), "%d", &gpioBit);
          if (nvals != 1) {
            debug(DEBUG_VARIABLE, functionName, "Error reading GPIO status", command);
            debug(DEBUG_VARIABLE, functionName, "    nvals", nvals);
            debug(DEBUG_VARIABLE, functionName, "    response", gpioString);
            status = asynError;
          } else {
            gpioInputs += gpioBit << gpio;
          }
        }
      }
      break;

    case PMAC_CID_PMAC_:
      // Outputs
      for (gpio = 0; gpio < 8; gpio++) {
        sprintf(command, "M%d", (gpio + 7716));
        gpioString = sPtr->readValue(command);
        if (gpioString == "") {
          debug(DEBUG_VARIABLE, functionName, "Problem reading GPIO status", command);
          status = asynError;
        } else {
          nvals = sscanf(gpioString.c_str(), "%d", &gpioBit);
          if (nvals != 1) {
            debug(DEBUG_VARIABLE, functionName, "Error reading GPIO status", command);
            debug(DEBUG_VARIABLE, functionName, "    nvals", nvals);
            debug(DEBUG_VARIABLE, functionName, "    response", gpioString);
            status = asynError;
          } else {
            gpioOutputs += gpioBit << (gpio + 8);
          }
        }
        sprintf(command, "M%d", (gpio + 7740));
        gpioString = sPtr->readValue(command);
        if (gpioString == "") {
          debug(DEBUG_VARIABLE, functionName, "Problem reading GPIO status", command);
          status = asynError;
        } else {
          nvals = sscanf(gpioString.c_str(), "%d", &gpioBit);
          if (nvals != 1) {
            debug(DEBUG_VARIABLE, functionName, "Error reading GPIO status", command);
            debug(DEBUG_VARIABLE, functionName, "    nvals", nvals);
            debug(DEBUG_VARIABLE, functionName, "    response", gpioString);
            status = asynError;
          } else {
            gpioOutputs += gpioBit << gpio;
          }
        }
      }
      // Inputs
      for (gpio = 0; gpio < 8; gpio++) {
        sprintf(command, "M%d", (gpio + 7616));
        gpioString = sPtr->readValue(command);
        if (gpioString == "") {
          debug(DEBUG_VARIABLE, functionName, "Problem reading GPIO status", command);
          status = asynError;
        } else {
          nvals = sscanf(gpioString.c_str(), "%d", &gpioBit);
          if (nvals != 1) {
            debug(DEBUG_VARIABLE, functionName, "Error reading GPIO status", command);
            debug(DEBUG_VARIABLE, functionName, "    nvals", nvals);
            debug(DEBUG_VARIABLE, functionName, "    response", gpioString);
            status = asynError;
          } else {
            gpioInputs += gpioBit << (gpio + 8);
          }
        }
        sprintf(command, "M%d", (gpio + 7640));
        gpioString = sPtr->readValue(command);
        if (gpioString == "") {
          debug(DEBUG_VARIABLE, functionName, "Problem reading GPIO status", command);
          status = asynError;
        } else {
          nvals = sscanf(gpioString.c_str(), "%d", &gpioBit);
          if (nvals != 1) {
            debug(DEBUG_VARIABLE, functionName, "Error reading GPIO status", command);
            debug(DEBUG_VARIABLE, functionName, "    nvals", nvals);
            debug(DEBUG_VARIABLE, functionName, "    response", gpioString);
            status = asynError;
          } else {
            gpioInputs += gpioBit << gpio;
          }
        }
      }
      break;

    default:
      // As we couldn't read the cid from the PMAC we don't know which m-vars to read
      debug(DEBUG_ERROR, functionName, "Unable to read GPIO M-vars, unknown Card ID");

  }

  // Read the motion program status variables
  for (prog = 0; prog < 16; prog++) {
    sprintf(command, "M%d", ((prog * 100) + 5180));
    progString = sPtr->readValue(command);
    if (progString == "") {
      debug(DEBUG_VARIABLE, functionName, "Problem reading motion program status", command);
      status = asynError;
    } else {
      nvals = sscanf(progString.c_str(), "%d", &progBit);
      if (nvals != 1) {
        debug(DEBUG_VARIABLE, functionName, "Error reading motion program status", command);
        debug(DEBUG_VARIABLE, functionName, "    nvals", nvals);
        debug(DEBUG_VARIABLE, functionName, "    response", progString);
        status = asynError;
      } else {
        progBits += progBit << prog;
      }
    }
  }

  // For each axis read try to read the assignment
  for (int axis = 1; axis <= this->numAxes_ - 1; axis++) {
    axisCs = 0;
    if (this->getAxis(axis) != NULL) {
      axisCs = this->getAxis(axis)->getAxisCSNo();
    }
    if (axisCs > 0) {
      if (pCSControllers_[axisCs]) {
        setIntegerParam(axis, PMAC_C_GroupCSPortRBV_, axisCs);
      } else {
        setIntegerParam(axis, PMAC_C_GroupCSPortRBV_, 0);
      }
      std::string cs_cmd = pHardware_->getCSMappingCmd(axisCs, axis).c_str();
      if (sPtr->checkForItem(cs_cmd)) {
        const std::string result = pHardware_->parseCSMappingResult(
                sPtr->readValue(cs_cmd));
        debugf(DEBUG_VARIABLE, functionName, "Axis %d CS %d assignment: %s",
                axis, axisCs, result.c_str());
        setStringParam(axis, PMAC_C_GroupAssignRBV_, result.c_str());
      } else {
        sPtr->addItem(cs_cmd);
      }
    } else {
      setStringParam(axis, PMAC_C_GroupAssignRBV_, "");
      setIntegerParam(axis, PMAC_C_GroupCSPortRBV_, 0);
    }
  }

  min_feedrate = 100;
  min_feedrate_cs = 0;
  // Lookup the value of the feedrate
  for (int csNo = 1; csNo <= csCount; csNo++) {
    // only check feedrate on those CS that we are using (have registered config)
    if (pCSControllers_[csNo] != NULL) {
      sprintf(command, "&%d%s", csNo, "%");
      std::string feedRate = sPtr->readValue(command);
      debugf(DEBUG_VARIABLE, functionName, "Feedrate [&%d%s] => %s", csNo, "%", feedRate.c_str());
      // Check the feedrate value is valid
      if (feedRate == "") {
        debug(DEBUG_ERROR, functionName, "Problem reading feed rate command %");
        status = asynError;
      } else {
        nvals = sscanf(feedRate.c_str(), "%d", &feedrate);
        if (nvals != 1) {
          debug(DEBUG_ERROR, functionName, "Error reading feedrate [%]");
          debug(DEBUG_ERROR, functionName, "    nvals", nvals);
          debug(DEBUG_ERROR, functionName, "    response", feedRate);
          status = asynError;
        } else {
          if (feedrate < min_feedrate) {
            min_feedrate = feedrate;
            min_feedrate_cs = csNo;
          }
        }
      }
    }
  }
  if (status == asynSuccess) {
    status = getIntegerParam(this->PMAC_C_FeedRateLimit_, &feedrate_limit);
  }

  if (status == asynSuccess) {
    if (min_feedrate < static_cast<int>(feedrate_limit - PMAC_FEEDRATE_DEADBAND_)) {
      status = setIntegerParam(this->PMAC_C_FeedRateProblem_, PMAC_ERROR_);
      int prev_min_cs = 0;
      getIntegerParam(PMAC_C_FeedRateCS_, &prev_min_cs);
      setIntegerParam(PMAC_C_FeedRateCS_, min_feedrate_cs);
      if (printErrors || (min_feedrate_cs != prev_min_cs)) {
        printNextError_ = false;
        debugf(DEBUG_ERROR, functionName,
               "*** ERROR ***: Coordinate System %d feed rate below limit.", min_feedrate_cs);
        debug(DEBUG_ERROR, functionName, "               feedrate", min_feedrate);
        debug(DEBUG_ERROR, functionName, "               feedrate limit", feedrate_limit);
      }
    } else {
      status = setIntegerParam(this->PMAC_C_FeedRateProblem_, PMAC_OK_);
      printNextError_ = true;
    }
  }
  if (status == asynSuccess) {
    status = setIntegerParam(this->PMAC_C_FeedRate_, min_feedrate);
  }


  // Call callbacks for PLC program status
  if (status == asynSuccess) {
    setIntegerParam(PMAC_C_PLCBits00_, plcBits00);
    setIntegerParam(PMAC_C_PLCBits01_, plcBits01);
    setIntegerParam(PMAC_C_GpioInputs_, gpioInputs);
    setIntegerParam(PMAC_C_GpioOutputs_, gpioOutputs);
    setIntegerParam(PMAC_C_ProgBits_, progBits);
    callParamCallbacks();
  }

  return status;
}

asynStatus pmacController::prefastUpdate(pmacCommandStore *sPtr) {
  asynStatus status = asynSuccess;
  int nvals;
  std::string trajBufPtr = "";
  static const char *functionName = "prefastUpdate";

  // Read the current trajectory status from the PMAC
  trajBufPtr = sPtr->readValue(PMAC_TRAJ_STATUS);
  if (trajBufPtr == "") {
    debug(DEBUG_ERROR, functionName, "Problem reading trajectory status", PMAC_TRAJ_STATUS);
    status = asynError;
  } else {
    nvals = sscanf(trajBufPtr.c_str(), "%d", &tScanPmacStatus_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading trajectory status", PMAC_TRAJ_STATUS);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", trajBufPtr);
      status = asynError;
    } else {
      // Save the value into the parameter
      setIntegerParam(PMAC_C_TrajStatus_, tScanPmacStatus_);
      debugf(DEBUG_VARIABLE, functionName, "Fast read trajectory status [%s] => %d",
             PMAC_TRAJ_STATUS, tScanPmacStatus_);
    }
  }

  return status;
}

asynStatus pmacController::fastUpdate(pmacCommandStore *sPtr) {
  asynStatus status = asynSuccess;
  int gStatus = 0;
  int gStat1 = 0;
  int gStat2 = 0;
  int gStat3 = 0;
  bool hardwareProblem;
  int nvals;
  std::string trajBufPtr = "";
  static const char *functionName = "fastUpdate";

  // Read the current trajectory buffer index read from the PMAC (within current buffer)
  trajBufPtr = sPtr->readValue(PMAC_TRAJ_CURRENT_INDEX);
  if (trajBufPtr == "") {
    debug(DEBUG_ERROR, functionName, "Problem reading trajectory current index",
          PMAC_TRAJ_CURRENT_INDEX);
    status = asynError;
  } else {
    nvals = sscanf(trajBufPtr.c_str(), "%d", &tScanPmacBufferPtr_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading trajectory current index",
            PMAC_TRAJ_CURRENT_INDEX);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", trajBufPtr);
      status = asynError;
    } else {
      // Save the value into the parameter
      setIntegerParam(PMAC_C_TrajCurrentIndex_, tScanPmacBufferPtr_);
      debugf(DEBUG_VARIABLE, functionName, "Fast read trajectory current index [%s] => %d",
             PMAC_TRAJ_CURRENT_INDEX, tScanPmacBufferPtr_);
    }
  }

  // Read the current trajectory total number of points from the PMAC
  trajBufPtr = sPtr->readValue(PMAC_TRAJ_TOTAL_POINTS);
  if (trajBufPtr == "") {
    debug(DEBUG_ERROR, functionName, "Problem reading trajectory total points",
          PMAC_TRAJ_TOTAL_POINTS);
    status = asynError;
  } else {
    nvals = sscanf(trajBufPtr.c_str(), "%d", &tScanPmacTotalPts_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading trajectory current index",
            PMAC_TRAJ_TOTAL_POINTS);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", trajBufPtr);
      status = asynError;
    } else {
      // Save the value into the parameter
      setIntegerParam(PMAC_C_TrajTotalPoints_, tScanPmacTotalPts_);
      debugf(DEBUG_VARIABLE, functionName, "Fast read trajectory total points [%s] => %d",
             PMAC_TRAJ_TOTAL_POINTS, tScanPmacTotalPts_);
      // Now work out the percent complete
      double pctComplete = 0.0;
      if (tScanNumPoints_ > 0) {
        pctComplete = (double) tScanPmacTotalPts_ * 100.0 / (double) tScanNumPoints_;
      }
      setDoubleParam(PMAC_C_TrajPercent_, pctComplete);
    }
  }

  // Read the current trajectory buffer (A=0,B=1) being read by the PMAC
  trajBufPtr = sPtr->readValue(PMAC_TRAJ_CURRENT_BUFFER);
  if (trajBufPtr == "") {
    debug(DEBUG_ERROR, functionName, "Problem reading trajectory current buffer",
          PMAC_TRAJ_CURRENT_BUFFER);
    status = asynError;
  } else {
    nvals = sscanf(trajBufPtr.c_str(), "%d", &tScanPmacBufferNumber_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading trajectory current buffer",
            PMAC_TRAJ_CURRENT_BUFFER);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", trajBufPtr);
      status = asynError;
    } else {
      // Save the value into the parameter
      setIntegerParam(PMAC_C_TrajCurrentBuffer_, tScanPmacBufferNumber_);
      debugf(DEBUG_VARIABLE, functionName, "Fast read trajectory current buffer [%s] => %d",
             PMAC_TRAJ_CURRENT_BUFFER, tScanPmacBufferNumber_);
    }
  }

  // Lookup the value of global status
  std::string globStatus = sPtr->readValue(pHardware_->getGlobalStatusCmd());
  debug(DEBUG_VARIABLE, functionName, "Global status", globStatus);

  globalStatus gs;
  status = pHardware_->parseGlobalStatus(globStatus, gs);
  //parseGlobalStatus(globStatus, &globalStatus, &gStat1, &gStat2, &gStat3);
  if (status == asynSuccess) {
    gStatus = gs.status_;
    gStat1 = gs.stat1_;
    gStat2 = gs.stat2_;
    gStat3 = gs.stat3_;
    setIntegerParam(PMAC_C_StatusBits01_, gStat1);
    setIntegerParam(PMAC_C_StatusBits02_, gStat2);
    setIntegerParam(PMAC_C_StatusBits03_, gStat3);
  }

  /*// Check the global status value is valid
  if (globStatus == ""){
    debug(DEBUG_ERROR, functionName, "Problem reading global status command ???");
    status = asynError;
  } else {
    nvals = sscanf(globStatus.c_str(), "%6x", &globalStatus);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading global status [???]");
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", globStatus);
      status = asynError;
    }
    nvals = sscanf(globStatus.c_str(), "%4x%4x%4x", &gStat1, &gStat2, &gStat3);
    if (nvals == 3){
      setIntegerParam(PMAC_C_StatusBits01_, gStat1);
      setIntegerParam(PMAC_C_StatusBits02_, gStat2);
      setIntegerParam(PMAC_C_StatusBits03_, gStat3);
    }
  }*/


  if (cid_ == PMAC_CID_GEOBRICK_ || cid_ == PMAC_CID_CLIPPER_) {
    // CPU Calculation
    int m70 = 0;
    int m71 = 0;
    int m72 = 0;
    int m73 = 0;
    if (status == asynSuccess) {
      status = parseIntegerVariable(PMAC_CPU_PHASE_INTR, sPtr->readValue(PMAC_CPU_PHASE_INTR),
                                    "Phase interrupt", m70);
    }
    if (status == asynSuccess) {
      status = parseIntegerVariable(PMAC_CPU_PHASE_TIME, sPtr->readValue(PMAC_CPU_PHASE_TIME),
                                    "Phase time", m71);
    }
    if (status == asynSuccess) {
      status = parseIntegerVariable(PMAC_CPU_SERVO_TIME, sPtr->readValue(PMAC_CPU_SERVO_TIME),
                                    "Servo time", m72);
    }
    if (status == asynSuccess) {
      status = parseIntegerVariable(PMAC_CPU_RTI_TIME, sPtr->readValue(PMAC_CPU_RTI_TIME),
                                    "RTI time", m73);
    }
    if (status == asynSuccess) {
      double P70 = i7002_ +
                   1;                                                        // phase interrupts per servo interrupt
      double P71 = (double) m71 /
                   (double) m70;                                         // Phase task duty cycle
      double P69 = double(m72 /
                          m70);                                                 // # of times phase interrupted servo
      double P72 = (m72 - P69 * m71) /
                   (m70 * P70);                                         // Servo task duty cycle
      double P68 = double(m73 /
                          m70);                                                 // # of times phase interrupted RTI
      double P67 = double(m73 / (m70 *
                                 (int) P70));                                      // # of times servo interrupted RTI
      double P73 = ((double) m73 - P68 * (double) m71 - P67 * ((double) m72 - P69 * (double) m71))
                   / ((double) m70 * P70 *
                      double(i8_ + 1));                                       // RTI duty cycle
      double P74 = 100.0 * (P71 + P72 +
                            P73);                                             // Latest total foreground duty cycle
      debug(DEBUG_TRACE, functionName, "Calculated CPU %", P74);
      setDoubleParam(PMAC_C_CpuUsage_, P74);
    }
  }
  //Set any controller specific parameters.
  //Some of these may be used by the axis poll to set axis problem bits.
  if (status == asynSuccess) {
    hardwareProblem = ((gStatus & PMAC_HARDWARE_PROB) != 0);
    status = setIntegerParam(this->PMAC_C_GlobalStatus_, hardwareProblem);
    if (hardwareProblem) {
      debug(DEBUG_ERROR, functionName, "*** Hardware Problem *** global status [???]",
            (int) gStatus);
    }
  }

  callParamCallbacks();

  if (status != asynSuccess) {
    debug(DEBUG_ERROR, functionName, "Error reading or setting params.");
    setIntegerParam(PMAC_C_CommsError_, PMAC_ERROR_);
  } else {
    setIntegerParam(PMAC_C_CommsError_, PMAC_OK_);
  }

  return status;
}

asynStatus pmacController::parseIntegerVariable(const std::string &command,
                                                const std::string &response,
                                                const std::string &desc,
                                                int &value) {
  asynStatus status = asynSuccess;
  static const char *functionName = "parseIntegerVariable";
  int iValue = 0;
  int nvals = 0;

  if (response == "") {
    debug(DEBUG_ERROR, functionName, "Read Error [" + desc + "]", command);
    status = asynError;
  } else {
    nvals = sscanf(response.c_str(), "%d", &iValue);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Read Error [" + desc + "]", command);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", response);
      status = asynError;
    } else {
      value = iValue;
    }
  }
  return status;
}

asynStatus pmacController::immediateWriteRead(const char *command, char *response) {
  asynStatus status = asynSuccess;
  static const char *functionName = "immediateWriteRead";
  this->startTimer(DEBUG_TIMING, functionName);
  status = this->lowLevelWriteRead(command, response);
  this->stopTimer(DEBUG_TIMING, functionName, "PMAC write/read time");
  return status;
}

asynStatus pmacController::axisWriteRead(const char *command, char *response) {
  int readonly = 0;
  asynStatus status = asynSuccess;
  static const char *functionName = "axisWriteRead";
  // Here we need to check if we are in axis readonly mode
  getIntegerParam(PMAC_C_AxisReadonly_, &readonly);
  if (readonly == 0) {
    this->startTimer(DEBUG_TIMING, functionName);
    status = this->lowLevelWriteRead(command, response);
    this->stopTimer(DEBUG_TIMING, functionName, "PMAC write/read time");
  } else {
    debug(DEBUG_TRACE, functionName, "Axis command not sent (readonly mode)", command);
  }
  return status;
}

/**
 * Wrapper for asynOctetSyncIO write/read functions.
 * @param command - String command to send.
 * @response response - String response back.
 */
asynStatus pmacController::lowLevelWriteRead(const char *command, char *response) {
  asynStatus status = asynSuccess;
  static const char *functionName = "lowLevelWriteRead";

  debug(DEBUG_FLOW, functionName);

  // Check if we are connected, if not then do not continue
  if (connected_ != 0) {
    status = pBroker_->immediateWriteRead(command, response);
    if (status == asynSuccess) {
      status = this->updateStatistics();
    }
  } else {
    strcpy(response, "");
    status = asynError;
    // there is (most likely) a conection issue
    connected_ = false;
  }
  return status;
}

void pmacController::report(FILE *fp, int level) {
  int axis = 0;
  pmacAxis *pAxis = NULL;

  fprintf(fp, "pmac motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
          this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
    for (axis = 0; axis < numAxes_; axis++) {
      pAxis = getAxis(axis);
      if (!pAxis) continue;
      fprintf(fp, "  axis %d\n"
                  "    scale = %d\n",
              pAxis->axisNo_,
              pAxis->scale_);
    }
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}

/**
 * Deal with controller specific epicsFloat64 params.
 * @param pasynUser
 * @param value
 * @param asynStatus
 */
asynStatus pmacController::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
  int function = pasynUser->reason;
  bool status = true;
  pmacAxis *pAxis = NULL;
  char command[PMAC_MAXBUF_] = {0};
  char response[PMAC_MAXBUF_] = {0};
  double encRatio = 1.0;
  epicsInt32 encposition = 0;
  const char *name[128];

  static const char *functionName = "writeFloat64";

  debug(DEBUG_FLOW, functionName);

  if (!initialised_) {
    return asynSuccess;
  }

  getParamName(function, name);
  debug(DEBUG_VARIABLE, functionName, "Parameter Updated", *name);
  pAxis = this->getAxis(pasynUser);
  if (!pAxis) {
    return asynError;
  }

  /* Set the parameter and readback in the parameter library. */
  status = (pAxis->setDoubleParam(function, value) == asynSuccess) && status;

  if (function == motorPosition_) {
    /*Set position on motor axis.*/
    epicsInt32 position = (epicsInt32) floor(value * 32 / pAxis->scale_ + 0.5);

    sprintf(command, "#%dK M%d61=%d*I%d08 M%d62=%d*I%d08",
            pAxis->axisNo_,
            pAxis->axisNo_, position, pAxis->axisNo_,
            pAxis->axisNo_, position, pAxis->axisNo_);

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s: Set axis %d on controller %s to position %f\n",
              functionName, pAxis->axisNo_, portName, value);

    if (command[0] != 0 && status) {
      status = (lowLevelWriteRead(command, response) == asynSuccess) && status;
    }

    sprintf(command, "#%dJ/", pAxis->axisNo_);

    if (command[0] != 0 && status) {
      status = (lowLevelWriteRead(command, response) == asynSuccess) && status;
    }

    /*Now set position on encoder axis, if one is in use.*/

    if (pAxis->encoder_axis_) {
      getDoubleParam(motorEncoderRatio_, &encRatio);
      encposition = (epicsInt32) floor((position * encRatio) + 0.5);

      sprintf(command, "#%dK M%d61=%d*I%d08 M%d62=%d*I%d08",
              pAxis->encoder_axis_,
              pAxis->encoder_axis_, encposition, pAxis->encoder_axis_,
              pAxis->encoder_axis_, encposition, pAxis->encoder_axis_);

      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s: Set encoder axis %d on controller %s to position %f\n",
                functionName, pAxis->axisNo_, portName, value);

      if (command[0] != 0 && status) {
        status = (lowLevelWriteRead(command, response) == asynSuccess) && status;
      }

      sprintf(command, "#%dJ/", pAxis->encoder_axis_);
      //The lowLevelWriteRead will be done at the end of this function.
    }

  } else if (function == PMAC_C_MotorRes_){
    pAxis->setResolution(value);
    // Direct resolution parameter will always match the raw motor
    setDoubleParam(pAxis->axisNo_, PMAC_C_DirectRes_, value);
  } else if (function == PMAC_C_MotorOffset_){
    pAxis->setOffset(value);
    // Direct offset parameter will always match the raw motor
    setDoubleParam(pAxis->axisNo_, PMAC_C_DirectOffset_, value);
  } else if (function == PMAC_C_DirectMove_){
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
  } else if (function == motorLowLimit_) {
    // Limits in counts
    int lowLimitCounts = int(std::round(value/pAxis->scale_));
    int highLimitCounts = int(std::round(pAxis->highLimit_/pAxis->scale_));
    // Check if requested limit is zero counts
    if (lowLimitCounts == 0) {
      // Check the other limit
      if (highLimitCounts == 0) {
        // Both limits are zero, so disable soft limits on PMAC
        sprintf(command, "I%d13=0 I%d14=0", pAxis->axisNo_, pAxis->axisNo_);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
          "%s: Setting both soft limits on controller %s, axis %d to 0 counts\n",
          functionName, portName, pAxis->axisNo_);
      }
      else {
        // Only one limit is zero, so set to 1 count to avoid disabling it
        sprintf(command, "I%d14=1", pAxis->axisNo_);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
          "%s: Setting low soft limit on controller %s, axis %d to 1 count\n",
          functionName, portName, pAxis->axisNo_);
      }
    }
    else {
      // Otherwise check if we also need to re-enable the other limit
      if (highLimitCounts == 0) {
        // Set low limit and re-enable the high limit by setting to 1 count
        sprintf(command, "I%d14=%d I%d13=1", pAxis->axisNo_, lowLimitCounts, pAxis->axisNo_);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
          "%s: Setting low, high soft limits on controller %s, axis %d to %d, 1 counts\n",
          functionName, portName, pAxis->axisNo_, lowLimitCounts);
      }
      else {
        // Just set low limit
        sprintf(command, "I%d14=%d", pAxis->axisNo_, lowLimitCounts);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s: Setting low soft limit on controller %s, axis %d to %d counts\n",
              functionName, portName, pAxis->axisNo_, lowLimitCounts);
      }
    }
    // Update limit on pmacAxis
    pAxis->lowLimit_ = value;
  } else if (function == motorHighLimit_) {
    // Limits in counts
    int lowLimitCounts = int(std::round(pAxis->lowLimit_/pAxis->scale_));
    int highLimitCounts = int(std::round(value/pAxis->scale_));
    // Check if requested limit is zero counts
    if (highLimitCounts == 0) {
      // Check the other limit
      if (lowLimitCounts == 0) {
        // Both limits are zero, so disable soft limits on PMAC
        sprintf(command, "I%d13=0 I%d14=0", pAxis->axisNo_, pAxis->axisNo_);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
          "%s: Setting both soft limits on controller %s, axis %d to 0 counts\n",
          functionName, portName, pAxis->axisNo_);
      }
      else {
        // Only one limit is zero, so set to 1 count to avoid disabling it
        sprintf(command, "I%d13=1", pAxis->axisNo_);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
          "%s: Setting high soft limit on controller %s, axis %d to 1 count\n",
          functionName, portName, pAxis->axisNo_);
      }
    }
    else {
      if (lowLimitCounts == 0) {
        // Set high limit and re-enable the low limit by setting to 1 count
        sprintf(command, "I%d13=%d I%d14=1", pAxis->axisNo_, highLimitCounts, pAxis->axisNo_);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
          "%s: Setting low, high soft limits on controller %s, axis %d to 1, %d counts\n",
          functionName, portName, pAxis->axisNo_, highLimitCounts);
      }
      else {
        // Just set the high limit
        sprintf(command, "I%d13=%d", pAxis->axisNo_, highLimitCounts);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
          "%s: Setting high soft limit on controller %s, axis %d to %d counts\n",
          functionName, portName, pAxis->axisNo_, highLimitCounts);
      }
    }
    // Update limit on pmacAxis
    pAxis->highLimit_ = value;
  } else if (pWriteParams_->hasKey(*name)) {
    // This is an integer write of a parameter, so send the immediate write/read
    sprintf(command, "%s=%.12f", pWriteParams_->lookup(*name).c_str(), value);
    debug(DEBUG_VARIABLE, functionName, "Command sent to PMAC", command);
    status = (this->immediateWriteRead(command, response) == asynSuccess) && status;
  }

  if (command[0] != 0 && status) {
    status = (lowLevelWriteRead(command, response) == asynSuccess) && status;
  }

  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = (asynMotorController::writeFloat64(pasynUser, value) == asynSuccess) && status;

  if (!status) {
    setIntegerParam(pAxis->axisNo_, this->motorStatusCommsError_, PMAC_ERROR_);
    return asynError;
  } else {
    setIntegerParam(pAxis->axisNo_, this->motorStatusCommsError_, PMAC_OK_);
  }

  return asynSuccess;

}

asynStatus
pmacController::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements) {
  asynStatus status = asynSuccess;
  int function = pasynUser->reason;
  static const char *functionName = "writeFloat64Array";
  debug(DEBUG_FLOW, functionName);

  if (!initialised_) {
    return asynSuccess;
  }

  if (!profileInitialized_) {
    // Initialise the trajectory scan interface pointers
    debug(DEBUG_TRACE, functionName, "Initialising CS trajectory scan interface");
    status = this->initializeProfile(PMAC_MAX_TRAJECTORY_POINTS);
  }

  if (status == asynSuccess) {
    profileInitialized_ = true;
    if (function == PMAC_C_ProfilePositionsA_) {
      memcpy(eguProfilePositions_[0], value, nElements * sizeof(double));
    } else if (function == PMAC_C_ProfilePositionsB_) {
      memcpy(eguProfilePositions_[1], value, nElements * sizeof(double));
    } else if (function == PMAC_C_ProfilePositionsC_) {
      memcpy(eguProfilePositions_[2], value, nElements * sizeof(double));
    } else if (function == PMAC_C_ProfilePositionsU_) {
      memcpy(eguProfilePositions_[3], value, nElements * sizeof(double));
    } else if (function == PMAC_C_ProfilePositionsV_) {
      memcpy(eguProfilePositions_[4], value, nElements * sizeof(double));
    } else if (function == PMAC_C_ProfilePositionsW_) {
      memcpy(eguProfilePositions_[5], value, nElements * sizeof(double));
    } else if (function == PMAC_C_ProfilePositionsX_) {
      memcpy(eguProfilePositions_[6], value, nElements * sizeof(double));
    } else if (function == PMAC_C_ProfilePositionsY_) {
      memcpy(eguProfilePositions_[7], value, nElements * sizeof(double));
    } else if (function == PMAC_C_ProfilePositionsZ_) {
      memcpy(eguProfilePositions_[8], value, nElements * sizeof(double));
    } else {
      status = asynMotorController::writeFloat64Array(pasynUser, value, nElements);
    }
  } else {
    debug(DEBUG_ERROR, functionName, "Failed to initialise trajectory scan interface");
  }

  return status;
}

/** Called when asyn clients call pasynInt32Array->write().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Pointer to the array to write.
  * \param[in] nElements Number of elements to write. */
asynStatus
pmacController::writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements) {
  asynStatus status = asynSuccess;
  int function = pasynUser->reason;
  static const char *functionName = "writeInt32Array";
  debug(DEBUG_FLOW, functionName);

  if (!initialised_) {
    return asynSuccess;
  }

  if (!profileInitialized_) {
    // Initialise the trajectory scan interface pointers
    debug(DEBUG_FLOW, functionName, "Initialising trajectory scan interface");
    status = this->initializeProfile(PMAC_MAX_TRAJECTORY_POINTS);
  }

  if (status == asynSuccess) {
    profileInitialized_ = true;
  } else {
    debug(DEBUG_ERROR, functionName, "Failed to initialise trajectory scan interface");
  }

  if (status == asynSuccess) {
    if (function == PMAC_C_ProfileUser_) {
      memcpy(profileUser_, value, nElements * sizeof(int));
    } else if (function == PMAC_C_ProfileVelMode_) {
      memcpy(profileVelMode_, value, nElements * sizeof(int));
    } else {
      status = asynMotorController::writeInt32Array(pasynUser, value, nElements);
    }
  }
  return status;
}

/** Called when asyn clients call pasynOctet->write().
  * This function performs actions for some parameters, including AttributesFile.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the string to write.
  * \param[in] nChars Number of characters to write.
  * \param[out] nActual Number of characters actually written. */
asynStatus
pmacController::writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual) {
  int addr = 0;
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  char command[PMAC_MAXBUF_] = {0};
  char response[PMAC_MAXBUF_] = {0};
  const char *functionName = "writeOctet";

  if (!initialised_) {
    return asynSuccess;
  }

  status = getAddress(pasynUser, &addr);
  if (status != asynSuccess) return (status);
  // Set the parameter in the parameter library.
  status = (asynStatus) setStringParam(addr, function, (char *) value);
  if (status != asynSuccess) return (status);

  if (function == PMAC_C_WriteCmd_) {
    // Write the arbitrary string to the PMAC, ignoring a reponse
    strcpy(command, value);
    status = this->immediateWriteRead(command, response);
  }

  // Do callbacks so higher layers see any changes
  callParamCallbacks(addr, addr);

  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = asynMotorController::writeOctet(pasynUser, value, nChars, nActual);

  if (status == asynSuccess) {
    if (function == PMAC_C_GroupAssign_) {
      // Force an immediate manual group update
      status = this->executeManualGroup();
    }
  }

  if (status != asynSuccess) {
    epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d, value=%s",
                  driverName, functionName, status, function, value);
  } else {
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%s\n",
              driverName, functionName, function, value);
  }
  *nActual = nChars;
  return status;
}

/**
 * Deal with controller specific epicsInt32 params.
 * @param pasynUser
 * @param value
 * @param asynStatus
 */
asynStatus pmacController::writeInt32(asynUser *pasynUser, epicsInt32 value) {
  int function = pasynUser->reason;
  char command[PMAC_MAXBUF_] = {0};
  char response[PMAC_MAXBUF_] = {0};
  bool status = true;
  pmacAxis *pAxis = NULL;
  const char *name[128];
  static const char *functionName = "writeInt32";

  debug(DEBUG_FLOW, functionName);

  if (!initialised_) {
    return asynSuccess;
  }

  getParamName(function, name);
  debug(DEBUG_VARIABLE, functionName, "Parameter Updated", *name);
  pAxis = this->getAxis(pasynUser);
  if (!pAxis) {
    return asynError;
  }

  status = (pAxis->setIntegerParam(function, value) == asynSuccess) && status;

  if (function == PMAC_C_PollAllNow_) {
    // force all three polls to fire now
    pollAllNow();
    // Reset the busy value to complete caput callback
    value = 0;
  } else if (function == PMAC_C_HomingStatus_) {
    if (value == 0) {
      // An auto home has just completed
      // make sure that pmacController->makeCSDemandsConsistent will reset the demand for all axes
      csResetAllDemands = true;
    }
  } else if (function == PMAC_C_StopAll_) {
    // Send the abort all command to the PMAC immediately
    status = (this->immediateWriteRead("\x01", response) == asynSuccess) && status;
    // Force all CS demands to refresh
    csResetAllDemands = true;
  } else if (function == PMAC_C_KillAll_) {
    // Send the kill all command to the PMAC immediately
    status = (this->immediateWriteRead("\x0b", response) == asynSuccess) && status;
    // Force all CS demands to refresh
    csResetAllDemands = true;
  } else if (function == PMAC_C_FeedRatePoll_) {
    if (value) {
      this->feedRatePoll_ = true;
    } else {
      this->feedRatePoll_ = false;
    }
  } else if (function == PMAC_C_MotorScale_) {
    pAxis->scale_ = value;
  } else if (function == PMAC_C_FeedRate_) {
    strcpy(command, "");
    for (int csNo = 1; csNo <= csCount; csNo++) {
      sprintf(command, "%s &%d%%%d", command, csNo, value);
    }
    debug(DEBUG_VARIABLE, functionName, "Feedrate Command", command);
    if (command[0] != 0) {
      //PMAC does not respond to this command.
      lowLevelWriteRead(command, response);
    }
  } else if (function == motorDeferMoves_) {
    debug(DEBUG_VARIABLE, functionName, "Motor defer value", value);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s: Setting deferred move mode on PMAC %s to %d\n", functionName, portName, value);
    if (value == 0) {
      status = (this->processDeferredMoves() == asynSuccess) && status;
    }
    this->movesDeferred_ = value;
  } else if (function == PMAC_C_CoordSysGroup_) {
    status = (pGroupList->switchToGroup(value) == asynSuccess) && status;
    updateCsAssignmentParameters();
    copyCsReadbackToDemand(false);
  } else if (pWriteParams_->hasKey(*name)) {
    // This is an integer write of a parameter, so send the immediate write/read
    sprintf(command, "%s=%d", pWriteParams_->lookup(*name).c_str(), value);
    debug(DEBUG_VARIABLE, functionName, "Command sent to PMAC", command);
    status = (this->immediateWriteRead(command, response) == asynSuccess) && status;
  } else if (function == PMAC_C_KillAxis_) {
    // call stop so that this kill can stop CS moves too
    // this is to get around unexpected behaviour that kill does not stop real axes
    // which are currently moving in a CS
    // removed this behaviour since it re-enables already killed axes in the same CS
    // pAxis->stop(0);
    // Send the kill command to the PMAC immediately
    sprintf(command, "#%dk", pAxis->axisNo_);
    status = (this->immediateWriteRead(command, response) == asynSuccess) && status;
  } else if (function == PMAC_C_ReportFast_) {
    status = (this->pBroker_->report(pmacMessageBroker::PMAC_FAST_READ) == asynSuccess) && status;
  } else if (function == PMAC_C_ReportMedium_) {
    status = (this->pBroker_->report(pmacMessageBroker::PMAC_MEDIUM_READ) == asynSuccess) && status;
  } else if (function == PMAC_C_ReportSlow_) {
    status = (this->pBroker_->report(pmacMessageBroker::PMAC_SLOW_READ) == asynSuccess) && status;
  } else if (function == PMAC_C_ProfileAppend_) {
    status = (this->appendToProfile() == asynSuccess) && status;
  } else if (function == PMAC_C_DebugCmd_) {
    // Read the level, axis number and CS number
    int level = 0;
    int axisNo = 0;
    int csNo = 0;
    getIntegerParam(PMAC_C_DebugLevel_, &level);
    getIntegerParam(PMAC_C_DebugAxis_, &axisNo);
    getIntegerParam(PMAC_C_DebugCS_, &csNo);
    this->setDebugLevel(level, axisNo, csNo);
  } else if (function == PMAC_C_DisablePolling_) {
    this->pBroker_->disable_poll = value;
  } else if (function == PMAC_C_GroupExecute_) {
    status = (this->executeManualGroup() == asynSuccess) && status;
  } else if (function == PMAC_C_GroupCSPort_) {
    status = (this->executeManualGroup() == asynSuccess) && status;
  }


  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = (asynMotorController::writeInt32(pasynUser, value) == asynSuccess) && status;

  if (!status) {
    setIntegerParam(pAxis->axisNo_, this->motorStatusCommsError_, PMAC_ERROR_);
    return asynError;
  } else {
    setIntegerParam(pAxis->axisNo_, this->motorStatusCommsError_, PMAC_OK_);
  }

  if (function == motorClosedLoop_) {
    bool closedLoop = (value == 0) ? 0 : 1;

    status = pAxis->setClosedLoop(closedLoop);
  }

  if (!status) {
    setIntegerParam(pAxis->axisNo_, this->motorStatusCommsError_, PMAC_ERROR_);
    return asynError;
  } else {
    setIntegerParam(pAxis->axisNo_, this->motorStatusCommsError_, PMAC_OK_);
  }

  return asynSuccess;

}

asynStatus
pmacController::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],
                         size_t nElements, size_t *nIn) {
  asynStatus status = asynSuccess;
  int function = pasynUser->reason;
  size_t index;
  size_t enumIndex;
  static const char *functionName = "readEnum";

  if (function == PMAC_C_TrajCSPort_ || function == PMAC_C_GroupCSPort_ ||
      function == PMAC_C_GroupCSPortRBV_) {
    enumIndex = 0;
    // Loop over all coordinate systems checking for valid CS
    for (index = 0; index < PMAC_MAX_CS; index++) {
      if (index == 0 && (function == PMAC_C_GroupCSPort_ || function == PMAC_C_GroupCSPortRBV_)) {
        if (function == PMAC_C_GroupCSPort_) {
          strings[enumIndex] = epicsStrDup("None");
        } else {
          strings[enumIndex] = epicsStrDup("");
        }
        values[enumIndex] = index;
        severities[enumIndex] = 0;
        enumIndex++;
      } else {
        if (pCSControllers_[index] != NULL) {
          if (strings[enumIndex]) {
            free(strings[enumIndex]);
          }
          strings[enumIndex] = epicsStrDup(pCSControllers_[index]->getPortName().c_str());
          debug(DEBUG_VARIABLE, functionName, "Reading CS port", strings[enumIndex]);
          values[enumIndex] = index;
          severities[enumIndex] = 0;
          enumIndex++;
        }
      }
    }
    *nIn = enumIndex;
  } else {
    *nIn = 0;
    status = asynError;
  }
  return status;
}

/** Returns a pointer to an pmacAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
pmacAxis *pmacController::getAxis(asynUser *pasynUser) {
  int axisNo = 0;

  getAddress(pasynUser, &axisNo);
  return getAxis(axisNo);
}


/** Returns a pointer to an pmacAxis object.
  * Returns NULL if the axis number is invalid.
  * \param[in] axisNo Axis index number. */
pmacAxis *pmacController::getAxis(int axisNo) {
  if ((axisNo < 0) || (axisNo >= numAxes_)) return NULL;
  return pAxes_[axisNo];
}


/**
 * Polls the controller, rather than individual axis.
 * @return asynStatus
 */
asynStatus pmacController::poll() {
  char tBuff[32];
  static const char *functionName = "poll";
  debug(DEBUG_FLOW, functionName);
  epicsTimeGetCurrent(&nowTime_);

  // First check the connection
  this->checkConnection();

  if (connected_ != 0 && initialised_ != 0) {
    // Always call for a fast update
    epicsTimeToStrftime(tBuff, 32, "%Y/%m/%d %H:%M:%S.%03f", &nowTime_);
    debug(DEBUG_TIMING, functionName, "Fast update has been called", tBuff);
    pBroker_->updateVariables(pmacMessageBroker::PMAC_FAST_READ);
    this->updateStatistics();
    setDoubleParam(PMAC_C_FastUpdateTime_, pBroker_->readUpdateTime());
    if (epicsTimeDiffInSeconds(&nowTime_, &lastMediumTime_) >= PMAC_MEDIUM_LOOP_TIME / 1000.0) {
      epicsTimeAddSeconds(&lastMediumTime_, PMAC_MEDIUM_LOOP_TIME / 1000.0);
      debug(DEBUG_TIMING, functionName, "Medium update has been called", tBuff);
      // Check if we are connected
      if (connected_ != 0 && initialised_ != 0) {
        pBroker_->updateVariables(pmacMessageBroker::PMAC_MEDIUM_READ);
      }
    }
    if (epicsTimeDiffInSeconds(&nowTime_, &lastSlowTime_) >= PMAC_SLOW_LOOP_TIME / 1000.0) {
      epicsTimeAddSeconds(&lastSlowTime_, PMAC_SLOW_LOOP_TIME / 1000.0);
      debug(DEBUG_TIMING, functionName, "Slow update has been called", tBuff);
      // Check if we are connected
      if (connected_ != 0 && initialised_ != 0) {
        pBroker_->updateVariables(pmacMessageBroker::PMAC_SLOW_READ);
      }
    }
  } else {
    // When there is no connection, set the problem flag
    lock();
    setIntegerParam(this->PMAC_C_GlobalStatus_, true);
    callParamCallbacks();
    unlock();
  }

  return asynSuccess;
}

asynStatus pmacController::initializeProfile(size_t maxPoints) {
  static const char *functionName = "initializeProfile";

  debug(DEBUG_FLOW, functionName);
  debug(DEBUG_VARIABLE, functionName, "maxPoints", (int) maxPoints);

  // Allocate the pointers
  tScanPositions_ = (double **) malloc(sizeof(double *) * PMAC_MAX_CS_AXES);
  // Now allocate each position array
  for (int axis = 0; axis < PMAC_MAX_CS_AXES; axis++) {
    tScanPositions_[axis] = (double *) malloc(sizeof(double) * maxPoints);
  }

  // Allocate the pointers
  eguProfilePositions_ = (double **) malloc(sizeof(double *) * PMAC_MAX_CS_AXES);
  // Now allocate each position array
  for (int axis = 0; axis < PMAC_MAX_CS_AXES; axis++) {
    eguProfilePositions_[axis] = (double *) malloc(sizeof(double) * maxPoints);
  }

  // Allocate memory required for user buffer
  if (profileUser_) {
    free(profileUser_);
  }
  profileUser_ = (int *) malloc(sizeof(int) * maxPoints);
  // Allocate memory required for velocity mode buffer
  if (profileVelMode_) {
    free(profileVelMode_);
  }
  profileVelMode_ = (int *) malloc(sizeof(int) * maxPoints);

  // Finally call super class
  return asynMotorController::initializeProfile(maxPoints);
}

asynStatus pmacController::buildProfile() {
  asynStatus status = asynSuccess;
  int csNo = 0;
  int csPort = 0;
  std::string csPortName;
  static const char *functionName = "buildProfile";

  debug(DEBUG_FLOW, functionName);

  // Read the port name for CS to execute
  getIntegerParam(PMAC_C_TrajCSPort_, &csPort);
  debug(DEBUG_VARIABLE, functionName, "csPort", csPort);
  csPortName = pCSControllers_[csPort]->getPortName();
  debug(DEBUG_VARIABLE, functionName, "csPortName", csPortName);

  // Check the CS port name against a CS number
  if (csPortName != "") {
    if (pPortToCs_->hasKey(csPortName.c_str())) {
      // Lookup the current CS number for this axis
      csNo = pPortToCs_->lookup(csPortName.c_str());
      // If the axis has an assigned CS no then we will use it
      if (csNo != 0) {
        // Execute the build for the specified CS
        status = this->buildProfile(csNo);
      } else {
        debug(DEBUG_ERROR, functionName, "Invalid Coordinate System specified");
        // Set the status to failure
        this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                             "Invalid Coordinate System specified");
        status = asynError;
      }
    } else {
      debug(DEBUG_ERROR, functionName, "Invalid Coordinate System specified");
      // Set the status to failure
      this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                           "Invalid Coordinate System specified");
      status = asynError;
    }
  } else {
    debug(DEBUG_ERROR, functionName, "No Coordinate System specified");
    // Set the status to failure
    this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                         "No Coordinate System specified");
    status = asynError;
  }

  return status;
}

asynStatus pmacController::buildProfile(int csNo) {
  asynStatus status = asynSuccess;
  int numPoints = 0;
  int numPointsToBuild = 0;
  int axisMask = 0;
  int counter = 0;
  static const char *functionName = "buildProfile";

  debug(DEBUG_FLOW, functionName);

  // Set the status to building
  this->setBuildStatus(PROFILE_BUILD_BUSY, PROFILE_STATUS_UNDEFINED, "Building profile");
  callParamCallbacks();

  // First check to see if we need to initialise memory
  if (!profileInitialized_) {
    // Initialise the trajectory scan interface pointers
    status = this->initializeProfile(PMAC_MAX_TRAJECTORY_POINTS);
    if (status == asynSuccess) {
      profileInitialized_ = true;
    } else {
      debug(DEBUG_ERROR, functionName, "Failed to allocate memory on controller");
      // Set the status to failure
      this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                           "Failed to allocate memory on controller");
    }
  }

  // Important to check the type of hardware and the memory locations
  // specified to hold the trajectory scan data
  if (status == asynSuccess) {
    if (!strcmp(cpu_.c_str(), PMAC_CPU_GEO_240MHZ)) {
      // New Geobrick with additional memory.
      // Check memory addresses are greater than 0x30000
      if (tScanPmacBufferAddressA_ < 0x30000) {
        // Set the status to failure
        this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                             "Buffer A memory address invalid");
        status = asynError;
      }
      if (tScanPmacBufferAddressB_ < 0x30000) {
        // Set the status to failure
        this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                             "Buffer B memory address invalid");
        status = asynError;
      }
      if (tScanPmacBufferAddressA_ == tScanPmacBufferAddressB_) {
        // Set the status to failure
        this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                             "Buffer memory addresses invalid");
        status = asynError;
      }
    } else {
      // Old Geobrick without additional memory.
      // Check memory addresses are less than of equal to (0x10800-18*buffer_size)
      if (tScanPmacBufferAddressA_ > (0x10800 - (10 * tScanPmacBufferSize_))) {
        // Set the status to failure
        this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                             "Buffer A memory address invalid");
        status = asynError;
      }
      if (tScanPmacBufferAddressB_ > (0x10800 - (10 * tScanPmacBufferSize_))) {
        // Set the status to failure
        this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                             "Buffer B memory address invalid");
        status = asynError;
      }
      if (tScanPmacBufferAddressA_ == tScanPmacBufferAddressB_) {
        // Set the status to failure
        this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                             "Buffer memory addresses invalid");
        status = asynError;
      }
    }
  }

  // Check the version numbers are matching
  // Assuming a newer Trajectory PMC version against older pmac driver is OK
  if (fabs(tScanPmacProgVersion_ - PMAC_TRAJECTORY_VERSION) > 0.0001) {
    debug(DEBUG_ERROR, functionName, "Motion program and driver versions do not match");
    debug(DEBUG_ERROR, functionName, "Motion program version", tScanPmacProgVersion_);
    debug(DEBUG_ERROR, functionName, "Driver version", PMAC_TRAJECTORY_VERSION);
    // Set the status to failure
    this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                         "Program and driver version mismatch");
    status = asynError;
  }

  // Check the CS number is valid
  if (csNo > PMAC_MAX_CS) {
    debug(DEBUG_ERROR, functionName, "Invalid CS number", csNo);
    // Set the status to failure
    this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                         "Invalid CS number requested build");
    status = asynError;
  }
  if (status == asynSuccess) {
    if (pCSControllers_[csNo] == NULL) {
      debug(DEBUG_ERROR, functionName, "Invalid CS number", csNo);
      // Set the status to failure
      this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                           "Invalid CS number requested build");
      status = asynError;
    }
  }

  // Verify that we are not currently executing a trajectory scan
  if (status == asynSuccess) {
    if (tScanExecuting_ > 0) {
      // Set the status to failure
      this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE, "Scan is already executing");
      status = asynError;
    }
  }

  // Compile the required arrays for time, position, user
  if (status == asynSuccess) {
    // Set the current scan CS
    tScanCSNo_ = csNo;
    debug(DEBUG_VARIABLE, functionName, "Current scan CS", tScanCSNo_);
    // Read the maximum number of points in the scan
    getIntegerParam(profileNumPoints_, &numPoints);
    // Read in the number of points ready for building
    getIntegerParam(PMAC_C_ProfileNumBuild_, &numPointsToBuild);

    // Check for any invalid times
    int maxValue = 0xFFFFFF;
    if (pvtTimeMode_ == 0) {
      // In this mode the maximum time is 4095 ms
      maxValue = 0x3E7C18;
    }
    while (counter < numPointsToBuild) {
      // Profile times must be less than 24bit
      if (profileTimes_[counter] > maxValue) {
        char errMsg[128];
        sprintf(errMsg, "Invalid profile time value (> %d microseconds)", maxValue);
        this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE, errMsg);
        status = asynError;
      }
      counter++;
    }

    if (status == asynSuccess) {
      // Ask the CS controller for the bitmap of axes that are to be included in the scan
      // 1 to 9 axes (0 is error) 111111111 => 1 .. 511
      status = this->tScanIncludedAxes(&axisMask);
      tScanAxisMask_ = axisMask;
      //Check if each axis from the coordinate system is involved in this trajectory scan
      for (int index = 0; index < PMAC_MAX_CS_AXES; index++) {
        if ((1 << index & axisMask) > 0) {
          if (status == asynSuccess) {
            // If the axis is going to be included then copy the position array into local
            // storage ready for the trajectory execution
            status = this->tScanBuildProfileArray(tScanPositions_[index], index, numPointsToBuild);
            if (status != asynSuccess) {
              // Set the status to failure
              this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                                   "Failed to build profile positions");
            }
          }
        }
      }
    }
  }

  if (status == asynSuccess) {
    // Initialise the trajectory store
    status = pTrajectory_->initialise(numPoints);

    if (status == asynSuccess) {
      // Set the trajectory store initial values
      status = pTrajectory_->append(tScanPositions_, profileTimes_, profileUser_, profileVelMode_,
                                    numPointsToBuild);
      setIntegerParam(PMAC_C_ProfileBuiltPoints_, pTrajectory_->getNoOfValidPoints());
      // Set the scan size to be equal to the number of built points
      tScanNumPoints_ = pTrajectory_->getNoOfValidPoints();

      if (status != asynSuccess) {
        // Set the status to failure
        this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                             "Failed to build trajectory object");
      }
    }
  }

  if (status == asynSuccess) {
    // If all checks have passed so far then send the relevant starting params
    // to the PMAC and fill the first half buffer
    status = preparePMAC();
    if (status != asynSuccess) {
      // Set the status to failure
      this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                           "Failed to write enough points, check quantities");
    }
  }

  // Finally if the profile build has completed then set the status accordingly
  if (status == asynSuccess) {
    // Set the maximum number of points in the scan
    setIntegerParam(profileNumPoints_, numPoints);
    // Set the status to complete
    this->setBuildStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_SUCCESS, "Profile built");
    // Reset any append status
    this->setAppendStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_SUCCESS, "");
    // Set the internal built flag
    profileBuilt_ = true;
    // Set the append flag to true
    appendAvailable_ = true;
  } else {
    // Zero the number of points in the scan
    setIntegerParam(profileNumPoints_, 0);
  }
  callParamCallbacks();

  return status;
}

asynStatus pmacController::appendToProfile() {
  asynStatus status = asynSuccess;
  int numPointsToBuild = 0;
  static const char *functionName = "appendToProfile";

  debug(DEBUG_FLOW, functionName);

  // Set the status to busy
  this->setAppendStatus(PROFILE_BUILD_BUSY, PROFILE_STATUS_SUCCESS,
                        "Appending points to trajectory");
  if (appendAvailable_) {
    // Read in the number of points to append
    getIntegerParam(PMAC_C_ProfileNumBuild_, &numPointsToBuild);
    //Check if each axis from the coordinate system is involved in this trajectory scan
    for (int index = 0; index < PMAC_MAX_CS_AXES; index++) {
      if ((1 << index & tScanAxisMask_) > 0) {
        if (status == asynSuccess) {
          // If the axis is going to be included then copy the position array into local
          // storage ready for the trajectory execution
          status = this->tScanBuildProfileArray(tScanPositions_[index], index, numPointsToBuild);
          if (status != asynSuccess) {
            // Set the status to failure
            this->setAppendStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                                  "Failed to compile profile positions");
          }
        }
      }
    }
    // Append the points to the store
    status = pTrajectory_->append(tScanPositions_, profileTimes_, profileUser_, profileVelMode_,
                                  numPointsToBuild);
    setIntegerParam(PMAC_C_ProfileBuiltPoints_, pTrajectory_->getNoOfValidPoints());
    // Set the scan size to be equal to the number of built points
    tScanNumPoints_ = pTrajectory_->getNoOfValidPoints();

    if (status != asynSuccess) {
      // Set the status to failure
      this->setAppendStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                            "Failed to append points to trajectory object");
    } else {
      // Set the status to success
      char msg[512];
      sprintf(msg, "Appended %d points to the trajectory", numPointsToBuild);
      this->setAppendStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_SUCCESS, msg);
    }
  } else {
    this->setAppendStatus(PROFILE_BUILD_DONE, PROFILE_STATUS_FAILURE,
                          "Cannot append points, is the scan built?");
    status = asynError;
  }

  return status;
}

asynStatus pmacController::preparePMAC() {
  asynStatus status = asynSuccess;
  int axisMask = 0;
  char response[1024];
  char cmd[1024];
  const char *functionName = "preparePMAC";

  debug(DEBUG_FLOW, functionName);

  // Set the trajectory CS number
  setIntegerParam(PMAC_C_TrajCSNumber_, tScanCSNo_);

  // Reset the scan point counter
  tScanPointCtr_ = 0;

  // Send the initial half buffer of position updates
  // (Always buffer 0 to start)
//  this->lock();
  status = this->sendTrajectoryDemands(0);
//  this->unlock();

  if (status == asynSuccess) {
    // Calculate the axis mask ready to send to the PMAC
    // Z => 256
    // Y => 128
    // X => 64
    // W => 32
    // V => 16
    // U => 8
    // C => 4
    // B => 2
    // A => 1
    for (int index = 0; index < PMAC_MAX_CS_AXES; index++) {
      if ((1 << index & tScanAxisMask_) > 0) {
        // Bits swapped from EPICS axis mask
//        axisMask += (1 << (8 - index));
        axisMask += (1 << index);
      }
    }
    sprintf(cmd, "%s=%d", PMAC_TRAJ_AXES, axisMask);
    debug(DEBUG_VARIABLE, functionName, "Axis mask to send to PMAC (P4003)", axisMask);
    status = this->immediateWriteRead(cmd, response);
  }

  if (status == asynSuccess) {
    // Re-initialise the values used by the motion program
    // Set buffer B position to 0
    sprintf(cmd, "%s=0 %s=0", PMAC_TRAJ_BUFF_FILL_B, PMAC_TRAJ_TOTAL_POINTS);
    status = this->immediateWriteRead(cmd, response);
    setIntegerParam(PMAC_C_TrajBuffFillB_, 0);
  }

  return status;
}

asynStatus pmacController::executeProfile() {
  asynStatus status = asynSuccess;
  int csNo = 0;
  int csPort = 0;
  std::string csPortName;
  static const char *functionName = "executeProfile";

  debug(DEBUG_FLOW, functionName);

  // Read the port name for CS to execute
  getIntegerParam(PMAC_C_TrajCSPort_, &csPort);
  debug(DEBUG_VARIABLE, functionName, "csPort", csPort);
  csPortName = pCSControllers_[csPort]->getPortName();
  debug(DEBUG_VARIABLE, functionName, "csPortName", csPortName);

  // Check the CS port name against a CS number
  if (csPortName != "") {
    if (pPortToCs_->hasKey(csPortName.c_str())) {
      // Lookup the current CS number for this axis
      csNo = pPortToCs_->lookup(csPortName.c_str());
      // If the axis has an assigned CS no then we will use it
      if (csNo != 0) {
        // Execute the trajectory scan for the CSthe port name for CS to execute
        status = this->executeProfile(csNo);
      } else {
        debug(DEBUG_ERROR, functionName, "Invalid Coordinate System specified");
        // Set the status to failure
        this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE,
                               "Invalid Coordinate System specified");
        status = asynError;
      }
    } else {
      debug(DEBUG_ERROR, functionName, "Invalid Coordinate System specified");
      // Set the status to failure
      this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE,
                             "Invalid Coordinate System specified");
      status = asynError;
    }
  } else {
    debug(DEBUG_ERROR, functionName, "No Coordinate System specified");
    // Set the status to failure
    this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE,
                           "No Coordinate System specified");
    status = asynError;
  }
  return status;
}

asynStatus pmacController::executeProfile(int csNo) {
  asynStatus status = asynSuccess;
  int buildState = 0;
  int buildStatus = 0;
  static const char *functionName = "executeProfile";

  debug(DEBUG_FLOW, functionName);

  // Set the status to starting
  this->setProfileStatus(PROFILE_EXECUTE_MOVE_START, PROFILE_STATUS_UNDEFINED,
                         "Starting trajectory execution");
  callParamCallbacks();

  // Check that the profile has been built successfully
  getIntegerParam(profileBuildState_, &buildState);
  getIntegerParam(profileBuildStatus_, &buildStatus);
  if ((buildStatus != PROFILE_STATUS_SUCCESS) || (buildState != PROFILE_BUILD_DONE) ||
      (!profileBuilt_)) {
    // Set the status to failure
    this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE,
                           "Trajectory profile was not built successfully");
    debug(DEBUG_ERROR, functionName, "Trajectory was not built successfully");
    status = asynError;
    callParamCallbacks();
  }

  // Check that the same CS that built the profile has asked to execute
  // the trajectory scan
  if (status == asynSuccess) {
    if (csNo != tScanCSNo_) {
      // Set the status to failure
      this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE,
                             "Build and execute called by different CS");
      debugf(DEBUG_ERROR, functionName, "Profile built by CS %d but execute was called by CS %d",
             tScanCSNo_, csNo);
      status = asynError;
      callParamCallbacks();
    }
  }

  // Verify that we are not currently executing a trajectory scan
  if (status == asynSuccess) {
    if (tScanExecuting_ > 0) {
      // Set the status to failure
      this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE,
                             "Scan is already executing");
      status = asynError;
      callParamCallbacks();
    }
  }

  // Ensure that the demands have been made consistent for the CS axes
  if (status == asynSuccess) {
    makeCSDemandsConsistent();
  }

  if (status == asynSuccess) {
    // Checks have passed.
    // Reset the built flag so that we cannot re-run the scan without re-building first.
    // This is necessary due to the build writing values into the PMAC hardware, we cannot
    // be sure that executing twice will produce a valid scan on the second iteration so we
    // must force a re-build of the scan each time.
    profileBuilt_ = false;
    // Send the signal to the trajectory thread
    epicsEventSignal(this->startEventId_);
  }
  return status;
}

asynStatus pmacController::abortProfile() {
  asynStatus status = asynSuccess;
  char cmd[1024];
  char response[1024];
  int progRunning = 1;
  const char *functionName = "abortProfile";

  debug(DEBUG_FLOW, functionName);

  // Send an immediate abort signal to the PMAC
  sprintf(cmd, "%s=1", PMAC_TRAJ_ABORT);
  status = this->immediateWriteRead(cmd, response);

  // Set the status P variable to idle in case the motion program cannot
  sprintf(cmd, "%s=%d", PMAC_TRAJ_STATUS, PMAC_TRAJ_STATUS_FINISHED);
  status = this->immediateWriteRead(cmd, response);

  // Set the scan executing variable to 0
  tScanExecuting_ = 0;

  // Send the signal to the trajectory thread
  epicsEventSignal(this->stopEventId_);

  // Check CS number is not zero
  if (tScanCSNo_ != 0) {
    // Now wait for the trajectory scan axes to stop
    while (progRunning == 1) {
      pCSControllers_[tScanCSNo_]->tScanCheckProgramRunning(&progRunning);
      if (progRunning == 1) {
        // Check again in 100ms
        this->unlock();
        epicsThreadSleep(0.1);
        this->lock();
      }
    }
  }

  // Set the status to aborted
  setIntegerParam(profileAbort_, 0);
  this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_ABORT, "Trajectory scan aborted");
  callParamCallbacks();

  return status;
}

void pmacController::trajectoryTask() {
  epicsTimeStamp startTime, endTime;
  double elapsedTime;
  int epicsErrorDetect = 0;
  int epicsBufferNumber = 0;
  int progRunning = 0;
  int progNo = 0;
  int totalProfilePoints = 0;
  //double position = 0.0;
  char response[1024];
  char cmd[1024];
  char msg[1024];
  const char *functionName = "trajectoryTask";

  this->lock();
  // Loop forever
#ifdef __clang__
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif
  while (true) {
    // If we are not scanning then wait for a semaphore that is given when a scan is started
    if (!tScanExecuting_) {
      // Set the append flag to false
      appendAvailable_ = false;
      // Reset any of our own errors
      epicsErrorDetect = 0;
      // Reset the execute parameter for caput callback
      setIntegerParam(profileExecute_, 0);
      callParamCallbacks();
      // Release the lock while we wait for an event that says scan has started, then lock again
      debug(DEBUG_TRACE, functionName, "Waiting for scan to start");
      this->unlock();
      epicsEventWait(this->startEventId_);
      this->lock();
      tScanExecuting_ = 1;

      debug(DEBUG_TRACE, functionName, "Trajectory scan started");

      // Reset the buffer number
      epicsBufferNumber = 0;

      // Record the scan start time
      epicsTimeGetCurrent(&startTime);

      // Make sure axes are enabled
      this->immediateWriteRead(pHardware_->getCSEnableCommand(tScanCSNo_).c_str(), response);

      if (response[0] == 0x7) {
        // Remove the line feed
        response[strlen(response) - 1] = 0;
        // Set the status to failure
        char msg[1024];
        sprintf(msg, "Scan failed to enable axes with %s", response + 1);
        this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE, msg);
        // Set the executing flag to 0
        tScanExecuting_ = 0;
        // Notify that EPICS detected the error
        epicsErrorDetect = 1;
      }

      // We are ready to execute the start demand
      // Only execute this if there has been no error detected so far
      if (epicsErrorDetect == 0) {
        // Read the total number of points within the scan
        totalProfilePoints = pTrajectory_->getNoOfValidPoints();
        getIntegerParam(PMAC_C_TrajProg_, &progNo);
        sprintf(cmd, "%s=%d", PMAC_TRAJ_STATUS, PMAC_TRAJ_STATUS_RUNNING);
        this->immediateWriteRead(cmd, response);
        sprintf(cmd, "&%dB%dR", tScanCSNo_, progNo);
        this->immediateWriteRead(cmd, response);
        // Check if this command returned an error
        if (response[0] == 0x7) {
          // Remove the line feed
          response[strlen(response) - 1] = 0;
          // Set the status to failure
          char msg[1024];
          sprintf(msg, "Scan failed to start motion program with %s", response + 1);
          this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE, msg);
          // Set the executing flag to 0
          tScanExecuting_ = 0;
          // Notify that EPICS detected the error
          epicsErrorDetect = 1;
        } else {
          // Force a poll
          this->poll();
          // Now wake up the fast polling
          this->wakeupPoller();
          // Reset our status
          setIntegerParam(PMAC_C_TrajEStatus_, 0);
          // Set the status to executing
          this->setProfileStatus(PROFILE_EXECUTE_EXECUTING, PROFILE_STATUS_SUCCESS,
                                 "Executing trajectory scan");
        }
      }
      callParamCallbacks();
    }

    // Now entering the main loop during a scan.  If the EPICS driver has detected
    // a problem then do not execute any of this code
    if (epicsErrorDetect == 0) {
      // Read the total number of points within the scan
      totalProfilePoints = pTrajectory_->getNoOfValidPoints();
      // Check if the reported PMAC buffer number is the same as the EPICS buffer number
      if (tScanPmacBufferNumber_ == epicsBufferNumber) {
        debug(DEBUG_TRACE, functionName, "Reading from buffer", tScanPmacBufferNumber_);
        debug(DEBUG_TRACE, functionName, "Send next demand set to PMAC");
        if (epicsBufferNumber == 0) {
          epicsBufferNumber = 1;
        } else {
          epicsBufferNumber = 0;
        }
        // EPICS buffer number has just been updated, so fill the next
        // half buffer with positions
        this->unlock();
        this->sendTrajectoryDemands(epicsBufferNumber);
        this->lock();
      }

      // Record the current scan time
      epicsTimeGetCurrent(&endTime);
      // Work out the elapsed time of the scan
      elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
      setDoubleParam(PMAC_C_TrajRunTime_, elapsedTime);

      // Check if the scan has stopped/finished (status from PMAC)
      // Only if we still think the scan is running (i.e. we didn't abort it
      if (tScanExecuting_ == 1 && tScanPmacStatus_ != PMAC_TRAJ_STATUS_RUNNING) {
        debug(DEBUG_VARIABLE, functionName, "tScanPmacStatus", tScanPmacStatus_);
        // Something has happened on the PMAC side
        tScanExecuting_ = 0;
        if (tScanPmacStatus_ == PMAC_TRAJ_STATUS_FINISHED) {
          // Test the number of points scanned to ensure the motion program has actually
          // completed and not just caught up with the buffer edge
          if (totalProfilePoints == tScanPmacTotalPts_) {
            // Set the status to complete
            this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_SUCCESS,
                                   "Trajectory scan complete");
          } else {
            // Set the status to failure
            sprintf(msg,"Scan failed, unable to fill buffers in time (%d/%d)",
                    tScanPmacTotalPts_, totalProfilePoints);
            this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE, msg);
          }
        } else {
          // Set the status to failure
          this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE,
                                 "Trajectory scan failed, motion program timeout start");
        }
        callParamCallbacks();
      }

      // Check here if the scan status reported by the PMAC is running but
      // the motion program is not running, this points to some other error
      // (possibly motor in limit)
      if (tScanPmacStatus_ == PMAC_TRAJ_STATUS_RUNNING) {
        if (pCSControllers_[tScanCSNo_]->tScanCheckProgramRunning(&progRunning) == asynSuccess) {
          if (progRunning == 0) {
            std::stringstream ss;
            ss << "Scan failed: ";
            for (int axis = 1; axis <= numAxes_; axis++) {
              // Check for any motors in limits
              if (this->getAxis(axis) != NULL) {
                int csNo = this->getAxis(axis)->getAxisCSNo();
                if (csNo == tScanCSNo_){
                  axisStatus mStatus = this->getAxis(axis)->getMotorStatus();
                  if (mStatus.highLimit_ == 1){
                    ss << "M" << axis << " Lim(HIGH) ";
                    std::stringstream em;
                    em << "Trajectory scan failed: Motor " << axis << " high limit activated";
                    debug(DEBUG_ERROR, functionName, em.str());
                  }
                  if (mStatus.lowLimit_ == 1){
                    ss << "M" << axis << " Lim(LOW) ";
                    std::stringstream em;
                    em << "Trajectory scan failed: Motor " << axis << " low limit activated";
                    debug(DEBUG_ERROR, functionName, em.str());
                  }
                  if (mStatus.ampEnabled_ == 0){
                    ss << "M" << axis << " AmpEna(OFF) ";
                    std::stringstream em;
                    em << "Trajectory scan failed: Motor " << axis << " amplifier disabled (could have been killed)";
                    debug(DEBUG_ERROR, functionName, em.str());
                  }
                  if (mStatus.followingError_ == 1){
                    ss << "M" << axis << " FFE(ON) ";
                    std::stringstream em;
                    em << "Trajectory scan failed: Motor " << axis << " Fatal Following Error";
                    debug(DEBUG_ERROR, functionName, em.str());
                  }
                }
              }
            }
            if (ss.str() == "Scan failed: "){
              // We have been unable to determine why the scan has failed, log this
              ss << "No motor problems - checking CS";
            }
            // Program not running but it should be
            tScanExecuting_ = 0;
            // Set the status to failure
            this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE, ss.str().c_str());
          }
        }
      }

      // Here we need to check for CS errors that would abort the scan
      if (pCSControllers_[tScanCSNo_]->tScanCheckForErrors() != asynSuccess) {
        // There has been a CS error reported.  Abort the scan
        tScanExecuting_ = 0;
        // Set the status to 1 here, error detected
        setIntegerParam(PMAC_C_TrajEStatus_, 1);
        // Set the status to failure
        this->setProfileStatus(PROFILE_EXECUTE_DONE, PROFILE_STATUS_FAILURE,
                               pCSControllers_[tScanCSNo_]->tScanGetErrorMessage());

        callParamCallbacks();
      }

      // If we are scanning then sleep for a short period before checking the status
      if (tScanExecuting_) {
        debug(DEBUG_FLOW, functionName, "Trajectory scan waiting");
        this->unlock();
        epicsEventWaitWithTimeout(this->stopEventId_, 0.1);
        this->lock();
      }
    }
  }
#ifdef __clang__
  #pragma clang diagnostic pop
#endif
}

void pmacController::setBuildStatus(int state, int status, const std::string &message) {
  // Set the build state
  setIntegerParam(profileBuildState_, state);
  // Set the build status
  setIntegerParam(profileBuildStatus_, status);
  // Set the status message
  setStringParam(profileBuildMessage_, message.c_str());
  // Call callbacks
  callParamCallbacks();
}

void pmacController::setAppendStatus(int state, int status, const std::string &message) {
  // Set the append state
  setIntegerParam(PMAC_C_ProfileAppendState_, state);
  // Set the append status
  setIntegerParam(PMAC_C_ProfileAppendStatus_, status);
  // Set the append message
  setStringParam(PMAC_C_ProfileAppendMessage_, message.c_str());
  // Call callbacks
  callParamCallbacks();
}

void pmacController::setProfileStatus(int state, int status, const std::string &message) {
  // Set the execute state
  setIntegerParam(profileExecuteState_, state);
  // Set the execute status
  setIntegerParam(profileExecuteStatus_, status);
  // Set the status message
  setStringParam(profileExecuteMessage_, message.c_str());
  // Call callbacks
  callParamCallbacks();
}

asynStatus pmacController::sendTrajectoryDemands(int buffer) {
  asynStatus status = asynSuccess;
  int nAxes = 0;
  int nBuffers = 0;
  int epicsBufferPtr = 0;
  int writeAddress = 0;
  int velModeValue = 0;
  double posValue = 0.0;
  int userValue = 0;
  int timeValue = 0;
  char response[1024];
  char cstr[1024];
  const char *functionName = "sendTrajectoryDemands";
  bool firstVal = true;

  debug(DEBUG_FLOW, functionName);
  startTimer(DEBUG_TIMING, functionName);

  // Calculate how many axes are included in this trajectory scan
  nAxes = 0;
  for (int index = 0; index < PMAC_MAX_CS_AXES; index++) {
    if ((1 << index & tScanAxisMask_) > 0) {
      nAxes++;
    }
  }

  // How many points can be written into a single message
  nBuffers = PMAC_POINTS_PER_WRITE;


  debug(DEBUG_VARIABLE, functionName, "tScanPmacBufferSize_", tScanPmacBufferSize_);
  debug(DEBUG_VARIABLE, functionName, "tScanPointCtr_", tScanPointCtr_);
  debug(DEBUG_VARIABLE, functionName, "tScanNumPoints_", tScanNumPoints_);

  // Supress the status reading within the message broker
  pBroker_->supressStatusReads();

  // Check the number of points we have, if greater than the buffer size
  // then fill the buffer, else fill up to the number of points
  while (epicsBufferPtr < tScanPmacBufferSize_ && tScanPointCtr_ < tScanNumPoints_ &&
         status == asynSuccess) {
    // Set the address of the write according to the half buffer
    if (buffer == PMAC_TRAJ_BUFFER_A) {
      writeAddress = tScanPmacBufferAddressA_;
    } else if (buffer == PMAC_TRAJ_BUFFER_B) {
      writeAddress = tScanPmacBufferAddressB_;
    } else {
      debug(DEBUG_ERROR, functionName, "Out of range buffer pointer", buffer);
      status = asynError;
    }
    // Offset the write address by the epics buffer pointer
    writeAddress += epicsBufferPtr;

    // Count how many buffers to fill
    char cmd[12][1024];
    // cmd[9,10,11] are reserved for the time, velocity, user values
    pHardware_->startTrajectoryTimePointsCmd(cmd[9], cmd[10], cmd[11], writeAddress);

    // cmd[0..8] are reserved for axis positions
    for (int index = 0; index < PMAC_MAX_CS_AXES; index++) {
      if ((1 << index & tScanAxisMask_) > 0) {
        pHardware_->startAxisPointsCmd(cmd[index], index, writeAddress, tScanPmacBufferSize_);
      }
    }

    int bufferCount = 0;
    firstVal = true;
    while ((bufferCount < nBuffers) && (epicsBufferPtr < tScanPmacBufferSize_) &&
           (tScanPointCtr_ < tScanNumPoints_)) {
      // Create the velmode/user/time memory writes:
      // First 4 bits are for velocity mode %01X
      // Second 4 bits are for user buffer %01X
      // Remaining 24 bits are for delta times %06X
      if (status == asynSuccess) {
        status = pTrajectory_->getVelocityMode(tScanPointCtr_, &velModeValue);
      }
      if (status == asynSuccess) {
        status = pTrajectory_->getUserMode(tScanPointCtr_, &userValue);
      }
      if (status == asynSuccess) {
        status = pTrajectory_->getTime(tScanPointCtr_, &timeValue);
      }
      if (status == asynSuccess) {
        pHardware_->addTrajectoryTimePointCmd(cmd[9], cmd[10], cmd[11],
                velModeValue, userValue, timeValue, firstVal);
        for (int index = 0; index < PMAC_MAX_CS_AXES; index++) {
          if ((1 << index & tScanAxisMask_) > 0) {
            status = pTrajectory_->getPosition(index, tScanPointCtr_, &posValue);
            pHardware_->addAxisPointCmd(cmd[index], index, posValue, tScanPmacBufferSize_,
                                        firstVal);
          }
        }
      }
      // Increment the scan point counter
      tScanPointCtr_++;
      // Increment the buffer count
      bufferCount++;
      // Increment the epicsBufferPtr
      epicsBufferPtr++;
      firstVal = false;
    }

    if (status == asynSuccess) {
      // First send the times/user buffer
      for (int index = 9; index <= 11; index++)
      {
        sprintf(cstr, "%s", cmd[index]);
        debug(DEBUG_VARIABLE, functionName, "Command", cstr);
        status = this->immediateWriteRead(cstr, response);
      }
      // Now send the axis positions
      for (int index = 0; index < PMAC_MAX_CS_AXES; index++) {
        if ((1 << index & tScanAxisMask_) > 0) {
          sprintf(cstr, "%s", cmd[index]);
          debug(DEBUG_VARIABLE, functionName, "Command", cstr);
          status = this->immediateWriteRead(cstr, response);
        }
      }

      // Set the parameter according to the filled points
      if (buffer == PMAC_TRAJ_BUFFER_A) {
        setIntegerParam(PMAC_C_TrajBuffFillA_, epicsBufferPtr);
      } else if (buffer == PMAC_TRAJ_BUFFER_B) {
        setIntegerParam(PMAC_C_TrajBuffFillB_, epicsBufferPtr);
      } else {
        debug(DEBUG_ERROR, functionName, "Out of range buffer pointer", buffer);
        status = asynError;
      }
    }
  }

  // Finally send the current buffer pointer to the PMAC
  if (buffer == PMAC_TRAJ_BUFFER_A) {
    sprintf(cstr, "%s=%d", PMAC_TRAJ_BUFF_FILL_A, epicsBufferPtr);
  } else if (buffer == PMAC_TRAJ_BUFFER_B) {
    sprintf(cstr, "%s=%d", PMAC_TRAJ_BUFF_FILL_B, epicsBufferPtr);
  } else {
    debug(DEBUG_ERROR, functionName, "Out of range buffer pointer", buffer);
    status = asynError;
  }
  debug(DEBUG_TRACE, functionName, "Command", cstr);
  status = this->immediateWriteRead(cstr, response);

  // Reinstate the status reading within the message broker
  pBroker_->reinstateStatusReads();

  stopTimer(DEBUG_TIMING, functionName, "Time taken to send trajectory demand");

  return status;
}

asynStatus pmacController::updateStatistics() {
  asynStatus status = asynSuccess;
  int noOfMsgs = 0;
  int totalBytesWritten = 0;
  int totalBytesRead = 0;
  int totalMsgTime = 0;
  int lastMsgBytesWritten = 0;
  int lastMsgBytesRead = 0;
  int lastMsgTime = 0;
  int maxBytesWritten = 0;
  int maxBytesRead = 0;
  int maxTime = 0;
  static const char *functionName = "updateStatistics";

  debug(DEBUG_FLOW, functionName);

  status = pBroker_->readStatistics(&noOfMsgs,
                                    &totalBytesWritten,
                                    &totalBytesRead,
                                    &totalMsgTime,
                                    &lastMsgBytesWritten,
                                    &lastMsgBytesRead,
                                    &lastMsgTime);

  setIntegerParam(PMAC_C_NoOfMsgs_, noOfMsgs);
  setIntegerParam(PMAC_C_TotalBytesWritten_, totalBytesWritten);
  setIntegerParam(PMAC_C_TotalBytesRead_, totalBytesRead);
  setIntegerParam(PMAC_C_MsgBytesWritten_, lastMsgBytesWritten);
  setIntegerParam(PMAC_C_MsgBytesRead_, lastMsgBytesRead);
  setIntegerParam(PMAC_C_MsgTime_, lastMsgTime);
  getIntegerParam(PMAC_C_MaxBytesWritten_, &maxBytesWritten);
  if (lastMsgBytesWritten > maxBytesWritten) {
    setIntegerParam(PMAC_C_MaxBytesWritten_, lastMsgBytesWritten);
  }
  getIntegerParam(PMAC_C_MaxBytesRead_, &maxBytesRead);
  if (lastMsgBytesRead > maxBytesRead) {
    setIntegerParam(PMAC_C_MaxBytesRead_, lastMsgBytesRead);
  }
  getIntegerParam(PMAC_C_MaxTime_, &maxTime);
  if (lastMsgTime > maxTime) {
    setIntegerParam(PMAC_C_MaxTime_, lastMsgTime);
  }
  if (noOfMsgs > 0) {
    setIntegerParam(PMAC_C_AveBytesWritten_, totalBytesWritten / noOfMsgs);
    setIntegerParam(PMAC_C_AveBytesRead_, totalBytesRead / noOfMsgs);
    setIntegerParam(PMAC_C_AveTime_, totalMsgTime / noOfMsgs);
  }
  callParamCallbacks();

  return status;
}

/**
 * Disable the check in the axis poller that reads ix24 to check if hardware limits
 * are disabled. By default this is enabled for safety reasons. It sets the motor
 * record PROBLEM bit in MSTA, which results in the record going into MAJOR/STATE alarm.
 * @param axis Axis number to disable the check for.
 */
asynStatus pmacController::pmacDisableLimitsCheck(int axis) {
  pmacAxis *pA = NULL;
  asynStatus result = asynSuccess;
  static const char *functionName = "pmacController::pmacDisableLimitsCheck";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  this->lock();
  pA = getAxis(axis);
  if (pA) {
    pA->limitsCheckDisable_ = 1;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s. Disabling hardware limits disable check on controller %s, axis %d\n",
              functionName, portName, pA->axisNo_);
  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s: Error: axis %d has not been configured using pmacCreateAxis.\n", functionName,
              axis);
    result = asynError;
  }
  this->unlock();
  return result;
}

/**
 * Disable the check in the axis poller that reads ix24 to check if hardware limits
 * are disabled. By default this is enabled for safety reasons. It sets the motor
 * record PROBLEM bit in MSTA, which results in the record going into MAJOR/STATE alarm.
 * This function will disable the check for all axes on this controller.
 */
asynStatus pmacController::pmacDisableLimitsCheck(void) {
  pmacAxis *pA = NULL;
  static const char *functionName = "pmacController::pmacDisableLimitsCheck";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  this->lock();
  for (int i = 1; i < numAxes_; i++) {
    pA = getAxis(i);
    if (!pA) continue;
    pA->limitsCheckDisable_ = 1;
//    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
//              "%s. Disabling hardware limits disable check on controller %s, axis %d\n",
//              functionName, portName, pA->axisNo_);
  }
  this->unlock();
  return asynSuccess;
}


/**
 * Set the PMAC axis scale factor to increase resolution in the motor record.
 * Default value is 1.
 * @param axis Axis number to set the PMAC axis scale factor.
 * @param scale Scale factor to set
 */
asynStatus pmacController::pmacSetAxisScale(int axis, int scale) {
  asynStatus result = asynSuccess;
  pmacAxis *pA = NULL;
  static const char *functionName = "pmacController::pmacSetAxisScale";

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
      setIntegerParam(axis, PMAC_C_MotorScale_, scale);
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


/**
 * If we have an open loop axis that has an encoder coming back on a different channel
 * then the encoder readback axis number can be set here. This ensures that the encoder
 * will be used for the position readback. It will also ensure that the encoder axis
 * is set correctly when performing a set position on the open loop axis.
 *
 * To use this function, the axis number used for the encoder must have been configured
 * already using pmacCreateAxis.
 *
 * @param controller The Asyn port name for the PMAC controller.
 * @param axis Axis number to set the PMAC axis scale factor.
 * @param encoder_axis The axis number that the encoder is fed into.
 */
asynStatus pmacController::pmacSetOpenLoopEncoderAxis(int axis, int encoder_axis) {
  pmacAxis *pA = NULL;
  asynStatus result = asynSuccess;
  static const char *functionName = "pmacController::pmacSetOpenLoopEncoderAxis";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  this->lock();
  pA = getAxis(axis);
  if (pA) {
    //Test that the encoder axis has also been configured
    if (getAxis(encoder_axis) == NULL) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s: Error: encoder axis %d has not been configured using pmacCreateAxis.\n",
                functionName, encoder_axis);
      result = asynError;
    } else {
      pA->encoder_axis_ = encoder_axis;
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s. Setting encoder axis %d for axis %d, on controller %s.\n",
                functionName, pA->encoder_axis_, pA->axisNo_, portName);
    }
  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s: Error: axis %d has not been configured using pmacCreateAxis.\n", functionName,
              axis);
    result = asynError;
  }
  this->unlock();
  return result;
}

asynStatus pmacController::registerForCallbacks(pmacCallbackInterface *cbPtr, int type) {
  return pBroker_->registerForUpdates(cbPtr, type);
}

asynStatus pmacController::monitorPMACVariable(int poll_speed, const char *var) {
  return pBroker_->addReadVariable(poll_speed, var);
}

asynStatus pmacController::registerCS(pmacCSController *csPtr, const char *portName, int csNo) {
  static const char *functionName = "registerCS";

  debug(DEBUG_VARIABLE, functionName, "Registering CS", csNo);

  // Add the CS to the list
  pCSControllers_[csNo] = csPtr;

  // Record the port name to CS number mapping
  debug(DEBUG_ERROR, functionName, "CS port name", portName);
  pPortToCs_->insert(portName, csNo);

  if (initialised_){
    this->initCSHardware(csNo);
  }

  return asynSuccess;
}

asynStatus pmacController::initCSHardware(int csNo)
{
  pmacCSController *csPtr = NULL;
  static const char *functionName = "initCSHardware";

  debug(DEBUG_VARIABLE, functionName, "Initialising CS with broker", csNo);
  // Check if we have a valid csPtr for this CS
  csPtr = pCSControllers_[csNo];

  if (csPtr != NULL){
    // setup monitoring of movement of all axes in this CS
    pAxisZero->registerCS(csPtr, csNo);

    // Add the CS status item to the fast update
    this->pHardware_->setupCSStatus(csNo);

    // Now register the CS object for callbacks from the broker
    this->pBroker_->registerForUpdates(csPtr, pmacMessageBroker::PMAC_FAST_READ);
    this->pBroker_->registerForUpdates(csPtr, pmacMessageBroker::PMAC_PRE_FAST_READ);
    this->pBroker_->registerForUpdates(csPtr, pmacMessageBroker::PMAC_SLOW_READ);

    // Notify the CS object that we have a valid connection
    csPtr->initComplete();
  }
  return asynSuccess;
}

asynStatus pmacController::makeCSDemandsConsistent() {
  static const char *functionName = "makeCSDemandsConsistent";
  int csNum = 0;
  int rawAxisIndex = 0;
  int qvar = 0;
  int qvars_assigned = 0;
  char axisAssignment[PMAC_MAXBUF_];
  char command[PMAC_MAXBUF_];
  char reply[PMAC_MAXBUF_];
  bool csHasRawMovedKinematics;
  double pos;
  std::string axesString = "ABCUVWXYZ";
  asynStatus status = asynSuccess;

  debug(DEBUG_FLOW, functionName);

  // Loop over all CS
  for (csNum = 0; csNum < PMAC_MAX_CS; csNum++) {
    csHasRawMovedKinematics = false;
    if (pCSControllers_[csNum] != NULL) {
      // set qvars assigned = 0 none have been set yet
      qvars_assigned = 0;
      // Valid CS, check the motors which are present in this CS
      // in this first loop we are looking for any 1-1 mapped axes
      for (rawAxisIndex = 1; rawAxisIndex <= numAxes_; rawAxisIndex++) {
        pmacAxis *aPtr = this->getAxis(rawAxisIndex);
        if (aPtr != NULL) {
          // Check the motor CS assignment
          if (csNum == aPtr->getAxisCSNo()) {
            getStringParam(rawAxisIndex, PMAC_C_GroupAssignRBV_, PMAC_MAXBUF_, axisAssignment);
            // is this motor assigned directly to a CS axis ?
            if (strcmp(axisAssignment, "") != 0) {
              unsigned long uCsAxisNo = axesString.find(axisAssignment);
              if (uCsAxisNo != std::string::npos) {
                int csAxisAssignmentNo = (int) uCsAxisNo;
                debug(DEBUG_TRACE, functionName, "Found motor assignment for CS", csNum);
                debug(DEBUG_TRACE, functionName, "Motor assignment for motor", rawAxisIndex);
                debug(DEBUG_TRACE, functionName, "Motor assignment", axisAssignment);
                debug(DEBUG_TRACE, functionName, "Axis index", csAxisAssignmentNo);
                if (aPtr->csRawMoveInitiated_ || this->csResetAllDemands) {
                  aPtr->csRawMoveInitiated_ = false;
                  qvar = 71 + csAxisAssignmentNo;
                  debug(DEBUG_TRACE, functionName, "Q Variable for demand", qvar);
                  // Set the qvars assigned flag and send the relevant demand position
                  qvars_assigned = qvars_assigned | 1 << csAxisAssignmentNo;
                  debug(DEBUG_TRACE, functionName, "Q Vars assigned flag", qvars_assigned);
                  if (this->csResetAllDemands) {
                    pos = aPtr->getPosition();
                    debugf(DEBUG_TRACE, functionName, "CS%d Q%d set to current pos %f", csNum, qvar, pos);
                    sprintf(command, "&%dQ%d=%f", csNum, qvar, pos);
                  } else {
                    pos = aPtr->getCachedPosition();
                    debugf(DEBUG_TRACE, functionName, "CS%d Q%d set to cached pos %f", csNum, qvar, pos);
                    sprintf(command, "&%dQ%d=%f", csNum, qvar, pos);
                  }
                  if (pBroker_->immediateWriteRead(command, reply) != asynSuccess) {
                    debug(DEBUG_ERROR, functionName, "Failed to send command", command);
                    status = asynError;
                  }
                }
                qvar = 1 + csAxisAssignmentNo;
                debug(DEBUG_TRACE, functionName, "Q Variable for demand", qvar);
                sprintf(command, "&%dQ%d=%f", csNum, qvar, aPtr->getPosition());
                if (pBroker_->immediateWriteRead(command, reply) != asynSuccess) {
                  debug(DEBUG_ERROR, functionName, "Failed to send command", command);
                  status = asynError;
                }
              } else if (aPtr->csRawMoveInitiated_ || this->csResetAllDemands) {
                if (strcmp(axisAssignment, "I") == 0) {
                  aPtr->csRawMoveInitiated_ = false;
                  csHasRawMovedKinematics = true;
                }
                // Note if axis assignment is not "I" then is not 1-1 and not kinematic so must be
                // of the form #1->2000*X or similar. We cannot support this form at present.
              }
            }
          }
        }
      }
      // Now loop over each CS axis and set any demands that haven't already been set
      // This loop covers all kinematic axes (and those that are not assigned)
      for (int csAxisIndex = 1; csAxisIndex <= PMAC_CS_AXES_COUNT; csAxisIndex++) {
        if ((qvars_assigned & (1 << (csAxisIndex - 1))) == 0) {
          // This axis has not already had its demand set.
          if (csHasRawMovedKinematics) {
            qvar = 70 + csAxisIndex;
            sprintf(command, "&%dQ%d=Q%d", csNum, qvar, (qvar + 10));
            if (pBroker_->immediateWriteRead(command, reply) != asynSuccess) {
              debug(DEBUG_ERROR, functionName, "Failed to send command", command);
              status = asynError;
            }
            qvar = csAxisIndex;
            sprintf(command, "&%dQ%d=Q%d", csNum, qvar, (qvar + 70));
            if (pBroker_->immediateWriteRead(command, reply) != asynSuccess) {
              debug(DEBUG_ERROR, functionName, "Failed to send command", command);
              status = asynError;
            }
          }
        }
      }
    }
  }
  this->csResetAllDemands = false;

  return status;
}

asynStatus pmacController::readDeviceType() {
  asynStatus status = asynSuccess;
  char reply[PMAC_MAXBUF];
  char cmd[PMAC_MAXBUF];
  int nvals = 0;
  static const char *functionName = "readDeviceType";

  debug(DEBUG_FLOW, functionName);

  strcpy(cmd, "cid");
  status = pBroker_->immediateWriteRead(cmd, reply);
  if (status == asynSuccess) {
    nvals = sscanf(reply, "%d", &cid_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading card ID (cid)");
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", reply);
      status = asynError;
    }
  }
  debug(DEBUG_VARIABLE, functionName, "Read device ID", cid_);
  if (status == asynSuccess) {
    strcpy(cmd, "cpu");
    status = pBroker_->immediateWriteRead(cmd, reply);
    if (status == asynSuccess) {
      cpu_.assign(reply);
      // Remove any CR characters
      if (cpu_.find("\r") != std::string::npos) {
        cpu_ = cpu_.substr(0, cpu_.find("\r"));
      }
    } else {
      debug(DEBUG_ERROR, functionName, "Error reading card cpu");
      debug(DEBUG_ERROR, functionName, "    response", reply);
    }
  }
  debug(DEBUG_VARIABLE, functionName, "Read device CPU", cpu_);
  return status;
}

asynStatus pmacController::listPLCProgram(int plcNo, char *buffer, size_t size) {
  int word = 0;
  char reply[PMAC_MAXBUF];
  char cmd[PMAC_MAXBUF];
  char line[PMAC_MAXBUF];
  int cword = 0;
  int running = 1;
  asynStatus status = asynSuccess;
  static const char *functionName = "listPLCProgram";

  startTimer(DEBUG_ERROR, functionName);
  debug(DEBUG_FLOW, functionName);
  debug(DEBUG_VARIABLE, functionName, "Listing PLC", plcNo);

  // Setup the list command
  strcpy(buffer, "");
  while (running == 1 && word < 10000) {
    sprintf(cmd, "list plc%d,%d,1", plcNo, word);
    pBroker_->immediateWriteRead(cmd, reply);
    if (reply[0] == 0x7) {
      running = 0;
    } else {
      sscanf(reply, "%d:%s", &cword, line);
      if (cword == word) {
        if (strlen(buffer) + strlen(line) + 1 > size) {
          // We cannot add the next line as the buffer would be full
          // Report the error
          running = 0;
          status = asynError;
        } else {
          strcat(buffer, line);
          strcat(buffer, " ");
        }
      }
    }
    word++;
  }
  debug(DEBUG_VARIABLE, functionName, "PLC", buffer);
  stopTimer(DEBUG_ERROR, functionName, "Time taken to list PLC");

  return status;
}

asynStatus pmacController::processDeferredMoves(void) {
  asynStatus status = asynSuccess;
  char command[PMAC_MAXBUF_] = {0};
  char response[PMAC_MAXBUF_] = {0};
  pmacAxis *pAxis = NULL;
  static const char *functionName = "processDeferredMoves";

  debug(DEBUG_FLOW, functionName);
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  //Build up combined move command for all axes involved in the deferred move.
  for (int axis = 1; axis < numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (pAxis != NULL) {
      if (pAxis->deferredMove_) {
        sprintf(command, "%s #%d%s%.2f", command, pAxis->axisNo_,
                pAxis->deferredRelative_ ? "J^" : "J=",
                pAxis->deferredPosition_);
      }
    }
  }

  //Execute the deferred move
  if (lowLevelWriteRead(command, response) != asynSuccess) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s ERROR Sending Deferred Move Command.\n",
              functionName);
    setIntegerParam(PMAC_C_CommsError_, PMAC_ERROR_);
    status = asynError;
  } else {
    setIntegerParam(PMAC_C_CommsError_, PMAC_OK_);
    status = asynSuccess;
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

asynStatus pmacController::executeManualGroup() {
  asynStatus status = asynSuccess;
  int csNo = 0;
  int csPort = 0;
  std::string csPortName;
  char csAssignment[MAX_STRING_SIZE];
  char cmd[PMAC_MAXBUF];
  static const char *functionName = "executeManualGroup";

  debug(DEBUG_FLOW, functionName);

  strcpy(cmd, "");
  // Loop over each axis, collecting CS based information
  // building a string representation of the manual group
  for (int axis = 1; axis <= this->numAxes_; axis++) {
    if (this->getAxis(axis) != NULL) {
      debug(DEBUG_VARIABLE, functionName, "Creating manual assignment for axis", axis);
      // Read the current CS port name for this axis
      getIntegerParam(axis, PMAC_C_GroupCSPort_, &csPort);
      debug(DEBUG_VARIABLE, functionName, "csPort", csPort);
      if (csPort > 0) {
        // todo check with Alan what is going on here - csPort is the index to Port Name but
        // todo pCSControllers_ is indexed by csNo isn't it???
        // todo - these are likely the same IFF you define CS in order with no gaps from 1
        csPortName = pCSControllers_[csPort]->getPortName();
        debug(DEBUG_VARIABLE, functionName, "Port name to look up", csPortName);
        if (strcmp(csPortName.c_str(), "")) {
          if (pPortToCs_->hasKey(csPortName.c_str())) {
            // Lookup the current CS number for this axis
            csNo = pPortToCs_->lookup(csPortName.c_str());
            // If the axis has an assigned CS no then we will use it
            if (csNo != 0) {
              // Read the assignment string for this axis
              getStringParam(axis, PMAC_C_GroupAssign_, MAX_STRING_SIZE, csAssignment);
              sprintf(cmd, "%s &%d#%d->%s ", cmd, csNo, axis, csAssignment);
            }
          }
        }
      }
    }
  }
  debug(DEBUG_VARIABLE, functionName, "Assignment", cmd);

  // Execute the manual assignment
  status = pGroupList->manualGroup(cmd);

  updateCsAssignmentParameters();
  copyCsReadbackToDemand(true);

  return status;
}

/**
 * update the parameters for the controller and coordinate system axes
 * to reflect the actual mappings on the pmac. Called every time the coordinate system
 * mappings are altered. Must be called with the lock
 *
 * @return status, asynSuccess if the function succeeds
 *
 */
asynStatus pmacController::updateCsAssignmentParameters() {
  asynStatus status = asynSuccess;
  int csNo = 0;
  char csAssignment[MAX_STRING_SIZE];
  std::string axesString = "ABCUVWXYZ";
  static const char *functionName = "updateCsAssignmentParameters";

  debug(DEBUG_FLOW, functionName);
  // Force updates of the fast loop to pickup any new CS numbers
  pBroker_->updateVariables(pmacMessageBroker::PMAC_FAST_READ);
  // Force two updates of the medium loop to pickup any new axis assignments
  // The first update adds any extra assignment variables to the store if required
  pBroker_->updateVariables(pmacMessageBroker::PMAC_MEDIUM_READ);
  // The second update picks up the assigned axis values for each motor
  pBroker_->updateVariables(pmacMessageBroker::PMAC_MEDIUM_READ);

  callParamCallbacks();

  // reset all CS axes RealMotor assignments before scanning real axes for current assignments
  for (csNo = 1; csNo < PMAC_MAX_CS; csNo++) {
    pmacCSController *pCS = pCSControllers_[csNo];
    if (pCS != NULL) {
      for (int csAxis = 1; csAxis <= 9; csAxis++) {
        pCS->pmacCSSetAxisDirectMapping(csAxis, 0);
      }
    }
  }

  // Loop over each axis, collecting CS based information
  for (int axis = 1; axis <= this->numAxes_; axis++) {
    if (this->getAxis(axis) != NULL) {
      // Read the current CS port name for this axis
      getIntegerParam(axis, PMAC_C_AxisCS_, &csNo);
      if (csNo > 0) {
        getStringParam(axis, PMAC_C_GroupAssignRBV_, MAX_STRING_SIZE, csAssignment);
        // determine if the assignment is a direct mapping to one of ABCUVWXYZ
        unsigned long uCsAxisNo = axesString.find(csAssignment);
        if (uCsAxisNo != std::string::npos) {
          // there is a direct mapping, tell the CS axis it has this mapping
          int csAxisAssignmentNo = (int) uCsAxisNo + 1;
          if (pCSControllers_[csNo] != NULL) {
            pCSControllers_[csNo]->pmacCSSetAxisDirectMapping(csAxisAssignmentNo, axis);
          } else {
            debugf(DEBUG_ERROR, functionName, "Unsupported axis mapping %s in CS%d",
                   csAssignment, csNo);
          }
        }
      }
    }
  }
  callParamCallbacks();
  return status;
}

/**
 * Ensure the demand parameters are consistent after a CS group change or a
 * manual update has taken place.
 *
 * - After a CS group change all values should be copied from the readback parameters
 *   into the demand parameters.
 * - After a manual CS change all non zero and non empty values should be copied from
 *   the readback parameters into the demand parameters.
 *
 * This method is called with the boolean manual flag
 *
 * @param manual A flag to indicate if this is called after a manual CS change.
 * @return status asynSuccess if the function succeeds
 */
asynStatus pmacController::copyCsReadbackToDemand(bool manual)
{
  asynStatus status = asynSuccess;
  int csNo = 0;
  char csAssignment[MAX_STRING_SIZE];
  static const char *functionName = "copyCsReadbackToDemand";

  debug(DEBUG_FLOW, functionName);

  for (int axis = 1; axis <= this->numAxes_; axis++) {
    if (this->getAxis(axis) != NULL) {
      getIntegerParam(axis, PMAC_C_GroupCSPortRBV_, &csNo);
      if (!manual){
        setIntegerParam(axis, PMAC_C_GroupCSPort_, csNo);
      } else if (csNo != 0){
        setIntegerParam(axis, PMAC_C_GroupCSPort_, csNo);
      }
      getStringParam(axis, PMAC_C_GroupAssignRBV_, MAX_STRING_SIZE, csAssignment);
      if (!manual){
        setStringParam(axis, PMAC_C_GroupAssign_, csAssignment);
      } else if (strcmp("", csAssignment) != 0){
        setStringParam(axis, PMAC_C_GroupAssign_, csAssignment);
      }
    }
  }
  callParamCallbacks();
  return status;
}

asynStatus pmacController::tScanBuildProfileArray(double *positions, int axis, int numPoints) {
  asynStatus status = asynSuccess;
  int index = 0;
  int csEnum = 0;
  double resolution = 1.0;
  double offset = 0.0;
  static const char *functionName = "tScanBuildProfileArray";

  debug(DEBUG_TRACE, functionName, "Called for axis", axis);

  if (axis < 0 || axis > 8) {
    debug(DEBUG_ERROR, functionName, "Invalid axis number", axis);
    status = asynError;
  }

  // Determine which CS we currently are using for trajectory scans
  getIntegerParam(PMAC_C_TrajCSPort_, &csEnum);

  // ask the CS for its axis resolution (axis no.s of CS are 1 based)
  resolution = pCSControllers_[csEnum]->getAxisResolution(axis + 1);
  offset = pCSControllers_[csEnum]->getAxisOffset(axis + 1);

  if (status == asynSuccess) {
    debug(DEBUG_VARIABLE, functionName, "Resolution", resolution);
    debug(DEBUG_VARIABLE, functionName, "Offset", offset);

    // Now loop over the points, applying offset and resolution and store
    for (index = 0; index < numPoints; index++) {
      positions[index] = (eguProfilePositions_[axis][index] - offset) / resolution;
    }
  }

  return status;
}

asynStatus pmacController::tScanIncludedAxes(int *axisMask) {
  asynStatus status = asynSuccess;
  int axisUseAddress;
  int mask = 0;
  int use = 0;
  static const char *functionName = "tScanIncludedAxes";

  debug(DEBUG_FLOW, functionName);

  // Loop over each axis and check if it is to be included in the trajectory scan
  //this->lock();
  axisUseAddress = PMAC_C_ProfileUseAxisA_;
  for (int i = 1; i <= PMAC_MAX_CS_AXES; i++) {
    // Check if each axis is in use for the trajectory scan
    getIntegerParam(axisUseAddress, &use);
    debug(DEBUG_VARIABLE, functionName, "Use value", use);
    if (use == 1) {
      mask += 1 << (i - 1);
      debug(DEBUG_VARIABLE, functionName, "Mask value", mask);
    }
    axisUseAddress++;
  }
  //this->unlock();

  debug(DEBUG_VARIABLE, functionName, "Trajectory axis mask", mask);
  *axisMask = mask;

  return status;
}

/*************************************************************************************/
/** The following functions have C linkage, and can be called directly or from iocsh */

extern "C" {

/**
 * C wrapper for the pmacController constructor.
 * See pmacController::pmacController.
 *
 */
asynStatus
pmacCreateController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress,
                     int numAxes, int movingPollPeriod, int idlePollPeriod) {
  pmacController *ppmacController = new pmacController(portName, lowLevelPortName,
                                                       lowLevelPortAddress, numAxes,
                                                       movingPollPeriod / 1000.,
                                                       idlePollPeriod / 1000.);
  ppmacController->startPMACPolling();
  ppmacController = NULL;
  return asynSuccess;
}

/**
 * C wrapper for the pmacAxis constructor.
 * See pmacAxis::pmacAxis.
 *
 */
asynStatus pmacCreateAxis(const char *pmacName,         /* specify which controller by port name */
                          int axis)                    /* axis number (start from 1). */
{
  pmacController *pC;

  static const char *functionName = "pmacCreateAxis";

  pC = (pmacController *) findAsynPortDriver(pmacName);
  if (!pC) {
    printf("%s::%s: ERROR Port %s Not Found.\n",
           driverName, functionName, pmacName);
    return asynError;
  }

  if (axis == 0) {
    printf("%s::%s: ERROR Axis Number 0 Not Allowed. This Asyn Address Is Reserved For Controller Specific Parameters.\n",
           driverName, functionName);
    return asynError;
  }

  pC->lock();
  new pmacAxis(pC, axis);
  pC->unlock();
  return asynSuccess;
}

/**
 * C Wrapper function for pmacAxis constructor.
 * See pmacAxis::pmacAxis.
 * This function allows creation of multiple pmacAxis objects with axis numbers 1 to numAxes.
 * @param pmacName Asyn port name for the controller (const char *)
 * @param numAxes The number of axes to create, starting at 1.
 *
 */
asynStatus pmacCreateAxes(const char *pmacName,
                          int numAxes) {
  pmacController *pC;

  static const char *functionName = "pmacCreateAxis";

  pC = (pmacController *) findAsynPortDriver(pmacName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, pmacName);
    return asynError;
  }

  pC->lock();
  for (int axis = 1; axis <= numAxes; axis++) {
    new pmacAxis(pC, axis);
  }
  pC->unlock();
  return asynSuccess;
}


/**
 * Disable the check in the axis poller that reads ix24 to check if hardware limits
 * are disabled. By default this is enabled for safety reasons. It sets the motor
 * record PROBLEM bit in MSTA, which results in the record going into MAJOR/STATE alarm.
 * @param controller Asyn port name for the controller (const char *)
 * @param axis Axis number to disable the check for.
 * @param allAxes Set to 0 if only dealing with one axis.
 *                Set to 1 to do all axes (in which case the axis parameter is ignored).
 */
asynStatus pmacDisableLimitsCheck(const char *controller, int axis, int allAxes) {
  pmacController *pC;
  static const char *functionName = "pmacDisableLimitsCheck";

  pC = (pmacController *) findAsynPortDriver(controller);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, controller);
    return asynError;
  }

  if (allAxes == 1) {
    return pC->pmacDisableLimitsCheck();
  } else if (allAxes == 0) {
    return pC->pmacDisableLimitsCheck(axis);
  }

  return asynError;
}


/**
 * Set the PMAC axis scale factor to increase resolution in the motor record.
 * Default value is 1.
 * @param controller The Asyn port name for the PMAC controller.
 * @param axis Axis number to set the PMAC axis scale factor.
 * @param scale Scale factor to set
 */
asynStatus pmacSetAxisScale(const char *controller, int axis, int scale) {
  pmacController *pC;
  static const char *functionName = "pmacSetAxisScale";

  pC = (pmacController *) findAsynPortDriver(controller);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, controller);
    return asynError;
  }

  return pC->pmacSetAxisScale(axis, scale);
}


/**
 * If we have an open loop axis that has an encoder coming back on a different channel
 * then the encoder readback axis number can be set here. This ensures that the encoder
 * will be used for the position readback. It will also ensure that the encoder axis
 * is set correctly when performing a set position on the open loop axis.
 *
 * To use this function, the axis number used for the encoder must have been configured
 * already using pmacCreateAxis.
 *
 * @param controller The Asyn port name for the PMAC controller.
 * @param axis Axis number to set the PMAC axis scale factor.
 * @param encoder_axis The axis number that the encoder is fed into.
 */
asynStatus pmacSetOpenLoopEncoderAxis(const char *controller, int axis, int encoder_axis) {
  pmacController *pC;
  static const char *functionName = "pmacSetOpenLoopEncoderAxis";

  pC = (pmacController *) findAsynPortDriver(controller);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, controller);
    return asynError;
  }

  return pC->pmacSetOpenLoopEncoderAxis(axis, encoder_axis);
}

/**
 * Creates a Coordinate System Group to allow us to switch axes into coordinate systems
 *
 * @param controller The Asyn port name for the PMAC controller.
 * @param group number for this CS group
 * @param number of axes in this CS group
 */
asynStatus pmacCreateCsGroup(const char *controller, int groupNo, char *groupName, int axisCount) {
  pmacController *pC;
  static const char *functionName = "pmacCreateCsGroup";

  pC = (pmacController *) findAsynPortDriver(controller);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, controller);
    return asynError;
  }

  pC->pGroupList->addGroup(groupNo, groupName, axisCount);

  return asynSuccess;
}

/**
 * Adds an axis to a Coordinate System Group
 *
 * @param controller The Asyn port name for the PMAC controller.
 * @param group number for this CS group
 * @param axis number to add
 * @param axis definition for this axis
 * @param coordinate system no. for this axis definition
 *
 */
asynStatus pmacCsGroupAddAxis(const char *controller, int groupNo, int axisNo, char *mapping,
                              int coordinateSysNo) {
  pmacController *pC;
  static const char *functionName = "pmacCsGroupAddAxis";

  pC = (pmacController *) findAsynPortDriver(controller);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, controller);
    return asynError;
  }

  pC->pGroupList->addAxisToGroup(groupNo, axisNo, mapping, coordinateSysNo);

  return asynSuccess;
}

/**
 * Sets the debug level for the PMAC controller or an axis
 *
 * @param controller The Asyn port name for the PMAC controller.
 * @param level The required level of debug (bitmask).
 * @param axis (0 for controller, 1-n for axis number).
 *
 */
asynStatus pmacDebug(const char *controller, int level, int axis, int csNo) {
  pmacController *pC;
  static const char *functionName = "pmacDebug";

  pC = (pmacController *) findAsynPortDriver(controller);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n", driverName, functionName, controller);
    return asynError;
  }

  pC->setDebugLevel(level, axis, csNo);

  return asynSuccess;
}

asynStatus pmacNoCsVelocity(const char *controller) {
  pmacController *pC;
  static const char *functionName = "pmacNoCsVelocity";

  pC = (pmacController *) findAsynPortDriver(controller);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n", driverName, functionName, controller);
    return asynError;
  }

  pC->useCsVelocity=false;

  return asynSuccess;
}

asynStatus pmacMonitorVariables(const char *controller, const char *variablesString) {
  std::string variables = std::string(variablesString);
  pmacController *pC;
  static const char *functionName = "pmacMonitorVariables";

  pC = (pmacController *) findAsynPortDriver(controller);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n", driverName, functionName, controller);
    return asynError;
  }

  pC->addBrokerVariables(variables);

  return asynSuccess;
}

/* Code for iocsh registration */

/* pmacCreateController */
static const iocshArg pmacCreateControllerArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateControllerArg1 = {"Low level port name", iocshArgString};
static const iocshArg pmacCreateControllerArg2 = {"Low level port address", iocshArgInt};
static const iocshArg pmacCreateControllerArg3 = {"Number of axes", iocshArgInt};
static const iocshArg pmacCreateControllerArg4 = {"Moving poll rate (ms)", iocshArgInt};
static const iocshArg pmacCreateControllerArg5 = {"Idle poll rate (ms)", iocshArgInt};
static const iocshArg *const pmacCreateControllerArgs[] = {&pmacCreateControllerArg0,
                                                           &pmacCreateControllerArg1,
                                                           &pmacCreateControllerArg2,
                                                           &pmacCreateControllerArg3,
                                                           &pmacCreateControllerArg4,
                                                           &pmacCreateControllerArg5};
static const iocshFuncDef configpmacCreateController = {"pmacCreateController", 6,
                                                        pmacCreateControllerArgs};
static void configpmacCreateControllerCallFunc(const iocshArgBuf *args) {
  pmacCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival,
                       args[5].ival);
}


/* pmacCreateAxis */
static const iocshArg pmacCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg *const pmacCreateAxisArgs[] = {&pmacCreateAxisArg0,
                                                     &pmacCreateAxisArg1};
static const iocshFuncDef configpmacAxis = {"pmacCreateAxis", 2, pmacCreateAxisArgs};

static void configpmacAxisCallFunc(const iocshArgBuf *args) {
  pmacCreateAxis(args[0].sval, args[1].ival);
}

/* pmacCreateAxes */
static const iocshArg pmacCreateAxesArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateAxesArg1 = {"Num Axes", iocshArgInt};
static const iocshArg *const pmacCreateAxesArgs[] = {&pmacCreateAxesArg0,
                                                     &pmacCreateAxesArg1};
static const iocshFuncDef configpmacAxes = {"pmacCreateAxes", 2, pmacCreateAxesArgs};

static void configpmacAxesCallFunc(const iocshArgBuf *args) {
  pmacCreateAxes(args[0].sval, args[1].ival);
}


/* pmacDisableLimitsCheck */
static const iocshArg pmacDisableLimitsCheckArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacDisableLimitsCheckArg1 = {"Axis number", iocshArgInt};
static const iocshArg pmacDisableLimitsCheckArg2 = {"All Axes", iocshArgInt};
static const iocshArg *const pmacDisableLimitsCheckArgs[] = {&pmacDisableLimitsCheckArg0,
                                                             &pmacDisableLimitsCheckArg1,
                                                             &pmacDisableLimitsCheckArg2};
static const iocshFuncDef configpmacDisableLimitsCheck = {"pmacDisableLimitsCheck", 3,
                                                          pmacDisableLimitsCheckArgs};

static void configpmacDisableLimitsCheckCallFunc(const iocshArgBuf *args) {
  pmacDisableLimitsCheck(args[0].sval, args[1].ival, args[2].ival);
}


/* pmacSetAxisScale */
static const iocshArg pmacSetAxisScaleArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacSetAxisScaleArg1 = {"Axis number", iocshArgInt};
static const iocshArg pmacSetAxisScaleArg2 = {"Scale", iocshArgInt};
static const iocshArg *const pmacSetAxisScaleArgs[] = {&pmacSetAxisScaleArg0,
                                                       &pmacSetAxisScaleArg1,
                                                       &pmacSetAxisScaleArg2};
static const iocshFuncDef configpmacSetAxisScale = {"pmacSetAxisScale", 3, pmacSetAxisScaleArgs};

static void configpmacSetAxisScaleCallFunc(const iocshArgBuf *args) {
  pmacSetAxisScale(args[0].sval, args[1].ival, args[2].ival);
}

/* pmacSetOpenLoopEncoderAxis */
static const iocshArg pmacSetOpenLoopEncoderAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacSetOpenLoopEncoderAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg pmacSetOpenLoopEncoderAxisArg2 = {"Encoder Axis", iocshArgInt};
static const iocshArg *const pmacSetOpenLoopEncoderAxisArgs[] = {&pmacSetOpenLoopEncoderAxisArg0,
                                                                 &pmacSetOpenLoopEncoderAxisArg1,
                                                                 &pmacSetOpenLoopEncoderAxisArg2};
static const iocshFuncDef configpmacSetOpenLoopEncoderAxis = {"pmacSetOpenLoopEncoderAxis", 3,
                                                              pmacSetOpenLoopEncoderAxisArgs};

static void configpmacSetOpenLoopEncoderAxisCallFunc(const iocshArgBuf *args) {
  pmacSetOpenLoopEncoderAxis(args[0].sval, args[1].ival, args[2].ival);
}

/* pmacCreateCsGroup */
static const iocshArg pmacCreateCsGroupArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateCsGroupArg1 = {"Group number", iocshArgInt};
static const iocshArg pmacCreateCsGroupArg2 = {"Group name", iocshArgString};
static const iocshArg pmacCreateCsGroupArg3 = {"Axis count", iocshArgInt};
static const iocshArg *const pmacCreateCsGroupArgs[] = {&pmacCreateCsGroupArg0,
                                                        &pmacCreateCsGroupArg1,
                                                        &pmacCreateCsGroupArg2,
                                                        &pmacCreateCsGroupArg3};
static const iocshFuncDef configpmacCreateCsGroup = {"pmacCreateCsGroup", 4, pmacCreateCsGroupArgs};

static void configpmacCreateCsGroupCallFunc(const iocshArgBuf *args) {
  pmacCreateCsGroup(args[0].sval, args[1].ival, args[2].sval, args[3].ival);
}

/* pmacCsGroupAddAxis */
static const iocshArg pmacCsGroupAddAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCsGroupAddAxisArg1 = {"Group number", iocshArgInt};
static const iocshArg pmacCsGroupAddAxisArg2 = {"Axis number", iocshArgInt};
static const iocshArg pmacCsGroupAddAxisArg3 = {"Axis CS definition", iocshArgString};
static const iocshArg pmacCsGroupAddAxisArg4 = {"CS number", iocshArgInt};
static const iocshArg *const pmacCsGroupAddAxisArgs[] = {&pmacCsGroupAddAxisArg0,
                                                         &pmacCsGroupAddAxisArg1,
                                                         &pmacCsGroupAddAxisArg2,
                                                         &pmacCsGroupAddAxisArg3,
                                                         &pmacCsGroupAddAxisArg4};
static const iocshFuncDef configpmacCsGroupAddAxis = {"pmacCsGroupAddAxis", 5,
                                                      pmacCsGroupAddAxisArgs};

static void configpmacCsGroupAddAxisCallFunc(const iocshArgBuf *args) {
  pmacCsGroupAddAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval, args[4].ival);
}

/* pmacDebug */
static const iocshArg pmacDebugArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacDebugArg1 = {"Debug level", iocshArgInt};
static const iocshArg pmacDebugArg2 = {"Axis number", iocshArgInt};
static const iocshArg pmacDebugArg3 = {"CS number", iocshArgInt};
static const iocshArg *const pmacDebugArgs[] = {&pmacDebugArg0,
                                                &pmacDebugArg1,
                                                &pmacDebugArg2,
                                                &pmacDebugArg3};
static const iocshFuncDef configpmacDebug = {"pmacDebug", 4, pmacDebugArgs};

static void configpmacDebugCallFunc(const iocshArgBuf *args) {
  pmacDebug(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
}

/* NoCsVelocity */
static const iocshArg pmacNoVelocityArg0 = {"Controller port name", iocshArgString};
static const iocshArg *const pmacNoCsVelocityArgs[] = {&pmacNoVelocityArg0};
static const iocshFuncDef configpmacNoCsVelocity = {"pmacNoCsVelocity", 1, pmacNoCsVelocityArgs};

static void configpmacNoCsVelocityCallFunc(const iocshArgBuf *args) {
    pmacNoCsVelocity(args[0].sval);
}


/* pmacMonitorVariables */
static const iocshArg pmacMonitorVariables0 = {"Controller port name", iocshArgString};
static const iocshArg pmacMonitorVariables1 = {"Variables", iocshArgString};
static const iocshArg *const pmacMonitorVariablesArgs[] = {&pmacMonitorVariables0, &pmacMonitorVariables1};
static const iocshFuncDef configMonitorVariables = {"pmacMonitorVariables", 2, pmacMonitorVariablesArgs};

static void configpmacMonitorVariablesCallFunc(const iocshArgBuf *args) {
  pmacMonitorVariables(args[0].sval, args[1].sval);
}

static void pmacControllerRegister(void) {
  iocshRegister(&configpmacCreateController, configpmacCreateControllerCallFunc);
  iocshRegister(&configpmacAxis, configpmacAxisCallFunc);
  iocshRegister(&configpmacAxes, configpmacAxesCallFunc);
  iocshRegister(&configpmacDisableLimitsCheck, configpmacDisableLimitsCheckCallFunc);
  iocshRegister(&configpmacSetAxisScale, configpmacSetAxisScaleCallFunc);
  iocshRegister(&configpmacSetOpenLoopEncoderAxis, configpmacSetOpenLoopEncoderAxisCallFunc);
  iocshRegister(&configpmacCreateCsGroup, configpmacCreateCsGroupCallFunc);
  iocshRegister(&configpmacCsGroupAddAxis, configpmacCsGroupAddAxisCallFunc);
  iocshRegister(&configpmacDebug, configpmacDebugCallFunc);
  iocshRegister(&configpmacNoCsVelocity, configpmacNoCsVelocityCallFunc);
  iocshRegister(&configMonitorVariables, configpmacMonitorVariablesCallFunc);
}
epicsExportRegistrar(pmacControllerRegister);

#ifdef vxWorks
//VxWorks register functions
epicsRegisterFunction(pmacCreateController);
epicsRegisterFunction(pmacCreateAxis);
epicsRegisterFunction(pmacCreateAxes);
epicsRegisterFunction(pmacDisableLimitsCheck);
epicsRegisterFunction(pmacSetAxisScale);
epicsRegisterFunction(pmacSetOpenLoopEncoderAxis);
epicsRegisterFunction(pmacDebug);
#endif
} // extern "C"
