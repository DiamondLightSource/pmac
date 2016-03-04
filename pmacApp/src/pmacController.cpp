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

#include <iostream>
using std::cout;
using std::endl;
using std::dec;

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
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
const epicsFloat64 pmacController::PMAC_TIMEOUT_ = 5.0;
const epicsUInt32 pmacController::PMAC_FEEDRATE_LIM_ = 100;
const epicsUInt32 pmacController::PMAC_ERROR_PRINT_TIME_ = 600; //seconds
const epicsUInt32 pmacController::PMAC_FORCED_FAST_POLLS_ = 10;
const epicsUInt32 pmacController::PMAC_OK_ = 0;
const epicsUInt32 pmacController::PMAC_ERROR_ = 1;
const epicsUInt32 pmacController::PMAC_FEEDRATE_DEADBAND_ = 1;
const epicsInt32 pmacController::PMAC_CID_PMAC_ = 602413;
const epicsInt32 pmacController::PMAC_CID_GEOBRICK_ = 603382;

const epicsUInt32 pmacController::PMAC_STATUS1_MAXRAPID_SPEED    = (0x1<<0);
const epicsUInt32 pmacController::PMAC_STATUS1_ALT_CMNDOUT_MODE  = (0x1<<1);
const epicsUInt32 pmacController::PMAC_STATUS1_SOFT_POS_CAPTURE  = (0x1<<2);
const epicsUInt32 pmacController::PMAC_STATUS1_ERROR_TRIGGER     = (0x1<<3);
const epicsUInt32 pmacController::PMAC_STATUS1_FOLLOW_ENABLE     = (0x1<<4);
const epicsUInt32 pmacController::PMAC_STATUS1_FOLLOW_OFFSET     = (0x1<<5);
const epicsUInt32 pmacController::PMAC_STATUS1_PHASED_MOTOR      = (0x1<<6);
const epicsUInt32 pmacController::PMAC_STATUS1_ALT_SRC_DEST      = (0x1<<7);
const epicsUInt32 pmacController::PMAC_STATUS1_USER_SERVO        = (0x1<<8);
const epicsUInt32 pmacController::PMAC_STATUS1_USER_PHASE        = (0x1<<9);
const epicsUInt32 pmacController::PMAC_STATUS1_HOMING            = (0x1<<10);
const epicsUInt32 pmacController::PMAC_STATUS1_BLOCK_REQUEST     = (0x1<<11);
const epicsUInt32 pmacController::PMAC_STATUS1_DECEL_ABORT       = (0x1<<12);
const epicsUInt32 pmacController::PMAC_STATUS1_DESIRED_VELOCITY_ZERO = (0x1<<13);
const epicsUInt32 pmacController::PMAC_STATUS1_DATABLKERR        = (0x1<<14);
const epicsUInt32 pmacController::PMAC_STATUS1_DWELL             = (0x1<<15);
const epicsUInt32 pmacController::PMAC_STATUS1_INTEGRATE_MODE    = (0x1<<16);
const epicsUInt32 pmacController::PMAC_STATUS1_MOVE_TIME_ON      = (0x1<<17);
const epicsUInt32 pmacController::PMAC_STATUS1_OPEN_LOOP         = (0x1<<18);
const epicsUInt32 pmacController::PMAC_STATUS1_AMP_ENABLED       = (0x1<<19);
const epicsUInt32 pmacController::PMAC_STATUS1_X_SERVO_ON        = (0x1<<20);
const epicsUInt32 pmacController::PMAC_STATUS1_POS_LIMIT_SET     = (0x1<<21);
const epicsUInt32 pmacController::PMAC_STATUS1_NEG_LIMIT_SET     = (0x1<<22);
const epicsUInt32 pmacController::PMAC_STATUS1_MOTOR_ON          = (0x1<<23);

const epicsUInt32 pmacController::PMAC_STATUS2_IN_POSITION       = (0x1<<0);
const epicsUInt32 pmacController::PMAC_STATUS2_WARN_FOLLOW_ERR   = (0x1<<1);
const epicsUInt32 pmacController::PMAC_STATUS2_ERR_FOLLOW_ERR    = (0x1<<2);
const epicsUInt32 pmacController::PMAC_STATUS2_AMP_FAULT         = (0x1<<3);
const epicsUInt32 pmacController::PMAC_STATUS2_NEG_BACKLASH      = (0x1<<4);
const epicsUInt32 pmacController::PMAC_STATUS2_I2T_AMP_FAULT     = (0x1<<5);
const epicsUInt32 pmacController::PMAC_STATUS2_I2_FOLLOW_ERR     = (0x1<<6);
const epicsUInt32 pmacController::PMAC_STATUS2_TRIGGER_MOVE      = (0x1<<7);
const epicsUInt32 pmacController::PMAC_STATUS2_PHASE_REF_ERR     = (0x1<<8);
const epicsUInt32 pmacController::PMAC_STATUS2_PHASE_SEARCH      = (0x1<<9);
const epicsUInt32 pmacController::PMAC_STATUS2_HOME_COMPLETE     = (0x1<<10);
const epicsUInt32 pmacController::PMAC_STATUS2_POS_LIMIT_STOP    = (0x1<<11);
const epicsUInt32 pmacController::PMAC_STATUS2_DESIRED_STOP      = (0x1<<12);
const epicsUInt32 pmacController::PMAC_STATUS2_FORE_IN_POS       = (0x1<<13);
const epicsUInt32 pmacController::PMAC_STATUS2_NA14              = (0x1<<14);
const epicsUInt32 pmacController::PMAC_STATUS2_ASSIGNED_CS       = (0x1<<15);

/*Global status ???*/
const epicsUInt32 pmacController::PMAC_GSTATUS_CARD_ADDR             = (0x1<<0);
const epicsUInt32 pmacController::PMAC_GSTATUS_ALL_CARD_ADDR         = (0x1<<1);
const epicsUInt32 pmacController::PMAC_GSTATUS_RESERVED              = (0x1<<2);
const epicsUInt32 pmacController::PMAC_GSTATUS_PHASE_CLK_MISS        = (0x1<<3);
const epicsUInt32 pmacController::PMAC_GSTATUS_MACRO_RING_ERRORCHECK = (0x1<<4);
const epicsUInt32 pmacController::PMAC_GSTATUS_MACRO_RING_COMMS      = (0x1<<5);
const epicsUInt32 pmacController::PMAC_GSTATUS_TWS_PARITY_ERROR      = (0x1<<6);
const epicsUInt32 pmacController::PMAC_GSTATUS_CONFIG_ERROR          = (0x1<<7);
const epicsUInt32 pmacController::PMAC_GSTATUS_ILLEGAL_LVAR          = (0x1<<8);
const epicsUInt32 pmacController::PMAC_GSTATUS_REALTIME_INTR         = (0x1<<9);
const epicsUInt32 pmacController::PMAC_GSTATUS_FLASH_ERROR           = (0x1<<10);
const epicsUInt32 pmacController::PMAC_GSTATUS_DPRAM_ERROR           = (0x1<<11);
const epicsUInt32 pmacController::PMAC_GSTATUS_CKSUM_ACTIVE          = (0x1<<12);
const epicsUInt32 pmacController::PMAC_GSTATUS_CKSUM_ERROR           = (0x1<<13);
const epicsUInt32 pmacController::PMAC_GSTATUS_LEADSCREW_COMP        = (0x1<<14);
const epicsUInt32 pmacController::PMAC_GSTATUS_WATCHDOG              = (0x1<<15);
const epicsUInt32 pmacController::PMAC_GSTATUS_SERVO_REQ             = (0x1<<16);
const epicsUInt32 pmacController::PMAC_GSTATUS_DATA_GATHER_START     = (0x1<<17);
const epicsUInt32 pmacController::PMAC_GSTATUS_RESERVED2             = (0x1<<18);
const epicsUInt32 pmacController::PMAC_GSTATUS_DATA_GATHER_ON        = (0x1<<19);
const epicsUInt32 pmacController::PMAC_GSTATUS_SERVO_ERROR           = (0x1<<20);
const epicsUInt32 pmacController::PMAC_GSTATUS_CPUTYPE               = (0x1<<21);
const epicsUInt32 pmacController::PMAC_GSTATUS_REALTIME_INTR_RE      = (0x1<<22);
const epicsUInt32 pmacController::PMAC_GSTATUS_RESERVED3             = (0x1<<23);

const epicsUInt32 pmacController::PMAC_HARDWARE_PROB = (PMAC_GSTATUS_MACRO_RING_ERRORCHECK | PMAC_GSTATUS_MACRO_RING_COMMS | PMAC_GSTATUS_REALTIME_INTR | PMAC_GSTATUS_FLASH_ERROR | PMAC_GSTATUS_DPRAM_ERROR | PMAC_GSTATUS_CKSUM_ERROR | PMAC_GSTATUS_WATCHDOG | PMAC_GSTATUS_SERVO_ERROR);

const epicsUInt32 pmacController::PMAX_AXIS_GENERAL_PROB1 = 0;
const epicsUInt32 pmacController::PMAX_AXIS_GENERAL_PROB2 = (PMAC_STATUS2_DESIRED_STOP | PMAC_STATUS2_AMP_FAULT);


//C function prototypes, for the functions that can be called on IOC shell.
//Some of these functions are provided to ease transition to the model 3 driver. Some of these
//functions could be handled by the parameter library.
extern "C"
{
  asynStatus pmacCreateController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, 
					 int numAxes, int movingPollPeriod, int idlePollPeriod);
  
  asynStatus pmacCreateAxis(const char *pmacName, int axis);

  asynStatus pmacCreateAxes(const char *pmacName, int numAxes);
  
  asynStatus pmacDisableLimitsCheck(const char *controller, int axis, int allAxes);
  
  asynStatus pmacSetAxisScale(const char *controller, int axis, int scale);
  
  asynStatus pmacSetOpenLoopEncoderAxis(const char *controller, int axis, int encoder_axis);

  
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
pmacController::pmacController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, 
			       int numAxes, double movingPollPeriod, double idlePollPeriod)
  : asynMotorController(portName, numAxes+1, NUM_MOTOR_DRIVER_PARAMS+NUM_PMAC_PARAMS,
			0, // No additional interfaces
			0, // No addition interrupt interfaces
			ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
			1, // autoconnect
			0, 0),  // Default priority and stack size
	pmacDebugger("pmacController")
{
  int index = 0;
  static const char *functionName = "pmacController::pmacController";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Constructor.\n", functionName);

  //Initialize non static data members
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

  // Create the parameter hashtables
  pIntParams_ = new IntegerHashtable();
  pDoubleParams_ = new IntegerHashtable();
  pStringParams_ = new IntegerHashtable();
  pWriteParams_ = new StringHashtable();

  pAxes_ = (pmacAxis **)(asynMotorController::pAxes_);

  // Initialise the table of CS controller pointers
  pCSControllers_ = (pmacCSController **)malloc(16 * sizeof(pmacCSController *));
  for (index = 0; index < 16; index++){
    pCSControllers_[index] = NULL;
  }

  //Create dummy axis for asyn address 0. This is used for controller parameters.
  //pAxisZero = new pmacAxis(this, 0);
  pAxisZero = new pmacCSMonitor(this);
  pGroupList = new pmacCsGroups(this);

  //Create controller-specific parameters
  createParam(PMAC_C_FirstParamString,       asynParamInt32,   &PMAC_C_FirstParam_);
  createParam(PMAC_C_GlobalStatusString,     asynParamInt32,   &PMAC_C_GlobalStatus_);
  createParam(PMAC_C_CommsErrorString,       asynParamInt32,   &PMAC_C_CommsError_);
  createParam(PMAC_C_FeedRateString,         asynParamInt32,   &PMAC_C_FeedRate_);
  createParam(PMAC_C_FeedRateLimitString,    asynParamInt32,   &PMAC_C_FeedRateLimit_);
  createParam(PMAC_C_FeedRatePollString,     asynParamInt32,   &PMAC_C_FeedRatePoll_);
  createParam(PMAC_C_FeedRateProblemString,  asynParamInt32,   &PMAC_C_FeedRateProblem_);
  createParam(PMAC_C_CoordSysGroup,          asynParamInt32,   &PMAC_C_CoordSysGroup_);
  createParam(PMAC_C_FastUpdateTimeString,   asynParamFloat64, &PMAC_C_FastUpdateTime_);
  createParam(PMAC_C_LastParamString,        asynParamInt32,   &PMAC_C_LastParam_);
  createParam(PMAC_C_AxisCSString,           asynParamInt32,   &PMAC_C_AxisCS_);
  createParam(PMAC_C_WriteCmdString,         asynParamOctet,   &PMAC_C_WriteCmd_);
  createParam(PMAC_C_KillAxisString,         asynParamInt32,   &PMAC_C_KillAxis_);

  pBroker_ = new pmacMessageBroker(this->pasynUserSelf);

  //printf("** Broker created\n");
  //printf("** Variables added\n");

  //Connect our Asyn user to the low level port that is a parameter to this constructor
//  if (lowLevelPortConnect(lowLevelPortName, lowLevelPortAddress, &lowLevelPortUser_,
//		  (char*)"\006", (char*)"\r") != asynSuccess) {

  if (pBroker_->connect(lowLevelPortName, lowLevelPortAddress) != asynSuccess){
    printf("%s: Failed to connect to low level asynOctetSyncIO port %s\n", functionName, lowLevelPortName);
    setIntegerParam(PMAC_C_CommsError_, PMAC_ERROR_);
  } else {
    setIntegerParam(PMAC_C_CommsError_, PMAC_OK_);
  }
  startPoller(movingPollPeriod, idlePollPeriod, PMAC_FORCED_FAST_POLLS_);

  bool paramStatus = true;
  paramStatus = ((setIntegerParam(PMAC_C_GlobalStatus_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_FeedRateProblem_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(PMAC_C_FeedRateLimit_, 100) == asynSuccess) && paramStatus);
  paramStatus = ((setDoubleParam(PMAC_C_FastUpdateTime_, 0.0) == asynSuccess) && paramStatus);

  callParamCallbacks();

  if (!paramStatus) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s Unable To Set Driver Parameters In Constructor.\n", functionName);
  }
 
  // Add the items required for global status
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, "???");
  pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, "%");
  // Register this class for updates
  pBroker_->registerForUpdates(this, pmacMessageBroker::PMAC_FAST_READ);
  pBroker_->registerForUpdates(this, pmacMessageBroker::PMAC_MEDIUM_READ);
  pBroker_->registerForUpdates(this, pmacMessageBroker::PMAC_SLOW_READ);

}


pmacController::~pmacController(void) 
{
  //Destructor. Should never get here.
  delete pAxisZero;
}

void pmacController::setDebugLevel(int level, int axis)
{
  // Check if an axis or controller wide debug is to be set
  if (axis == 0){
    printf("Setting PMAC controller debug level to %d\n", level);
    // Set the level for the controller
    this->setLevel(level);
    // Set the level for the broker
    pBroker_->setLevel(level);
  } else {
    if (this->getAxis(axis) != NULL){
      printf("Setting PMAC axis %d debug level to %d\n", axis, level);
      this->getAxis(axis)->setLevel(level);
    }
  }
}

asynStatus pmacController::drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize)
{
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
  // x is I for int, D for double or S for string
  //
  // The must be no j or = in a variable, these items will simply be polled for their current status
  //
  // For commands
  // PMAC_CMDx_... => Send command to PMAC
  //
  // x is I for int, D for double or S for string

  // For Writing only
  // PMAC_WI_... => Write Integer Value
  // PMAC_WD_... => Write Double Value
  // PMAC_WS_... => Write String Value
  //
  // Writing to these parameters will result in immediate writes to the PMAC


  // Check if we have already provided maximum number of custom parameters
  if (parameterIndex_ > PMAC_MAX_PARAMETERS){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "%s:%s: Not enough space allocated to store all camera features, increase NFEATURES\n",
          driverName, functionName);
    status = asynError;
  }

  if (status == asynSuccess){

    if (findParam(drvInfo, &index) && strlen(drvInfo) > 9 && strncmp(drvInfo, "PMAC_V", 6) == 0 && drvInfo[8] == '_'){

      // Retrieve the name of the variable
      char *pmacVariable = epicsStrDup(drvInfo + 9);

      printf("Creating new parameter %s\n", pmacVariable);
      // Check for I, D or S in drvInfo[7]
      switch(drvInfo[6]) {
        case 'I':
          // Create the parameter
          createParam(drvInfo, asynParamInt32, &(this->parameters[parameterIndex_]));
          setIntegerParam(this->parameters[parameterIndex_], 0);
          // Add variable to integer parameter hashtable
          this->pIntParams_->insert(pmacVariable, this->parameters[parameterIndex_]);
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

      if (status == asynSuccess){
        // Check for F, M or S in drvInfo[6]
        switch(drvInfo[7]) {
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

        if (status == asynSuccess){
          this->pWriteParams_->insert(drvInfo, pmacVariable);
        }
      }



      // Create the parameter, store the param ID along with the var name in the hashtable

    }

    if (findParam(drvInfo, &index) && strlen(drvInfo) > 9 && strncmp(drvInfo, "PMAC_W", 6) == 0 && drvInfo[7] == '_'){

      // Retrieve the name of the variable
      char *pmacVariable = epicsStrDup(drvInfo + 8);

      printf("Creating new write only parameter %s\n", pmacVariable);
      // Check for I, D or S in drvInfo[7]
      switch(drvInfo[6]) {
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

  if (status == asynSuccess){
    // Now return baseclass result
    status = asynMotorController::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
  }
  return status;
}


void pmacController::callback(pmacCommandStore *sPtr, int type)
{
  static const char *functionName = "callback";
  debug(DEBUG_FLOW, functionName);

  if (type == pmacMessageBroker::PMAC_FAST_READ){
    // Parse PMAC global status
    this->newGetGlobalStatus(sPtr);
  }

  // Loop over parameter list and search for values

  // Check for integer params
  std::string key = this->pIntParams_->firstKey();
  if (key != ""){
    if (sPtr->checkForItem(key)){
      int val = 0;
      sscanf(sPtr->readValue(key).c_str(), "%d", &val);
      debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
      debug(DEBUG_VARIABLE, functionName, "      value", val);
      setIntegerParam(this->pIntParams_->lookup(key), val);
    }
    while (this->pIntParams_->hasNextKey()){
      key = this->pIntParams_->nextKey();
      if (sPtr->checkForItem(key)){
        int val = 0;
        sscanf(sPtr->readValue(key).c_str(), "%d", &val);
        debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
        debug(DEBUG_VARIABLE, functionName, "      value", val);
        setIntegerParam(this->pIntParams_->lookup(key), val);
      }
    }
    callParamCallbacks();
  }

  // Check for double params
  key = this->pDoubleParams_->firstKey();
  if (key != ""){
    if (sPtr->checkForItem(key)){
      double val = 0;
      sscanf(sPtr->readValue(key).c_str(), "%lf", &val);
      debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
      debug(DEBUG_VARIABLE, functionName, "      value", val);
      setDoubleParam(this->pDoubleParams_->lookup(key), val);
    }
    while (this->pDoubleParams_->hasNextKey()){
      key = this->pDoubleParams_->nextKey();
      if (sPtr->checkForItem(key)){
        double val = 0;
        sscanf(sPtr->readValue(key).c_str(), "%lf", &val);
        debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
        debug(DEBUG_VARIABLE, functionName, "      value", val);
        setDoubleParam(this->pDoubleParams_->lookup(key), val);
      }
    }
    callParamCallbacks();
  }

  // Check for string params
  key = this->pStringParams_->firstKey();
  if (key != ""){
    if (sPtr->checkForItem(key)){
      char val[MAX_STRING_SIZE];
      sscanf(sPtr->readValue(key).c_str(), "%s", val);
      debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
      debug(DEBUG_VARIABLE, functionName, "      value", (char *)val);
      setStringParam(this->pStringParams_->lookup(key), val);
    }
    while (this->pStringParams_->hasNextKey()){
      key = this->pStringParams_->nextKey();
      if (sPtr->checkForItem(key)){
        char val[MAX_STRING_SIZE];
        sscanf(sPtr->readValue(key).c_str(), "%s", val);
        debug(DEBUG_VARIABLE, functionName, "Found key  ", key.c_str());
        debug(DEBUG_VARIABLE, functionName, "      value", (char *)val);
        setStringParam(this->pDoubleParams_->lookup(key), val);
      }
    }
    callParamCallbacks();
  }

}

/**
 * Connect to the underlying low level Asyn port that is used for comms.
 * This uses the asynOctetSyncIO interface, and also sets the input and output terminators.
 * @param port The port to connect to
 * @param addr The address of the port to connect to
 * @param ppasynUser A pointer to the pasynUser structure used by the controller
 * @param inputEos The input EOS character
 * @param outputEos The output EOS character
 * @return asynStatus  
 */
/*asynStatus pmacController::lowLevelPortConnect(const char *port, int addr, asynUser **ppasynUser, char *inputEos, char *outputEos)
{
  asynStatus status = asynSuccess;
 
  static const char *functionName = "pmacController::lowLevelPortConnect";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  status = pasynOctetSyncIO->connect( port, addr, ppasynUser, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
	      "pmacController::motorAxisAsynConnect: unable to connect to port %s\n", 
	      port);
    return status;
  }

  //Do I want to disconnect below? If the IP address comes up, will the driver recover
  //if the poller functions are running? Might have to use asynManager->isConnected to
  //test connection status of low level port (in the pollers). But then autosave 
  //restore doesn't work (and we would save wrong positions). So I need to 
  //have a seperate function(s) to deal with connecting after IOC init.

  status = pasynOctetSyncIO->setInputEos(*ppasynUser, inputEos, strlen(inputEos) );
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
	      "pmacController: unable to set input EOS on %s: %s\n", 
	      port, (*ppasynUser)->errorMessage);
    pasynOctetSyncIO->disconnect(*ppasynUser);
    //Set my low level pasynUser pointer to NULL
    *ppasynUser = NULL;
    return status;
  }
  
  status = pasynOctetSyncIO->setOutputEos(*ppasynUser, outputEos, strlen(outputEos));
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
	      "pmacController: unable to set output EOS on %s: %s\n", 
	      port, (*ppasynUser)->errorMessage);
    pasynOctetSyncIO->disconnect(*ppasynUser);
    //Set my low level pasynUser pointer to NULL
    *ppasynUser = NULL;
    return status;
  }
  
  return status;
}*/

/**
 * Utilty function to print the connected status of the low level asyn port.
 * @return asynStatus
 */
/*asynStatus pmacController::printConnectedStatus()
{
  asynStatus status = asynSuccess;
  int asynManagerConnected = 0;
  static const char *functionName = "pmacController::printConnectedStatus";
  
  if (lowLevelPortUser_) {
    status = pasynManager->isConnected(lowLevelPortUser_, &asynManagerConnected);
      if (status!=asynSuccess) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
		"pmacController: Error calling pasynManager::isConnected.\n");
      return status;
      } else {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s isConnected: %d\n", functionName, asynManagerConnected);
    }
  }
  return status;
}*/

asynStatus pmacController::immediateWriteRead(const char *command, char *response)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "immediateWriteRead";
  this->startTimer(DEBUG_TIMING, functionName);
  status = this->lowLevelWriteRead(command, response);
  this->stopTimer(DEBUG_TIMING, functionName, "PMAC write/read time");
  return status;
}

/**
 * Wrapper for asynOctetSyncIO write/read functions.
 * @param command - String command to send.
 * @response response - String response back.
 */
asynStatus pmacController::lowLevelWriteRead(const char *command, char *response)
{
  //asynStatus status = asynSuccess;
  //int eomReason = 0;
  //size_t nwrite = 0;
  //size_t nread = 0;
  //int commsError = 0;
  static const char *functionName = "lowLevelWriteRead";

  debug(DEBUG_FLOW, functionName);

  return pBroker_->immediateWriteRead(command, response);

  //asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);
  //printf("Writing: %s\n", command);

/*  if (!lowLevelPortUser_) {
    setIntegerParam(PMAC_C_CommsError_, PMAC_ERROR_);
    return asynError;
  }
  
  asynPrint(lowLevelPortUser_, ASYN_TRACEIO_DRIVER, "%s: command: %s\n", functionName, command);
  
  //Make sure the low level port is connected before we attempt comms
  //Use the controller-wide param PMAC_C_CommsError_
  getIntegerParam(PMAC_C_CommsError_, &commsError);
  
  if (!commsError) {
    status = pasynOctetSyncIO->writeRead(lowLevelPortUser_ ,
					 command, strlen(command),
					 response, PMAC_MAXBUF_,
					 PMAC_TIMEOUT_,
					 &nwrite, &nread, &eomReason );
    
    if (status) {
      asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, "%s: Error from pasynOctetSyncIO->writeRead. command: %s\n", functionName, command);
      setIntegerParam(PMAC_C_CommsError_, PMAC_ERROR_);
    } else {
      setIntegerParam(PMAC_C_CommsError_, PMAC_OK_);
    }
  }
  
  asynPrint(lowLevelPortUser_, ASYN_TRACEIO_DRIVER, "%s: response: %s\n", functionName, response); 
  
  return status;*/
}


void pmacController::report(FILE *fp, int level)
{
  int axis = 0;
  pmacAxis *pAxis = NULL;

  fprintf(fp, "pmac motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
          this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
    for (axis=0; axis<numAxes_; axis++) {
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
asynStatus pmacController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
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
    epicsInt32 position = (epicsInt32) floor(value*32/pAxis->scale_ + 0.5);
    
    sprintf(command, "#%dK M%d61=%d*I%d08 M%d62=%d*I%d08",
	     pAxis->axisNo_,
	     pAxis->axisNo_, position, pAxis->axisNo_, 
	     pAxis->axisNo_, position, pAxis->axisNo_ );

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
	      "%s: Set axis %d on controller %s to position %f\n", 
	      functionName, pAxis->axisNo_, portName, value);
    
    if ( command[0] != 0 && status) {
      status = (lowLevelWriteRead(command, response) == asynSuccess) && status;
    }
                
    sprintf(command, "#%dJ/", pAxis->axisNo_);

    if (command[0] != 0 && status) {
      status = (lowLevelWriteRead(command, response) == asynSuccess) && status;
    }

    /*Now set position on encoder axis, if one is in use.*/
                
    if (pAxis->encoder_axis_) {
      getDoubleParam(motorEncoderRatio_,  &encRatio);
      encposition = (epicsInt32) floor((position*encRatio) + 0.5);
                  
      sprintf(command, "#%dK M%d61=%d*I%d08 M%d62=%d*I%d08",
	      pAxis->encoder_axis_,
	      pAxis->encoder_axis_, encposition, pAxis->encoder_axis_, 
	      pAxis->encoder_axis_, encposition, pAxis->encoder_axis_ );
                  
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
		"%s: Set encoder axis %d on controller %s to position %f\n", 
		functionName, pAxis->axisNo_, portName, value);
                  
      if (command[0] != 0 && status) {
	status = (lowLevelWriteRead(command, response) == asynSuccess) && status;
      }
                  
      sprintf(command, "#%dJ/", pAxis->encoder_axis_);
      //The lowLevelWriteRead will be done at the end of this function.
    }

    /*Now do an update, to get the new position from the controller.*/
    bool moving = true;
    pAxis->getAxisStatus(&moving);
  } 
  else if (function == motorLowLimit_) {
    sprintf(command, "I%d14=%f", pAxis->axisNo_, value/pAxis->scale_);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
	      "%s: Setting low limit on controller %s, axis %d to %f\n",
	      functionName, portName, pAxis->axisNo_, value);
  }
  else if (function == motorHighLimit_) {
    sprintf(command, "I%d13=%f", pAxis->axisNo_, value/pAxis->scale_);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
	      "%s: Setting high limit on controller %s, axis %d to %f\n",
	      functionName, portName, pAxis->axisNo_, value);
  } else if (pWriteParams_->hasKey(*name)){
    // This is an integer write of a parameter, so send the immediate write/read
    sprintf(command, "%s=%f", pWriteParams_->lookup(*name).c_str(), value);
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

/** Called when asyn clients call pasynOctet->write().
  * This function performs actions for some parameters, including AttributesFile.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the string to write.
  * \param[in] nChars Number of characters to write.
  * \param[out] nActual Number of characters actually written. */
asynStatus pmacController::writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual)
{
  int addr=0;
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  char command[PMAC_MAXBUF_] = {0};
  char response[PMAC_MAXBUF_] = {0};
  const char *functionName = "writeOctet";

  status = getAddress(pasynUser, &addr); if (status != asynSuccess) return(status);
  // Set the parameter in the parameter library.
  status = (asynStatus)setStringParam(addr, function, (char *)value);
  if (status != asynSuccess) return(status);

  if (function == PMAC_C_WriteCmd_){
    // Write the arbitrary string to the PMAC, ignoring a reponse
    strcpy(command, value);
    status = this->immediateWriteRead(command, response);
  }

  // Do callbacks so higher layers see any changes
  callParamCallbacks(addr, addr);

  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = asynMotorController::writeOctet(pasynUser, value, nChars, nActual);

  if (status != asynSuccess){
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
asynStatus pmacController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  char command[PMAC_MAXBUF_] = {0};
  char response[PMAC_MAXBUF_] = {0};
  bool status = true;
  pmacAxis *pAxis = NULL;
  const char *name[128];
  static const char *functionName = "writeInt32";

  debug(DEBUG_ERROR, functionName);

  getParamName(function, name);
  debug(DEBUG_VARIABLE, functionName, "Parameter Updated", *name);
  pAxis = this->getAxis(pasynUser);
  if (!pAxis) {
    return asynError;
  } 

  status = (pAxis->setIntegerParam(function, value) == asynSuccess) && status;

  if (function == PMAC_C_FeedRatePoll_) {
    if (value) {
      this->feedRatePoll_ = true;
    } else {
      this->feedRatePoll_ = false;
    }
  } 
  else if (function == PMAC_C_FeedRate_) {
    sprintf(command, "%%%d", value);
    if (command[0] != 0) {
      //PMAC does not respond to this command.
      lowLevelWriteRead(command, response);
    }
  }
  else if (function == motorDeferMoves_) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s: Setting deferred move mode on PMAC %s to %d\n", functionName, portName, value);
    if (value == 0 && this->movesDeferred_ == DEFERRED_FAST_MOVES) {
      status = (this->processDeferredMoves() == asynSuccess) && status;
    }
    else if (value == 0 && this->movesDeferred_ == DEFERRED_COORDINATED_MOVES) {
    	status = (pGroupList->processDeferredCoordMoves() == asynSuccess) && status;
    }
    this->movesDeferred_ = value;
  }
  else if (function == PMAC_C_CoordSysGroup_) {
	  status = (pGroupList->switchToGroup(value) == asynSuccess) && status;
  } else if (pWriteParams_->hasKey(*name)){
    // This is an integer write of a parameter, so send the immediate write/read
    sprintf(command, "%s=%d", pWriteParams_->lookup(*name).c_str(), value);
    debug(DEBUG_ERROR, functionName, "Command sent to PMAC", command);
    status = (this->immediateWriteRead(command, response) == asynSuccess) && status;
  } else if (function == PMAC_C_KillAxis_){
    // Send the kill command to the PMAC immediately
    sprintf(command, "#%dk", pAxis->axisNo_);
    status = (this->immediateWriteRead(command, response) == asynSuccess) && status;
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


/** Returns a pointer to an pmacAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
pmacAxis* pmacController::getAxis(asynUser *pasynUser)
{
  int axisNo = 0;
    
  getAddress(pasynUser, &axisNo);
  return getAxis(axisNo);
}



/** Returns a pointer to an pmacAxis object.
  * Returns NULL if the axis number is invalid.
  * \param[in] axisNo Axis index number. */
pmacAxis* pmacController::getAxis(int axisNo)
{
  if ((axisNo < 0) || (axisNo >= numAxes_)) return NULL;
  return pAxes_[axisNo];
}


/** 
 * Polls the controller, rather than individual axis.
 * @return asynStatus
 */
asynStatus pmacController::poll()
{
  char tBuff[32];
  static const char *functionName = "poll";
  debug(DEBUG_FLOW, functionName);

  epicsTimeGetCurrent(&nowTime_);
  // Always call for a fast update
  debug(DEBUG_TRACE, functionName, "Fast update has been called", tBuff);
  pBroker_->updateVariables(pmacMessageBroker::PMAC_FAST_READ);
  setDoubleParam(PMAC_C_FastUpdateTime_, pBroker_->readUpdateTime());

  if (epicsTimeDiffInSeconds(&nowTime_, &lastMediumTime_) >= PMAC_MEDIUM_LOOP_TIME/1000.0){
    epicsTimeAddSeconds(&lastMediumTime_, PMAC_MEDIUM_LOOP_TIME/1000.0);
    epicsTimeToStrftime(tBuff, 32, "%Y/%m/%d %H:%M:%S.%03f", &nowTime_);
    debug(DEBUG_TRACE, functionName, "Medium update has been called", tBuff);
    pBroker_->updateVariables(pmacMessageBroker::PMAC_MEDIUM_READ);
  }
  if (epicsTimeDiffInSeconds(&nowTime_, &lastSlowTime_) >= PMAC_SLOW_LOOP_TIME/1000.0){
    epicsTimeAddSeconds(&lastSlowTime_, PMAC_SLOW_LOOP_TIME/1000.0);
    epicsTimeToStrftime(tBuff, 32, "%Y/%m/%d %H:%M:%S.%03f", &nowTime_);
    debug(DEBUG_TRACE, functionName, "Slow update has been called", tBuff);
    pBroker_->updateVariables(pmacMessageBroker::PMAC_SLOW_READ);
  }

  return asynSuccess;

  /*
  if (!lowLevelPortUser_) {
    return asynError;
  }
*/

  /* Get the time and decide if we want to print errors.*/
/*  epicsTimeGetCurrent(&nowTime_);
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
  
  //Set any controller specific parameters. 
  //Some of these may be used by the axis poll to set axis problem bits.
  status = (getGlobalStatus(&globalStatus, &feedrate, feedRatePoll_) == asynSuccess) && status;
  hardwareProblem = ((globalStatus & PMAC_HARDWARE_PROB) != 0);
  status = (setIntegerParam(this->PMAC_C_GlobalStatus_, hardwareProblem) == asynSuccess) && status;
  if(hardwareProblem){
	  asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: *** Hardware Problem *** globalstatus = %x\n",
			  functionName, globalStatus);
  }

  if (status && feedRatePoll_) {
    status = (setIntegerParam(this->PMAC_C_FeedRate_, feedrate) == asynSuccess) && status;
    status = (getIntegerParam(this->PMAC_C_FeedRateLimit_, &feedrate_limit) == asynSuccess) && status;
    if (feedrate < static_cast<int>(feedrate_limit-PMAC_FEEDRATE_DEADBAND_)) {
      status = (setIntegerParam(this->PMAC_C_FeedRateProblem_, PMAC_ERROR_) == asynSuccess) && status;
      if (printErrors) {
	asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, 
		  "%s: *** ERROR ***: global feed rate below limit. feedrate: %d, feedrate limit %d\n", functionName, feedrate, feedrate_limit);
	printNextError_ = false;
      }
    } else {
      status = (setIntegerParam(this->PMAC_C_FeedRateProblem_, PMAC_OK_) == asynSuccess) && status;
      printNextError_ = true;
    }
  }
  
  callParamCallbacks();

  if (!status) {
    asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, "%s: Error reading or setting params.\n", functionName);
    setIntegerParam(PMAC_C_CommsError_, PMAC_ERROR_);
    return asynError;
  } else {
    setIntegerParam(PMAC_C_CommsError_, PMAC_OK_);
    return asynSuccess;
  }*/
}


asynStatus pmacController::newGetGlobalStatus(pmacCommandStore *sPtr)
{
  asynStatus status = asynSuccess;
  epicsUInt32 globalStatus = 0;
  int feedrate = 0;
  int feedrate_limit = 0;
  bool printErrors = 0;
  bool hardwareProblem;
  int nvals;
  static const char *functionName = "getGlobalStatus";

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

  // Lookup the value of global status
  std::string globStatus = sPtr->readValue("???");
  debug(DEBUG_VARIABLE, functionName, "Global status [???]", globStatus);

  // Check the global status value is valid
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
  }

  // Lookup the value of the feedrate
  std::string feedRate = sPtr->readValue("%");
  debug(DEBUG_VARIABLE, functionName, "Feedrate [%]", feedRate);

  // Check the feedrate value is valid
  if (feedRate == ""){
    debug(DEBUG_ERROR, functionName, "Problem reading feed rate command %");
    status = asynError;
  } else {
    nvals = sscanf(feedRate.c_str(), "%d", &feedrate);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading feedrate [%]");
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", feedRate);
      status = asynError;
    }
  }

  //Set any controller specific parameters.
  //Some of these may be used by the axis poll to set axis problem bits.
  if (status == asynSuccess){
    hardwareProblem = ((globalStatus & PMAC_HARDWARE_PROB) != 0);
    status = setIntegerParam(this->PMAC_C_GlobalStatus_, hardwareProblem);
    if(hardwareProblem){
      debug(DEBUG_ERROR, functionName, "*** Hardware Problem *** global status [???]", (int)globalStatus);
    }
  }
  if (status == asynSuccess){
    status = setIntegerParam(this->PMAC_C_FeedRate_, feedrate);
  }
  if (status == asynSuccess){
    status = getIntegerParam(this->PMAC_C_FeedRateLimit_, &feedrate_limit);
  }

  if (status == asynSuccess){
    if (feedrate < static_cast<int>(feedrate_limit-PMAC_FEEDRATE_DEADBAND_)) {
      status = setIntegerParam(this->PMAC_C_FeedRateProblem_, PMAC_ERROR_);
      if (printErrors){
        debug(DEBUG_ERROR, functionName, "*** ERROR ***: global feed rate below limit.");
        debug(DEBUG_ERROR, functionName, "               feedrate", feedrate);
        debug(DEBUG_ERROR, functionName, "               feedrate limit", feedrate_limit);
        printNextError_ = false;
      }
    } else {
      status = setIntegerParam(this->PMAC_C_FeedRateProblem_, PMAC_OK_);
      printNextError_ = true;
    }
  }

  callParamCallbacks();

  if (status != asynSuccess){
    debug(DEBUG_ERROR, functionName, "Error reading or setting params.");
    setIntegerParam(PMAC_C_CommsError_, PMAC_ERROR_);
  } else {
    setIntegerParam(PMAC_C_CommsError_, PMAC_OK_);
  }

  return status;
}

/**
 * Read the PMAC global status integer (using a ??? ) and global feed rate (%)
 * @param int The global status integer (23 active bits)
 * @param int The global feed rate
 */
/*
asynStatus pmacController::getGlobalStatus(epicsUInt32 *globalStatus, int *feedrate, int feedrate_poll)
{
  char command[PMAC_MAXBUF_];
  char response[PMAC_MAXBUF_];
  int nvals = 0;
  asynStatus status = asynSuccess;
  static const char *functionName = "pmacController::getGlobalStatus";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);
  
  sprintf(command, "???");
  if (lowLevelWriteRead(command, response) != asynSuccess) {
    asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, "%s: Error reading ???.\n", functionName);
    status = asynError;
  } else {
    nvals = sscanf(response, "%6x", globalStatus);
    if (nvals != 1) {
      asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, "%s: Error reading ???. nvals: %d, response: %s\n", functionName, nvals, response);
      status = asynError;
    } else {
      status = asynSuccess;
    }
  }

  if (feedrate_poll) {
    sprintf(command, "%%");
    if (lowLevelWriteRead(command, response) != asynSuccess) {
      asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, "%s: Error reading feedrate.\n", functionName);
      status = asynError;
    } else {
      nvals = sscanf(response, "%d", feedrate);
      if (nvals != 1) {
	asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, "%s: Error reading feedrate: nvals: %d, response: %s\n", functionName, nvals, response);
	status = asynError;
      } else {
	status = asynSuccess;
      }
    }
  }
  
  if (status == asynSuccess) {
    setIntegerParam(PMAC_C_CommsError_, PMAC_OK_);
  } else {
    setIntegerParam(PMAC_C_CommsError_, PMAC_ERROR_);
  }

  return status;

}
*/

/**
 * Disable the check in the axis poller that reads ix24 to check if hardware limits
 * are disabled. By default this is enabled for safety reasons. It sets the motor
 * record PROBLEM bit in MSTA, which results in the record going into MAJOR/STATE alarm.
 * @param axis Axis number to disable the check for.
 */
asynStatus pmacController::pmacDisableLimitsCheck(int axis) 
{
  pmacAxis *pA = NULL;
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
	      "%s: Error: axis %d has not been configured using pmacCreateAxis.\n", functionName, axis);
    return asynError;
  }
  this->unlock();
  return asynSuccess;
}

/**
 * Disable the check in the axis poller that reads ix24 to check if hardware limits
 * are disabled. By default this is enabled for safety reasons. It sets the motor
 * record PROBLEM bit in MSTA, which results in the record going into MAJOR/STATE alarm.
 * This function will disable the check for all axes on this controller.
 */
asynStatus pmacController::pmacDisableLimitsCheck(void) 
{
  pmacAxis *pA = NULL;
  static const char *functionName = "pmacController::pmacDisableLimitsCheck";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  this->lock();
  for (int i=1; i<numAxes_; i++) {
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
asynStatus pmacController::pmacSetAxisScale(int axis, int scale) 
{
  pmacAxis *pA = NULL;
  static const char *functionName = "pmacController::pmacSetAxisScale";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (scale < 1) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: Error: scale factor must be >=1.\n", functionName);
    return asynError;
  }

  this->lock();
  pA = getAxis(axis);
  if (pA) {
    pA->scale_ = scale;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s. Setting scale factor of %d on axis %d, on controller %s.\n",
              functionName, pA->scale_, pA->axisNo_, portName);

  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: Error: axis %d has not been configured using pmacCreateAxis.\n", functionName, axis);
    return asynError;
  }
  this->unlock();
  return asynSuccess;
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
asynStatus pmacController::pmacSetOpenLoopEncoderAxis(int axis, int encoder_axis)
{
  pmacAxis *pA = NULL;
  static const char *functionName = "pmacController::pmacSetOpenLoopEncoderAxis";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  this->lock();
  pA = getAxis(axis);
  if (pA) {
    //Test that the encoder axis has also been configured
    if (getAxis(encoder_axis) == NULL) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
		"%s: Error: encoder axis %d has not been configured using pmacCreateAxis.\n", functionName, encoder_axis);
      return asynError;
    }
    pA->encoder_axis_ = encoder_axis;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s. Setting encoder axis %d for axis %d, on controller %s.\n",
              functionName, pA->encoder_axis_, pA->axisNo_, portName);

  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: Error: axis %d has not been configured using pmacCreateAxis.\n", functionName, axis);
    return asynError;
  }
  this->unlock();
  return asynSuccess;
}

asynStatus pmacController::registerForCallbacks(pmacCallbackInterface *cbPtr, int type)
{
  return pBroker_->registerForUpdates(cbPtr, type);
}

asynStatus pmacController::monitorPMACVariable(int poll_speed, const char *var)
{
  return pBroker_->addReadVariable(poll_speed, var);
}

asynStatus pmacController::registerCS(pmacCSController *csPtr, int csNo)
{
  char statVar[8];
  static const char *functionName = "registerCS";

  debug(DEBUG_VARIABLE, functionName, "Registering CS", csNo);

  // Add the CS to the list
  pCSControllers_[csNo] = csPtr;
  pAxisZero->registerCS(csPtr, csNo);

  // First add the CS status item to the fast update
  sprintf(statVar, "&%d??", csNo);
  this->pBroker_->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, statVar);

  // Now register the CS object for callbacks from the broker
  this->pBroker_->registerForUpdates(csPtr, pmacMessageBroker::PMAC_FAST_READ);

  return asynSuccess;
}

asynStatus pmacController::processDeferredMoves(void)
{
  asynStatus status = asynSuccess;
  char command[PMAC_MAXBUF_] = {0};
  char response[PMAC_MAXBUF_] = {0};
  pmacAxis *pAxis = NULL;
  static const char *functionName = "pmacController::processDeferredMoves";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  //Build up combined move command for all axes involved in the deferred move.
  for (int axis=0; axis<numAxes_; axis++) {
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
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s ERROR Sending Deferred Move Command.\n", functionName);
    setIntegerParam(PMAC_C_CommsError_, PMAC_ERROR_);
    status = asynError;
  } else {
    setIntegerParam(PMAC_C_CommsError_, PMAC_OK_);
    status = asynSuccess;
  }

  //Clear deferred move flag for the axes involved.
  for (int axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (pAxis!=NULL) {
      if (pAxis->deferredMove_) {
	pAxis->deferredMove_ = 0;
      }
    }
  }
     
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
asynStatus pmacCreateController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, 
				int numAxes, int movingPollPeriod, int idlePollPeriod)
{

    pmacController *ppmacController
      = new pmacController(portName, lowLevelPortName, lowLevelPortAddress, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
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
  pmacAxis *pAxis;

  static const char *functionName = "pmacCreateAxis";

  pC = (pmacController*) findAsynPortDriver(pmacName);
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
  pAxis = new pmacAxis(pC, axis);
  pAxis = NULL;
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
			  int numAxes)                   
{
  pmacController *pC;
  pmacAxis *pAxis;

  static const char *functionName = "pmacCreateAxis";

  pC = (pmacController*) findAsynPortDriver(pmacName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, pmacName);
    return asynError;
  }
  
  pC->lock();
  for (int axis=1; axis<=numAxes; axis++) {
    pAxis = new pmacAxis(pC, axis);
    pAxis = NULL;
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
asynStatus pmacDisableLimitsCheck(const char *controller, int axis, int allAxes)
{
  pmacController *pC;
  static const char *functionName = "pmacDisableLimitsCheck";

  pC = (pmacController*) findAsynPortDriver(controller);
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
asynStatus pmacSetAxisScale(const char *controller, int axis, int scale)
{
  pmacController *pC;
  static const char *functionName = "pmacSetAxisScale";

  pC = (pmacController*) findAsynPortDriver(controller);
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
asynStatus pmacSetOpenLoopEncoderAxis(const char *controller, int axis, int encoder_axis)
{
  pmacController *pC;
  static const char *functionName = "pmacSetOpenLoopEncoderAxis";

  pC = (pmacController*) findAsynPortDriver(controller);
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
asynStatus pmacCreateCsGroup(const char *controller, int groupNo, char* groupName, int axisCount)
{
	pmacController *pC;
	static const char *functionName = "pmacCreateCsGroup";

	pC = (pmacController*) findAsynPortDriver(controller);
	if (!pC)
	{
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
asynStatus pmacCsGroupAddAxis(const char *controller, int groupNo, int axisNo, char *mapping, int coordinateSysNo)
{
  pmacController *pC;
  static const char *functionName = "pmacCsGroupAddAxis";

  pC = (pmacController*) findAsynPortDriver(controller);
  if (!pC)
  {
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
asynStatus pmacDebug(const char *controller, int level, int axis)
{
  pmacController *pC;
  static const char *functionName = "pmacDebug";

  pC = (pmacController*) findAsynPortDriver(controller);
  if (!pC)
  {
    printf("%s:%s: Error port %s not found\n",
         driverName, functionName, controller);
    return asynError;
  }

  pC->setDebugLevel(level, axis);

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
static const iocshArg * const pmacCreateControllerArgs[] = {&pmacCreateControllerArg0,
							    &pmacCreateControllerArg1,
							    &pmacCreateControllerArg2,
							    &pmacCreateControllerArg3,
							    &pmacCreateControllerArg4,
							    &pmacCreateControllerArg5};
static const iocshFuncDef configpmacCreateController = {"pmacCreateController", 6, pmacCreateControllerArgs};
static void configpmacCreateControllerCallFunc(const iocshArgBuf *args)
{
  pmacCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}


/* pmacCreateAxis */
static const iocshArg pmacCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg * const pmacCreateAxisArgs[] = {&pmacCreateAxisArg0,
                                                     &pmacCreateAxisArg1};
static const iocshFuncDef configpmacAxis = {"pmacCreateAxis", 2, pmacCreateAxisArgs};

static void configpmacAxisCallFunc(const iocshArgBuf *args)
{
  pmacCreateAxis(args[0].sval, args[1].ival);
}

/* pmacCreateAxes */
static const iocshArg pmacCreateAxesArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateAxesArg1 = {"Num Axes", iocshArgInt};
static const iocshArg * const pmacCreateAxesArgs[] = {&pmacCreateAxesArg0,
                                                     &pmacCreateAxesArg1};
static const iocshFuncDef configpmacAxes = {"pmacCreateAxes", 2, pmacCreateAxesArgs};

static void configpmacAxesCallFunc(const iocshArgBuf *args)
{
  pmacCreateAxes(args[0].sval, args[1].ival);
}


/* pmacDisableLimitsCheck */
static const iocshArg pmacDisableLimitsCheckArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacDisableLimitsCheckArg1 = {"Axis number", iocshArgInt};
static const iocshArg pmacDisableLimitsCheckArg2 = {"All Axes", iocshArgInt};
static const iocshArg * const pmacDisableLimitsCheckArgs[] = {&pmacDisableLimitsCheckArg0,
							      &pmacDisableLimitsCheckArg1,
							      &pmacDisableLimitsCheckArg2};
static const iocshFuncDef configpmacDisableLimitsCheck = {"pmacDisableLimitsCheck", 3, pmacDisableLimitsCheckArgs};

static void configpmacDisableLimitsCheckCallFunc(const iocshArgBuf *args)
{
  pmacDisableLimitsCheck(args[0].sval, args[1].ival, args[2].ival);
}



/* pmacSetAxisScale */
static const iocshArg pmacSetAxisScaleArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacSetAxisScaleArg1 = {"Axis number", iocshArgInt};
static const iocshArg pmacSetAxisScaleArg2 = {"Scale", iocshArgInt};
static const iocshArg * const pmacSetAxisScaleArgs[] = {&pmacSetAxisScaleArg0,
							      &pmacSetAxisScaleArg1,
							      &pmacSetAxisScaleArg2};
static const iocshFuncDef configpmacSetAxisScale = {"pmacSetAxisScale", 3, pmacSetAxisScaleArgs};

static void configpmacSetAxisScaleCallFunc(const iocshArgBuf *args)
{
  pmacSetAxisScale(args[0].sval, args[1].ival, args[2].ival);
}

/* pmacSetOpenLoopEncoderAxis */
static const iocshArg pmacSetOpenLoopEncoderAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacSetOpenLoopEncoderAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg pmacSetOpenLoopEncoderAxisArg2 = {"Encoder Axis", iocshArgInt};
static const iocshArg * const pmacSetOpenLoopEncoderAxisArgs[] = {&pmacSetOpenLoopEncoderAxisArg0,
								  &pmacSetOpenLoopEncoderAxisArg1,
								  &pmacSetOpenLoopEncoderAxisArg2};
static const iocshFuncDef configpmacSetOpenLoopEncoderAxis = {"pmacSetOpenLoopEncoderAxis", 3, pmacSetOpenLoopEncoderAxisArgs};

static void configpmacSetOpenLoopEncoderAxisCallFunc(const iocshArgBuf *args)
{
  pmacSetOpenLoopEncoderAxis(args[0].sval, args[1].ival, args[2].ival);
}

/* pmacCreateCsGroup */
static const iocshArg pmacCreateCsGroupArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCreateCsGroupArg1 = {"Group number", iocshArgInt};
static const iocshArg pmacCreateCsGroupArg2 = {"Group name", iocshArgString};
static const iocshArg pmacCreateCsGroupArg3 = {"Axis count", iocshArgInt};
static const iocshArg * const pmacCreateCsGroupArgs[] = {&pmacCreateCsGroupArg0,
								  &pmacCreateCsGroupArg1,
								  &pmacCreateCsGroupArg2,
								  &pmacCreateCsGroupArg3};
static const iocshFuncDef configpmacCreateCsGroup = {"pmacCreateCsGroup", 4, pmacCreateCsGroupArgs};

static void configpmacCreateCsGroupCallFunc(const iocshArgBuf *args)
{
	pmacCreateCsGroup(args[0].sval, args[1].ival, args[2].sval, args[3].ival);
}

/* pmacCsGroupAddAxis */
static const iocshArg pmacCsGroupAddAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacCsGroupAddAxisArg1 = {"Group number", iocshArgInt};
static const iocshArg pmacCsGroupAddAxisArg2 = {"Axis number", iocshArgInt};
static const iocshArg pmacCsGroupAddAxisArg3 = {"Axis CS definition", iocshArgString};
static const iocshArg pmacCsGroupAddAxisArg4 = {"CS number", iocshArgInt};
static const iocshArg * const pmacCsGroupAddAxisArgs[] = {&pmacCsGroupAddAxisArg0,
                  &pmacCsGroupAddAxisArg1,
                  &pmacCsGroupAddAxisArg2,
                  &pmacCsGroupAddAxisArg3,
                  &pmacCsGroupAddAxisArg4};
static const iocshFuncDef configpmacCsGroupAddAxis = {"pmacCsGroupAddAxis", 5, pmacCsGroupAddAxisArgs};

static void configpmacCsGroupAddAxisCallFunc(const iocshArgBuf *args)
{
  pmacCsGroupAddAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval, args[4].ival);
}

/* pmacCsGroupAddAxis */
static const iocshArg pmacDebugArg0 = {"Controller port name", iocshArgString};
static const iocshArg pmacDebugArg1 = {"Debug level", iocshArgInt};
static const iocshArg pmacDebugArg2 = {"Axis number", iocshArgInt};
static const iocshArg * const pmacDebugArgs[] = {&pmacDebugArg0,
                                                 &pmacDebugArg1,
                                                 &pmacDebugArg2};
static const iocshFuncDef configpmacDebug = {"pmacDebug", 3, pmacDebugArgs};

static void configpmacDebugCallFunc(const iocshArgBuf *args)
{
  pmacDebug(args[0].sval, args[1].ival, args[2].ival);
}


static void pmacControllerRegister(void)
{
  iocshRegister(&configpmacCreateController,       configpmacCreateControllerCallFunc);
  iocshRegister(&configpmacAxis,                   configpmacAxisCallFunc);
  iocshRegister(&configpmacAxes,                   configpmacAxesCallFunc);
  iocshRegister(&configpmacDisableLimitsCheck,     configpmacDisableLimitsCheckCallFunc);
  iocshRegister(&configpmacSetAxisScale,           configpmacSetAxisScaleCallFunc);
  iocshRegister(&configpmacSetOpenLoopEncoderAxis, configpmacSetOpenLoopEncoderAxisCallFunc);
  iocshRegister(&configpmacCreateCsGroup,          configpmacCreateCsGroupCallFunc);
  iocshRegister(&configpmacCsGroupAddAxis,         configpmacCsGroupAddAxisCallFunc);
  iocshRegister(&configpmacDebug,                  configpmacDebugCallFunc);
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

