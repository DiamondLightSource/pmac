/********************************************
 *  pmacAxis.cpp
 *
 *  PMAC Asyn motor based on the
 *  asynMotorAxis class.
 *
 *  Matthew Pearson
 *  23 May 2012
 *
 ********************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsExit.h>
#include <epicsString.h>
#include <iocsh.h>

#include "pmacController.h"
#include <iostream>
#include <sstream>

using std::cout;
using std::endl;

#include <epicsExport.h>

/////////////////replace with a runtime function that can be called on IOC shell.////////////////////////
/////////////////Or, provide an overloaded constructor with this as an argument.////////////////////////
/* This #define affects the behaviour of the driver.

   REMOVE_LIMITS_ON_HOME removes the limits protection when homing if
    - ms??,i912 indicates you are homing onto a limit
    - ms??,i913 and the home velocity indicate that the limit you trigger.
      for home detection is the one you are homing towards.
    - any home offset is in the opposite sense to the home velocity.
*/

#define REMOVE_LIMITS_ON_HOME

static void shutdownCallback(void *pPvt) {
  pmacController *pC = static_cast<pmacController *>(pPvt);

  pC->lock();
  pC->shuttingDown_ = 1;
  pC->unlock();
}

/**
 * pmacAxis constructor.
 * @param pC Pointer to a pmacController object.
 * @param axisNo The axis number for this pmacAxis (1 based).
 */
pmacAxis::pmacAxis(pmacController *pC, int axisNo)
        : asynMotorAxis((asynMotorController *) pC, axisNo),
          pmacDebugger("pmacAxis"),
          pC_(pC) {
  static const char *functionName = "pmacAxis::pmacAxis";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  //Initialize non-static data members
  assignedCS_ = 0;
  resolution_ = 1.0;
  offset_ = 0.0;
  setpointPosition_ = 0.0;
  encoderPosition_ = 0.0;
  currentVelocity_ = 0.0;
  velocity_ = 0.0;
  accel_ = 0.0;
  highLimit_ = 0.0;
  lowLimit_ = 0.0;
  limitsDisabled_ = 0;
  stepSize_ = 1; //Don't need?
  deferredPosition_ = 0.0;
  cachedPosition_ = 0.0;
  deferredMove_ = 0;
  deferredRelative_ = 0;
  deferredTime_ = 0;
  scale_ = 1;
  rawPosition_ = 0.0;
  initiatedMove_ = false;
  csRawMoveInitiated_ = false;
  previous_position_ = 0.0;
  previous_direction_ = 0;
  amp_enabled_ = 0;
  amp_enabled_prev_ = 0;
  fatal_following_ = 0;
  encoder_axis_ = 0;
  limitsCheckDisable_ = 0;
  nowTimeSecs_ = 0.0;
  lastTimeSecs_ = 0.0;
  printNextError_ = false;
  moving_ = false;
  connected_ = true;
  initialised_ = false;

  /* Set an EPICS exit handler that will shut down polling before asyn kills the IP sockets */
  epicsAtExit(shutdownCallback, pC_);

  initialSetup(axisNo_);

  /* Wake up the poller task which will make it do a poll,
   * updating values for this axis to use the new resolution (stepSize_) */
  pC_->wakeupPoller();
}

double pmacAxis::getScale()
{
  return this->scale_;
}

void pmacAxis::badConnection() {
  setIntegerParam(pC_->motorStatusProblem_, true);
  setIntegerParam(pC_->motorStatusCommsError_, true);
  connected_ = false;
  statusChanged_ = 1;
  callParamCallbacks();
}

void pmacAxis::goodConnection() {
  setIntegerParam(pC_->motorStatusProblem_, false);
  setIntegerParam(pC_->motorStatusCommsError_, false);
  connected_ = true;
  statusChanged_ = 1;
  // We must send the value of the offset and resolution to the controller to ensure it is correct
  this->setResolution(this->resolution_);
  this->setOffset(this->offset_);
  callParamCallbacks();
}

void pmacAxis::initialSetup(int axisNo) {
  static const char *functionName = "pmacAxis::initialSetup";

  if(pC_->initialised_) {

    //Do an initial poll to get some values from the PMAC
    if (getAxisInitialStatus() != asynSuccess) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s: getAxisInitialStatus failed to return asynSuccess. Controller: %s, Axis: %d.\n",
                functionName, pC_->portName, axisNo_);
    }

    callParamCallbacks();
    if (axisNo > 0) {
      char var[16];
      // Request position readback
      sprintf(var, "#%dP", axisNo);
      pC_->monitorPMACVariable(pmacMessageBroker::PMAC_FAST_READ, var);
      // Request following error readback
      sprintf(var, "#%dF", axisNo);
      pC_->monitorPMACVariable(pmacMessageBroker::PMAC_FAST_READ, var);
      // Request ixx24 readback
      sprintf(var, "i%d24", axisNo);
      pC_->monitorPMACVariable(pmacMessageBroker::PMAC_FAST_READ, var);

      // Setup any specific hardware status items
      pC_->pHardware_->setupAxisStatus(axisNo);

      // important to get status LAST since otherwise there is a race condition
      // with reading position just before the motors stop - leading to the
      // position being slightly out when the motor record gets DMOV = 1

      // Request status readback
      sprintf(var, "#%d?", axisNo);
      pC_->monitorPMACVariable(pmacMessageBroker::PMAC_FAST_READ, var);

      pC_->registerForCallbacks(this, pmacMessageBroker::PMAC_FAST_READ);
    }
    initialised_ = true;
  } else {
    setIntegerParam(pC_->motorStatusProblem_, true);
    setIntegerParam(pC_->motorStatusCommsError_, true);
    callParamCallbacks();
  }
}

/**
 * Poll for initial axis status (soft limits, PID settings).
 * Set parameters needed for correct motor record behaviour.
 * @return asynStatus
 */
asynStatus pmacAxis::getAxisInitialStatus(void) {
  char command[PMAC_MAXBUF] = {0};
  char response[PMAC_MAXBUF] = {0};
  int cmdStatus = 0;
  double low_limit = 0.0;
  double high_limit = 0.0;
  double pgain = 0.0;
  double igain = 0.0;
  double dgain = 0.0;
  int nvals = 0;

  static const char *functionName = "pmacAxis::getAxisInitialStatus";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (axisNo_ != 0) {

    sprintf(command, "I%d13 I%d14 I%d30 I%d31 I%d33", axisNo_, axisNo_, axisNo_, axisNo_, axisNo_);
    cmdStatus = pC_->lowLevelWriteRead(command, response);
    nvals = sscanf(response, "%lf %lf %lf %lf %lf", &high_limit, &low_limit, &pgain, &dgain,
                   &igain);

    if (cmdStatus || nvals != 5) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s: Error: initial status poll failed on axis %d.\n", functionName, axisNo_);
      return asynError;
    } else {
      setDoubleParam(pC_->motorLowLimit_, low_limit * scale_);
      setDoubleParam(pC_->motorHighLimit_, high_limit * scale_);
      setDoubleParam(pC_->motorPGain_, pgain);
      setDoubleParam(pC_->motorIGain_, igain);
      setDoubleParam(pC_->motorDGain_, dgain);
      setIntegerParam(pC_->motorStatusHasEncoder_, 1);
      setIntegerParam(pC_->motorStatusGainSupport_, 1);
    }

    // Read back the current position and store as cached position
    sprintf(command, "#%dP", axisNo_);
    cmdStatus = pC_->lowLevelWriteRead(command, response);
    nvals = sscanf(response, "%lf", &cachedPosition_);

    setIntegerParam(pC_->motorStatusGainSupport_, 1);
  }
  return asynSuccess;
}


pmacAxis::~pmacAxis() {
  //Destructor
}

void pmacAxis::setResolution(double new_resolution)
{
  char command[PMAC_MAXBUF] = {0};
  char response[PMAC_MAXBUF] = {0};
  int p_var = 4800 + axisNo_;
  static const char *functionName = "setResolution";
  debug(DEBUG_TRACE, functionName, "Setting axis resolution", new_resolution);
  this->resolution_ = new_resolution;
  if (this->connected_){
    sprintf(command, "P%d=%f", p_var, this->resolution_);
    debug(DEBUG_TRACE, functionName, "Axis resolution P variable command", command);
    pC_->axisWriteRead(command, response);
  }
}

double pmacAxis::getResolution()
{
  return this->resolution_;
}

void pmacAxis::setOffset(double new_offset)
{
  char command[PMAC_MAXBUF] = {0};
  char response[PMAC_MAXBUF] = {0};
  int p_var = 4900 + axisNo_;
  static const char *functionName = "setOffset";
  debug(DEBUG_TRACE, functionName, "Setting axis offset", new_offset);
  this->offset_ = new_offset;
  if (this->connected_){
    sprintf(command, "P%d=%f", p_var, this->offset_);
    debug(DEBUG_TRACE, functionName, "Axis offset P variable command", command);
    pC_->axisWriteRead(command, response);
  }
}

double pmacAxis::getOffset()
{
  return this->offset_;
}

asynStatus pmacAxis::directMove(double position, double min_velocity, double max_velocity, double acceleration) {
  static const char *functionName = "directMove";
  double raw_position = 0.0;

  // Calculate the real position demand using the resolution and offset and then call the move command
  raw_position = (position - this->offset_) / this->resolution_ * this->scale_;

  std::stringstream ss;
  ss << "Direct move called for motor [" << this->axisNo_ << "] Position: " << position;
  ss << " Min Velocity: " << min_velocity << " Max velocity: " << max_velocity << " Acceleration: " << acceleration << std::endl;
  ss << "Calculated raw position (in counts): " << raw_position;
  debug(DEBUG_TRACE, functionName, ss.str());

  return this->move(raw_position, 0, min_velocity, max_velocity, acceleration);
}

/**
 * See asynMotorAxis::move
 */
asynStatus pmacAxis::move(double position, int relative, double min_velocity, double max_velocity,
                          double acceleration) {
  double distance = 0;
  asynStatus status = asynError;
  static const char *functionName = "move";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (connected_){
    setIntegerParam(pC_->motorStatusMoving_, true);

    char acc_buff[PMAC_MAXBUF] = {0};
    char vel_buff[PMAC_MAXBUF] = {0};
    char command[PMAC_MAXBUF] = {0};
    char response[PMAC_MAXBUF] = {0};

    if (max_velocity != 0) {
      sprintf(vel_buff, "I%d22=%f ", axisNo_, (max_velocity / (scale_ * 1000.0)));
    }
    if (acceleration != 0) {
      if (max_velocity != 0) {
        sprintf(acc_buff, "I%d20=%f ", axisNo_, (fabs(max_velocity / acceleration) * 1000.0));
      }
    }

    if (pC_->movesDeferred_ == 0) {
      sprintf(command, "%s%s#%d %s%.2f", vel_buff, acc_buff, axisNo_,
              (relative ? "J^" : "J="), position / scale_);
    } else { /* deferred moves */
      sprintf(command, "%s%s", vel_buff, acc_buff);
      deferredPosition_ = position / scale_;
      deferredMove_ = pC_->movesDeferred_;
      deferredRelative_ = relative;
      distance = relative ? fabs(position) : fabs(previous_position_ - position);
      deferredTime_ = (max_velocity != 0) ? fabs(distance / max_velocity) * 1000 : 0;
    }

  #ifdef REMOVE_LIMITS_ON_HOME
    if (limitsDisabled_) {
      char buffer[PMAC_MAXBUF] = {0};
      /* Re-enable limits */
      sprintf(buffer, " i%d24=i%d24&$FDFFFF", axisNo_, axisNo_);
      strncat(command, buffer, PMAC_MAXBUF - 1);
      limitsDisabled_ = 0;
    }
  #endif
    debug(DEBUG_TRACE, functionName, "Axis Move command", command);
    status = pC_->axisWriteRead(command, response);

    // Update the cached position
    cachedPosition_ = position / scale_;
    // Notify that a move has been initiated
    initiatedMove_ = true;

    // make sure that pmacController->makeCSDemandsConsistent will know this axis has moved
    int csNum = this->getAxisCSNo();
    if (csNum > 0) {
      csRawMoveInitiated_ = true;
    }
  } else {
    debug(DEBUG_ERROR, functionName, "Cannot move motor, connection lost");
    status = asynError;
  }

  return status;
}


/**
 * See asynMotorAxis::home
 */
asynStatus
pmacAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards) {
  asynStatus status = asynError;
  char command[PMAC_MAXBUF] = {0};
  char response[PMAC_MAXBUF] = {0};
  static const char *functionName = "home";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (connected_){
    sprintf(command, "#%d HOME", axisNo_);

      // make sure that pmacController->makeCSDemandsConsistent will reset the demand for all axes
      int csNum = getAxisCSNo();
      if (csNum > 0) {
          pC_->csResetAllDemands = true;
      }

  #ifdef REMOVE_LIMITS_ON_HOME
    /* If homing onto an end-limit and home velocity is in the right direction, clear limits protection */
    int macro_station = ((axisNo_ - 1) / 2) * 4 + (axisNo_ - 1) % 2;
    int home_type = 0;
    int home_flag = 0;
    int flag_mode = 0;
    int nvals = 0;
    int home_offset = 0;
    int controller_type = 0;
    double home_velocity = 0.0;
    char buffer[PMAC_MAXBUF] = {0};

    /* Discover type of controller */
    strncpy(buffer, "cid", PMAC_MAXBUF);
    status = pC_->lowLevelWriteRead(buffer, response);
    if (status != asynSuccess) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "Controller %s Addr %d. %s: ERROR Reading Controller Type.\n", pC_->portName, axisNo_,
                functionName);
      return asynError;
    }
    nvals = sscanf(response, "%d", &controller_type);

    if (controller_type == pC_->PMAC_CID_GEOBRICK_ || controller_type == pC_->PMAC_CID_CLIPPER_) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
                "Controller %s Addr %d. %s: This is a Geobrick LV.\n", pC_->portName, axisNo_,
                functionName);
    } else if (controller_type == pC_->PMAC_CID_PMAC_) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
                "Controller %s Addr %d. %s: This is a Turbo PMAC 2 Ultralite.\n", pC_->portName,
                axisNo_, functionName);
    } else if (controller_type == pC_->PMAC_CID_POWER_) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
                "Controller %s Addr %d. %s: This is a Power Brick.\n", pC_->portName,
                axisNo_, functionName);
    } else {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "Controller %s Addr %d. %s: ERROR Unknown controller type = %d.\n", pC_->portName,
                axisNo_, functionName, controller_type);
      return asynError;
    }

    if (controller_type == pC_->PMAC_CID_GEOBRICK_
    || controller_type == pC_->PMAC_CID_CLIPPER_
    || controller_type == pC_->PMAC_CID_POWER_)  {
      /* Read home flags and home direction from Geobrick LV */
      if (axisNo_ < 5) {
        sprintf(buffer, "I70%d2 I70%d3 i%d24 i%d23 i%d26", axisNo_, axisNo_, axisNo_, axisNo_,
                axisNo_);
      } else {
        sprintf(buffer, "I71%d2 I71%d3 i%d24 i%d23 i%d26", axisNo_ - 4, axisNo_ - 4, axisNo_, axisNo_,
                axisNo_);
      }
      status = pC_->lowLevelWriteRead(buffer, response);
      nvals = sscanf(response, "%d %d $%x %lf %d", &home_type, &home_flag, &flag_mode, &home_velocity,
                    &home_offset);
    }

    if (controller_type == pC_->PMAC_CID_PMAC_) {
      /* Read home flags and home direction from VME PMAC */
      sprintf(buffer, "ms%d,i912 ms%d,i913 i%d24 i%d23 i%d26", macro_station, macro_station, axisNo_,
              axisNo_, axisNo_);
      status = pC_->lowLevelWriteRead(buffer, response);
      nvals = sscanf(response, "$%x $%x $%x %lf %d", &home_type, &home_flag, &flag_mode,
                    &home_velocity, &home_offset);
    }

    if ((status != asynSuccess) || (nvals != 5)) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "Controller %s Addr %d. %s: ERROR Cannot Read Home Flags.\n", pC_->portName, axisNo_,
                functionName);
      return asynError;
    }

    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
              "Controller %s Addr %d. %s: .home_type = %d, home_flag = %d, flag_mode = %x, home_velocity = %f, home_offset = %d\n",
              pC_->portName, axisNo_, functionName, home_type, home_flag, flag_mode, home_velocity,
              home_offset);

    if (max_velocity != 0) {
      home_velocity = (forwards ? 1 : -1) * (fabs(max_velocity) / 1000.0);
    }

    if ((home_type <= 15) &&
        (home_type % 4 >= 2) &&
        !(flag_mode & 0x20000) &&
        ((home_velocity > 0 && home_flag == 1 && home_offset <= 0) ||
        (home_velocity < 0 && home_flag == 2 && home_offset >= 0))) {
      sprintf(buffer, " i%d24=i%d24|$20000", axisNo_, axisNo_);
      strncat(command, buffer, PMAC_MAXBUF - 1);
      limitsDisabled_ = 1;
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s. Disabling limits whilst homing PMAC controller %s, axis %d, type:%d, flag:$%x, vel:%f\n",
                functionName, pC_->portName, axisNo_, home_type, home_flag, home_velocity);
    } else {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s: Error: Cannot disable limits to home PMAC controller %s, axis %d, type:%x, flag:$%d, vel:%f, mode:0x%x, offset: %d\n",
                functionName, pC_->portName, axisNo_, home_type, home_flag, home_velocity, flag_mode,
                home_offset);
    }
  #endif
    debug(DEBUG_TRACE, functionName, "Axis Home command", command);
    status = pC_->axisWriteRead(command, response);

  } else {
    debug(DEBUG_ERROR, functionName, "Cannot home motor, connection lost");
    status = asynError;
  }
  return status;
}

/**
 * See asynMotorAxis::moveVelocity
 */
asynStatus pmacAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration) {
  asynStatus status = asynError;
  char acc_buff[PMAC_MAXBUF] = {0};
  char vel_buff[PMAC_MAXBUF] = {0};
  char command[PMAC_MAXBUF] = {0};
  char response[PMAC_MAXBUF] = {0};
  static const char *functionName = "moveVelocity";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (connected_){
    if (max_velocity != 0) {
      sprintf(vel_buff, "I%d22=%f ", axisNo_, (fabs(max_velocity) / (scale_ * 1000.0)));
    }
    if (acceleration != 0) {
      if (max_velocity != 0) {
        sprintf(acc_buff, "I%d20=%f ", axisNo_, (fabs(max_velocity / acceleration) * 1000.0));
      }
    }
    sprintf(command, "%s%s#%d %s", vel_buff, acc_buff, axisNo_, (max_velocity < 0 ? "J-" : "J+"));

  #ifdef REMOVE_LIMITS_ON_HOME
    if (limitsDisabled_) {
      char buffer[PMAC_MAXBUF];
      /* Re-enable limits */
      sprintf(buffer, " i%d24=i%d24&$FDFFFF", axisNo_, axisNo_);
      strncat(command, buffer, PMAC_MAXBUF - 1);
      limitsDisabled_ = 0;
    }
  #endif
    debug(DEBUG_TRACE, functionName, "Axis MoveVelocity command", command);
    status = pC_->axisWriteRead(command, response);

  } else {
    debug(DEBUG_ERROR, functionName, "Cannot move motor, connection lost");
    status = asynError;
  }

  return status;
}

/**
 * See asynMotorAxis::setPosition
 */
asynStatus pmacAxis::setPosition(double position) {
  //int status = 0;
  static const char *functionName = "setPosition";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  return asynSuccess;
}

/**
 * See asynMotorAxis::stop
 */
asynStatus pmacAxis::stop(double acceleration) {
  asynStatus status = asynError;
  static const char *functionName = "stop";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  char command[PMAC_MAXBUF] = {0};
  char response[PMAC_MAXBUF] = {0};

  /*Only send a J/ if the amplifier output is enabled. When we send a stop,
    we don't want to power on axes that have been powered off for a reason. */
  if ((amp_enabled_ == 1) || (fatal_following_ == 1)) {
      sprintf(command, "#%d J/ M%d40=1", axisNo_, axisNo_);
  } else {
    /*Just set the in position bit in this case.*/
    sprintf(command, "M%d40=1", axisNo_);
  }
  deferredMove_ = 0;

  debug(DEBUG_TRACE, functionName, "Axis Stop command", command);
  status = pC_->axisWriteRead(command, response);

  // Also abort the CS so that stopping a real motor will stop CS motion
  // This needs to be separate command for some reason
  if(assignedCS_) {
    sprintf(command, "&%dA Q7%d=Q8%d", assignedCS_, axisNo_, axisNo_);
  }
  pC_->axisWriteRead(command, response);

  return status;
}

/**
 * See asynMotorAxis::setClosedLoop
 */
asynStatus pmacAxis::setClosedLoop(bool closedLoop) {
  asynStatus status = asynError;
  static const char *functionName = "setClosedLoop";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  char command[PMAC_MAXBUF] = {0};
  char response[PMAC_MAXBUF] = {0};

  if (closedLoop) {
    sprintf(command, "#%d J/", axisNo_);
  } else {
    sprintf(command, "#%d K", axisNo_);
  }
  debug(DEBUG_TRACE, functionName, "Axis Set Closed Loop command", command);
  status = pC_->axisWriteRead(command, response);
  return status;
}

void pmacAxis::callback(pmacCommandStore *sPtr, int type) {
  asynStatus status = asynSuccess;
  static const char *functionName = "callback";
//  debug()

  // Are we initialised?
  if (!initialised_){
    // Attempt once more to initialise
    initialSetup(axisNo_);
    pC_->wakeupPoller();
  } else {
    if (type == pmacMessageBroker::PMAC_FAST_READ) {
      // todo this locking is more extreme than required
      // todo factor out the writeXXXParam in getAxisStatus
      pC_->lock();
      status = this->getAxisStatus(sPtr);
      if (status != asynSuccess) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "Controller %s Axis %d. %s: getAxisStatus failed to return asynSuccess.\n",
                  pC_->portName, axisNo_, functionName);
      }
      pC_->unlock();
      callParamCallbacks();
    }
  }
}

void pmacAxis::debug(int level, const std::string &method) {
  std::stringstream sstr;
  sstr << method << " [Motor " << axisNo_ << "]";
  pmacDebugger::debug(level, sstr.str());
}

void pmacAxis::debug(int level, const std::string &method, const std::string &message) {
  std::stringstream sstr;
  sstr << method << " [Motor " << axisNo_ << "]";
  pmacDebugger::debug(level, sstr.str(), message);
}

void pmacAxis::debug(int level, const std::string &method, const std::string &message,
                     const std::string &value) {
  std::stringstream sstr;
  sstr << method << " [Motor " << axisNo_ << "]";
  pmacDebugger::debug(level, sstr.str(), message, value);
}

void pmacAxis::debug(int level, const std::string &method, const std::string &message, int value) {
  std::stringstream sstr;
  sstr << method << " [Motor " << axisNo_ << "]";
  pmacDebugger::debug(level, sstr.str(), message, value);
}

void
pmacAxis::debug(int level, const std::string &method, const std::string &message, double value) {
  std::stringstream sstr;
  sstr << method << " [Motor " << axisNo_ << "]";
  pmacDebugger::debug(level, sstr.str(), message, value);
}

/**
 * Read the axis status and set axis related parameters.
 * @param moving Boolean flag to indicate if the axis is moving. This is set by this function
 * to indcate to the polling thread how quickly to poll for status.
 * @return asynStatus
 */
asynStatus pmacAxis::getAxisStatus(pmacCommandStore *sPtr) {
    char command[PMAC_MAXBUF] = {0};
    char response[PMAC_MAXBUF] = {0};
    int cmdStatus = 0;;
    double position = 0;
    double enc_position = 0;
    int nvals = 0;
    int axisProblemFlag = 0;
    int limitsDisabledBit = 0;
    bool printErrors = true;
    char key[16];
    std::string value = "";
    int retStatus = asynSuccess;

    static const char *functionName = "getAxisStatus";

    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

    if (pC_->initialised_ && pC_->connected_) {
        /* Get the time and decide if we want to print errors.*/
        epicsTimeGetCurrent(&nowTime_);
        nowTimeSecs_ = nowTime_.secPastEpoch;
        if ((nowTimeSecs_ - lastTimeSecs_) < pC_->PMAC_ERROR_PRINT_TIME_) {
            printErrors = 0;
        } else {
            printErrors = 1;
            lastTimeSecs_ = nowTimeSecs_;
        }

        if (printNextError_) {
            printErrors = 1;
        }

        // Parse the axis status
        axisStatus axStatus;
        retStatus = pC_->pHardware_->parseAxisStatus(axisNo_, sPtr, axStatus);
        status_ = axStatus;

        setIntegerParam(pC_->PMAC_C_AxisBits01_, axStatus.status16Bit1_);
        setIntegerParam(pC_->PMAC_C_AxisBits02_, axStatus.status16Bit2_);
        setIntegerParam(pC_->PMAC_C_AxisBits03_, axStatus.status16Bit3_);

        // Parse the position
        sprintf(key, "#%dP", axisNo_);
        value = sPtr->readValue(key);
        nvals = sscanf(value.c_str(), "%lf", &enc_position);
        if (nvals != 1) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s: Failed to parse position. Key: %s  Value: %s\n",
                      functionName, key, value.c_str());
            retStatus |= asynError;
        }

        // Parse the following error or encoder channel
        if (encoder_axis_ != 0) {
            sprintf(key, "#%dP", encoder_axis_);
        } else {
            // Encoder position comes back on this axis - note we initially read
            // the following error into the position variable
            sprintf(key, "#%dF", axisNo_);
        }
        value = sPtr->readValue(key);
        nvals = sscanf(value.c_str(), "%lf", &position);
        if (nvals != 1) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s: Failed to parse following error. Key: %s  Value: %s\n",
                      functionName, key, value.c_str());
            retStatus |= asynError;
        }


        if (retStatus == asynSuccess) {

            //int homeSignal = ((status[1] & pC_->PMAC_STATUS2_HOME_COMPLETE) != 0);
            int direction = 0;

            // For closed loop axes, position is actually following error up to this point
            if (encoder_axis_ == 0) {
                position += enc_position;
            }

            // Store the raw position
            rawPosition_ = position;

            position *= scale_;
            enc_position *= scale_;

            setDoubleParam(pC_->motorPosition_, position);
            setDoubleParam(pC_->motorEncoderPosition_, enc_position);

            // Use previous position and current position to calculate direction.
            if ((position - previous_position_) > 0) {
                direction = 1;
            } else if (position - previous_position_ == 0.0) {
                direction = previous_direction_;
            } else {
                direction = 0;
            }
            setIntegerParam(pC_->motorStatusDirection_, direction);
            // Store position to calculate direction for next poll.
            previous_position_ = position;
            previous_direction_ = direction;

            // Test that we initiated a move, were moving and have now
            // stopped moving.  In this case we must update the cached
            // position.
            if (moving_ && initiatedMove_ && axStatus.done_) {
                initiatedMove_ = false;
                cachedPosition_ = rawPosition_;
                debug(DEBUG_TRACE, functionName, "Updating cached position after move complete",
                      cachedPosition_);
            }

            moving_ = !axStatus.done_ || deferredMove_;

            // Read the currently assigned CS for the axis, and whether it is assigned at all
            assignedCS_ = axStatus.currentCS_;

            // Set the currently assigned CS number
            setIntegerParam(pC_->PMAC_C_AxisCS_, assignedCS_);

            setIntegerParam(pC_->motorStatusHighLimit_, axStatus.highLimit_);
            setIntegerParam(pC_->motorStatusHomed_, axStatus.home_);

            setIntegerParam(pC_->motorStatusMoving_, moving_);
            setIntegerParam(pC_->motorStatusDone_, !moving_);

            setIntegerParam(pC_->motorStatusLowLimit_, axStatus.lowLimit_);
            setIntegerParam(pC_->motorStatusFollowingError_, axStatus.followingError_);
            fatal_following_ = axStatus.followingError_;

            // Need to make sure that we can write the CNEN flag, by setting the gain support flag in the status word
            setIntegerParam(pC_->motorStatusGainSupport_, 1);
            // Reflect PMAC_STATUS1_OPEN_LOOP in the CNEN Flag. CNEN can be set from the (user) motor record via the motorAxisClosedLoop command
            setIntegerParam(pC_->motorStatusPowerOn_, axStatus.power_);

            axisProblemFlag = 0;
            // Set any axis specific general problem bits.
            if (((axStatus.status24Bit1_ & pC_->PMAX_AXIS_GENERAL_PROB1) != 0) ||
                ((axStatus.status24Bit2_ & pC_->PMAX_AXIS_GENERAL_PROB2) != 0)) {
                if (printErrors) {
                    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                              "*** Warning *** axis %d problem status0=%x status1=%x \n",
                              axisNo_, axStatus.status24Bit1_, axStatus.status24Bit2_);
                    printNextError_ = false;
                }
                axisProblemFlag = 1;
            }

            int globalStatus = 0;
            int feedrate_problem = 0;
            pC_->getIntegerParam(0, pC_->PMAC_C_GlobalStatus_, &globalStatus);
            pC_->getIntegerParam(0, pC_->PMAC_C_FeedRateProblem_, &feedrate_problem);
            if (globalStatus || feedrate_problem) {
                if (printErrors) {
                    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                              "*** Warning *** %d problem globalStatus=%x feedrate_problem=%x \n",
                              axisNo_, globalStatus, feedrate_problem);
                    printNextError_ = false;
                }
                axisProblemFlag = 1;
            }
            // Check limits disabled bit in ix24, and if we haven't intentially disabled limits
            // because we are homing, set the motorAxisProblem bit. Also check the limitsCheckDisable
            // flag, which the user can set to disable this feature.*/
            if (!limitsCheckDisable_) {
                // Check we haven't intentially disabled limits for homing.
                if (!limitsDisabled_) {
                    // Parse ixx24
                    sprintf(key, "i%d24", axisNo_);
                    value = sPtr->readValue(key);
                    sscanf(value.c_str(), "$%x", &limitsDisabledBit);
                    limitsDisabledBit = ((0x20000 & limitsDisabledBit) >> 17);
                    if (limitsDisabledBit) {
                        axisProblemFlag = 1;
                        if (printErrors) {
                            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                                      "*** WARNING *** Limits are disabled on controller %s, axis %d\n",
                                      pC_->portName, axisNo_);
                            printNextError_ = false;
                        }
                    }
                }
            }
            setIntegerParam(pC_->motorStatusProblem_, axisProblemFlag);

            // Clear error print flag for this axis if problem has been removed.
            if (axisProblemFlag == 0) {
                printNextError_ = true;
            }
        }

#ifdef REMOVE_LIMITS_ON_HOME
        if (limitsDisabled_ && (axStatus.status24Bit2_ & pC_->PMAC_STATUS2_HOME_COMPLETE) &&
            (axStatus.status24Bit1_ & pC_->PMAC_STATUS1_DESIRED_VELOCITY_ZERO)) {
            // Re-enable limits
            sprintf(command, "i%d24=i%d24&$FDFFFF", axisNo_, axisNo_);
            cmdStatus = pC_->lowLevelWriteRead(command, response);
            limitsDisabled_ = (cmdStatus != 0);
        }
#endif
        // Set amplifier enabled bit.
        setIntegerParam(pC_->motorStatusPowerOn_, axStatus.ampEnabled_);
        amp_enabled_ = axStatus.ampEnabled_;

        if (amp_enabled_ != amp_enabled_prev_) {
            debugf(DEBUG_TRACE, functionName, "Axis %d amp enabled changed ==> %d",
                   axisNo_, amp_enabled_);
            amp_enabled_prev_ = amp_enabled_;
        }

        // if the motor stopped in an unexpected fashion, make sure the CS demands are reset
        if (!amp_enabled_ || axStatus.followingError_ || !axStatus.power_||
                axStatus.lowLimit_|| axStatus.highLimit_) {
            // make sure that pmacController->makeCSDemandsConsistent will reset the demand for all axes
            int csNum = getAxisCSNo();
            if (csNum > 0) {
                debugf(DEBUG_TRACE, functionName,
                        "Reset Q7x demands due to axis %d, amp %d, FE %d, power %d, lo %d, hi %d\n",
                        axisNo_, !amp_enabled_ , axStatus.followingError_ , !axStatus.power_,
                        axStatus.lowLimit_, axStatus.highLimit_);
                pC_->csResetAllDemands = true;
            }
        }
    }

  return asynSuccess;
}

/**
 * Return the current motor status
 */
axisStatus pmacAxis::getMotorStatus()
{
  return status_;
}

/**
 * See asynMotorAxis::poll
 */
asynStatus pmacAxis::poll(bool *moving) {
  asynStatus status = asynSuccess;
  static const char *functionName = "poll";
  debug(DEBUG_TIMING, functionName, "Poll called");
  if (axisNo_ != 0) {
    *moving = moving_;
  }

  if(!pC_->initialised_) {
      // controller is not connected, set axis problem bit
      setIntegerParam(pC_->motorStatusProblem_, true);
      callParamCallbacks();
  }

  if (!pC_->connected_) {
    setIntegerParam(pC_->motorStatusProblem_, true);
    setIntegerParam(pC_->motorStatusCommsError_, true);
  } else {
    setIntegerParam(pC_->motorStatusCommsError_, false);
  }
  callParamCallbacks();

  // If the controller is initialised and connected, but this axis is not 
  // then re-execute the initialisation
  if (pC_->initialised_ && pC_->connected_ && !initialised_){
    initialSetup(axisNo_);

    /* Wake up the poller task which will make it do a poll,
    * updating values for this axis to use the new resolution (stepSize_) */
    pC_->wakeupPoller();
  }

  return status;
}


int pmacAxis::getAxisCSNo() {
  return assignedCS_;
}

double pmacAxis::getCachedPosition() {
  return cachedPosition_;
}

double pmacAxis::getPosition() {
  return rawPosition_;
}
