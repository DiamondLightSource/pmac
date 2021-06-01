/*
 * pmacCSAxis.cpp
 *
 *  Created on: 29 Feb 2016
 *      Author: gnx91527
 */

#include <math.h>
#include <iostream>
#include <sstream>
#include "pmacCSAxis.h"
#include "pmacCSController.h"
#include "pmacMessageBroker.h"
#include "pmacController.h"

/* Use Q71 - Q79 for motor demand positions */
/* Use Q81 - Q89 for motor readback positions */
#define READBACK "Q8%d"

pmacCSAxis::pmacCSAxis(pmacCSController *pController, int axisNo)
        : asynMotorAxis((asynMotorController *) pController, axisNo),
          pmacDebugger("pmacCSAxis"),
          pC_(pController) {
  //Initialize non-static data members
  deferredMove_ = 0;
  scale_ = 10000;
  position_ = 0.0;
  previous_position_ = 0.0;
  previous_direction_ = 0;
  nowTimeSecs_ = 0.0;
  lastTimeSecs_ = 0.0;
  printNextError_ = false;
  moving_ = false;
  connected_ = false;
  initialized_ = false;

  if (pC_->initialised()) {
//    this->goodConnection();

    if (axisNo > 0) {
      char var[16];
      // Request position readback
      sprintf(var, "&%dQ8%d", pC_->getCSNumber(), axisNo_);
      pC_->monitorPMACVariable(pmacMessageBroker::PMAC_FAST_READ, var);

      // Register for callbacks
      pC_->registerForCallbacks(this, pmacMessageBroker::PMAC_FAST_READ);
    }

    // Wake up the poller task which will make it do a poll,
    // updating values for this axis to use the new resolution (stepSize_)
    pC_->wakeupPoller();

    // Finally, set the connection to good
    connected_ = true;
    initialized_ = true;
  }
}

pmacCSAxis::~pmacCSAxis() {
  // TODO Auto-generated destructor stub
}

void pmacCSAxis::badConnection() {
  connected_ = false;
  setIntegerParam(pC_->motorStatusProblem_, true);
  setIntegerParam(pC_->motorStatusCommsError_, true);
  statusChanged_ = 1;
  callParamCallbacks();
}

void pmacCSAxis::goodConnection() {
  if (!initialized_){
    initialized_ = true;
    if (axisNo_ > 0) {
      char var[16];
      // Request position readback
      sprintf(var, "&%dQ8%d", pC_->getCSNumber(), axisNo_);
      pC_->monitorPMACVariable(pmacMessageBroker::PMAC_FAST_READ, var);

      // Register for callbacks
      pC_->registerForCallbacks(this, pmacMessageBroker::PMAC_FAST_READ);
    }

    // Wake up the poller task which will make it do a poll,
    // updating values for this axis to use the new resolution (stepSize_)
    pC_->wakeupPoller();
  }

  connected_ = true;
  setIntegerParam(pC_->motorStatusProblem_, false);
  setIntegerParam(pC_->motorStatusCommsError_, false);
  statusChanged_ = 1;
  callParamCallbacks();
}

void pmacCSAxis::setKinematicResolution(double new_resolution)
{
  static const char *functionName = "setKinematicResolution";
  debug(DEBUG_TRACE, functionName, "Setting CS axis kinematic resolution", new_resolution);
  this->kinematic_resolution_ = new_resolution;
}

double pmacCSAxis::getResolution()
{
  static const char *functionName = "getResolution";
  double resolution = 0.0;
  int mappedAxis = 0;

  // Find out if this motor is mapped directly to a raw motor
  mappedAxis = pC_->pmacCSGetAxisDirectMapping(this->axisNo_);

  // If the axis is not mapped then it is in a kinematic, use the stored values
  if (mappedAxis == 0){
    resolution = this->kinematic_resolution_ * this->scale_;
  } else {
    // Get the raw axis offset and resolution
    pmacAxis *ptr = pC_->getRawAxis(mappedAxis);
    resolution = ptr->getResolution();
  }
  debug(DEBUG_TRACE, functionName, "Returned resolution", resolution);
  return resolution;
}

void pmacCSAxis::setKinematicOffset(double new_offset)
{
  static const char *functionName = "setKinematicOffset";
  debug(DEBUG_TRACE, functionName, "Setting CS axis kinematic offset", new_offset);
  this->kinematic_offset_ = new_offset;
}

double pmacCSAxis::getOffset()
{
  static const char *functionName = "getOffset";
  double offset = 0.0;
  int mappedAxis = 0;
  // Find out if this motor is mapped directly to a raw motor
  mappedAxis = pC_->pmacCSGetAxisDirectMapping(this->axisNo_);

  // If the axis is not mapped then it is in a kinematic, use the stored values
  if (mappedAxis == 0){
    offset = this->kinematic_offset_;
  } else {
    // Get the raw axis offset
    pmacAxis *ptr = pC_->getRawAxis(mappedAxis);
    offset = ptr->getOffset();
  }
  debug(DEBUG_TRACE, functionName, "Returned offset", offset);
  return offset;
}

asynStatus pmacCSAxis::directMove(double position, double min_velocity, double max_velocity, double acceleration) {
  static const char *functionName = "directMove";
  double raw_position = 0.0;
  int mappedAxis = 0;

  // Find out if this motor is mapped directly to a raw motor
  mappedAxis = pC_->pmacCSGetAxisDirectMapping(this->axisNo_);

  // If the axis is not mapped then it is in a kinematic, use the stored values
  if (mappedAxis == 0){
    debug(DEBUG_TRACE, functionName, "Scale", this->scale_);
    debug(DEBUG_TRACE, functionName, "Kinematic resolution", this->kinematic_resolution_);
    raw_position = (position - this->kinematic_offset_) / this->kinematic_resolution_;
  } else {
    // Get the raw axis offset and resolution
    pmacAxis *ptr = pC_->getRawAxis(mappedAxis);
    raw_position = (position - ptr->getOffset()) / ptr->getResolution() * this->scale_;
  }

  // Calculate the real position demand using the resolution and offset and then call the move command
  
  std::stringstream ss;
  ss << "Direct move called for motor [" << this->axisNo_ << "] EGU Position: " << position;
  ss << " Min Velocity: " << min_velocity << " Max velocity: " << max_velocity << " Acceleration: " << acceleration << std::endl;
  ss << "Calculated raw position (in counts): " << (raw_position / this->scale_);
  debug(DEBUG_TRACE, functionName, ss.str());

  return this->move(raw_position, 0, min_velocity, max_velocity, acceleration);
}

asynStatus pmacCSAxis::move(double position, int /*relative*/, double min_velocity, double max_velocity,
                            double acceleration) {
  asynStatus status = asynSuccess;
  char acc_buff[128] = "\0";
  char command[128];
  char response[128];
  static const char *functionName = "move";

  char vel_buff[128] = "";
  char buff[128];
  double deviceUnits = 0.0;
  double steps = fabs(position - position_);

  debug(DEBUG_FLOW, functionName);

  if (connected_){
    setIntegerParam(pC_->motorStatusMoving_, true);

    // Make any CS demands consistent with this move
    if (pC_->movesDeferred_ == 0) {
      pC_->makeCSDemandsConsistent();
    }

    if (this->pC_->pC_->useCsVelocity) {
      strcpy(vel_buff, pC_->getVelocityCmd(max_velocity, steps).c_str());
    }
    if (acceleration != 0) {
      if (max_velocity != 0) {
        /* Isx87 = accel time in msec */
        sprintf(acc_buff, "%s",
                pC_->getCSAccTimeCmd(fabs(max_velocity / acceleration) * 1000.0).c_str());
      }
    }

    deviceUnits = position / (double) scale_;


    if (pC_->movesDeferred_ == 0) {
      sprintf(command, "&%d%s%sQ7%d=%.12f", pC_->getCSNumber(), vel_buff,
              acc_buff, axisNo_, deviceUnits);
      if (pC_->getProgramNumber() != 0) {
        // Abort current move to make sure axes are enabled
        status = pC_->axisWriteRead(
                pC_->pC_->pHardware_->getCSEnableCommand(pC_->getCSNumber()).c_str(),
                response);
        /* If the program specified is non-zero, add a command to run the program.
        * If program number is zero, then the move will have to be started by some
        * external process, which is a mechanism of allowing coordinated starts to
        * movement. */
        sprintf(buff, " B%dR", pC_->getProgramNumber());
        strcat(command, buff);
        status = pC_->axisWriteRead(command, response);
      }
    } else {
      // do not pass the velocity buffer, deferred velocity is controlled separately
      sprintf(command, "%sQ7%d=%.12f", acc_buff, axisNo_, deviceUnits);
      deferredMove_ = pC_->movesDeferred_;
      sprintf(deferredCommand_, "%s", command);
    }
  } else {
    debug(DEBUG_ERROR, functionName, "Cannot move CS axis, connection lost");
    status = asynError;
  }
  return status;
}

asynStatus pmacCSAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration) {
  static const char *functionName = "moveVelocity";
  debug(DEBUG_ERROR, functionName, "not implemented for CS axes");
  return asynError;
}

asynStatus
pmacCSAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
  static const char *functionName = "home";
  debug(DEBUG_ERROR, functionName, "not implemented for CS axes");
  return asynError;
}

asynStatus pmacCSAxis::stop(double acceleration) {
  asynStatus status = asynSuccess;
  char acc_buff[32] = "\0";
  char command[128];
  char response[128];
  static const char *functionName = "stop";

  sprintf(command, "&%d%sA Q7%d=Q8%d", pC_->getCSNumber(), acc_buff, axisNo_, axisNo_);
  deferredMove_ = 0;

  debug(DEBUG_TRACE, functionName, "CS Stop command", command);

  if (connected_){
    status = pC_->axisWriteRead(command, response);
  } else {
    debug(DEBUG_ERROR, functionName, "Cannot stop CS axis, connection lost");
    status = asynError;
  }
  return status;
}

bool pmacCSAxis::getMoving() {
  return moving_;
}

double pmacCSAxis::getCurrentPosition() {
  return position_;
}

void pmacCSAxis::callback(pmacCommandStore *sPtr, int type) {
  asynStatus status = asynSuccess;
  static const char *functionName = "callback";

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

/**
 * Read the axis status and set axis related parameters.
 * @param moving Boolean flag to indicate if the axis is moving. This is set by this function
 * to indcate to the polling thread how quickly to poll for status.
 * @return asynStatus
 */
asynStatus pmacCSAxis::getAxisStatus(pmacCommandStore *sPtr) {
  double position = 0;
  int nvals = 0;
  int axisProblemFlag = 0;
  bool printErrors = true;
  char key[16];
  std::string value;
  int homeSignal = 0;
  int direction = 0;
  int mappedAxis = 0;
  int retStatus = asynSuccess;

  static const char *functionName = "pmacCSAxis::GetAxisStatus";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

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

  // Read in the status
  csStatus cStatus = pC_->getStatus();

  // Update this CS motor resolution and offset
  // Find out if this motor is mapped directly to a raw motor
  mappedAxis = pC_->pmacCSGetAxisDirectMapping(this->axisNo_);
  // If the axis is not mapped then it is in a kinematic, update resultion and offset
  if (mappedAxis == 0){
    setDoubleParam(pC_->PMAC_CS_DirectRes_, this->kinematic_resolution_);
    setDoubleParam(pC_->PMAC_CS_DirectOffset_, this->kinematic_offset_);
  } else {
    // Use the raw axis offset and resolution
    pmacAxis *ptr = pC_->getRawAxis(mappedAxis);
    setDoubleParam(pC_->PMAC_CS_DirectRes_, ptr->getResolution());
    setDoubleParam(pC_->PMAC_CS_DirectOffset_, ptr->getOffset());
  }

  // Parse the position
  sprintf(key, "&%dQ8%d", pC_->getCSNumber(), axisNo_);
  value = sPtr->readValue(key);
  nvals = sscanf(value.c_str(), "%lf", &position);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s: Failed to parse position. Key: %s  Value: %s\n",
              functionName, key, value.c_str());
    retStatus |= asynError;
  }

  /* TODO: possibly look at the aggregate of the home status of all motors in the c.s ?? */
  homeSignal = 0;

  position *= scale_;

  // Record current position
  position_ = position;

  setDoubleParam(pC_->motorPosition_, position);
  setDoubleParam(pC_->motorEncoderPosition_, position);

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

  moving_ = !cStatus.done_ || deferredMove_;

  setIntegerParam(pC_->motorStatusDone_, !moving_);
  setIntegerParam(pC_->motorStatusMoving_, moving_);

  setIntegerParam(pC_->motorStatusHighLimit_, cStatus.highLimit_);
  setIntegerParam(pC_->motorStatusHomed_, homeSignal);
  setIntegerParam(pC_->motorStatusLowLimit_, cStatus.lowLimit_);
  setIntegerParam(pC_->motorStatusFollowingError_, cStatus.followingError_);
  setIntegerParam(pC_->motorStatusProblem_, cStatus.problem_);

  axisProblemFlag = 0;
  if (cStatus.problem_) {
    if (printErrors) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "*** Warning *** Coordinate System [%d] axis %d problem status0=%x status1=%x status3=%x\n",
                pC_->csNumber_, axisNo_, cStatus.stat1_, cStatus.stat2_, cStatus.stat3_);
      printNextError_ = false;
    }
    axisProblemFlag = 1;
  }
  // Clear error print flag for this axis if problem has been removed.
  if (axisProblemFlag == 0) {
    printNextError_ = true;
  }

  return asynSuccess;
}
