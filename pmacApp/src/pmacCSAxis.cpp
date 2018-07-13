/*
 * pmacCSAxis.cpp
 *
 *  Created on: 29 Feb 2016
 *      Author: gnx91527
 */

#include <math.h>
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

  if (pC_->initialised()) {
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
  }
}

pmacCSAxis::~pmacCSAxis() {
  // TODO Auto-generated destructor stub
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
  char commandtemp[128];
  double deviceUnits = 0.0;
  double steps = fabs(position - position_);

  setIntegerParam(pC_->motorStatusMoving_, true);

  // Make any CS demands consistent with this move
  if (pC_->movesDeferred_ == 0) {
    pC_->makeCSDemandsConsistent();
  }

  strcpy(vel_buff, pC_->getVelocityCmd(max_velocity, steps).c_str());
  if (acceleration != 0) {
    if (max_velocity != 0) {
      /* Isx87 = accel time in msec */
      sprintf(acc_buff, pC_->getCSAccTimeCmd(
              fabs(max_velocity / acceleration) * 1000.0).c_str());
    }
  }

  deviceUnits = position / (double) scale_;


  if (pC_->movesDeferred_ == 0) {
    sprintf(command, "&%d%s%sQ7%d=%.12f", pC_->getCSNumber(), vel_buff,
            acc_buff, axisNo_, deviceUnits);
    if (pC_->getProgramNumber() != 0) {
      // Abort current move to make sure axes are enabled
      sprintf(commandtemp, "&%dE", pC_->getCSNumber());
      debug(DEBUG_TRACE, functionName, "Sending command to PMAC", commandtemp);
      status = pC_->axisWriteRead(commandtemp, response);
      /* If the program specified is non-zero, add a command to run the program.
       * If program number is zero, then the move will have to be started by some
       * external process, which is a mechanism of allowing coordinated starts to
       * movement. */
      sprintf(buff, " B%dR", pC_->getProgramNumber());
      strcat(command, buff);
      debug(DEBUG_TRACE, functionName, "Sending command to PMAC", command);
      status = pC_->axisWriteRead(command, response);
    }
  } else {
    // do not pass the velocity buffer, deferred velocity is controlled separately
    sprintf(command, "%sQ7%d=%.12f", acc_buff, axisNo_, deviceUnits);
    deferredMove_ = pC_->movesDeferred_;
    sprintf(deferredCommand_, "%s", command);
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
  status = pC_->axisWriteRead(command, response);
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
  int retStatus = asynSuccess;

  static const char *functionName = "pmacCSAxis::newGetAxisStatus";

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
