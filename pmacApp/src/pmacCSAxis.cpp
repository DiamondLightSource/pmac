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

/* Use Q71 - Q79 for motor demand positions */
#define DEMAND "Q7%d"
/* Use Q81 - Q89 for motor readback positions */
#define READBACK "Q8%d"

pmacCSAxis::pmacCSAxis(pmacCSController *pController, int axisNo)
  : asynMotorAxis((asynMotorController *)pController, axisNo),
    pmacDebugger("pmacCSAxis"),
    pC_(pController)
{
  //Initialize non-static data members
//  setpointPosition_ = 0.0;
//  encoderPosition_ = 0.0;
//  currentVelocity_ = 0.0;
//  velocity_ = 0.0;
//  accel_ = 0.0;
//  highLimit_ = 0.0;
//  lowLimit_ = 0.0;
//  limitsDisabled_ = 0;
//  stepSize_ = 1; //Don't need?
//  deferredPosition_ = 0.0;
  deferredMove_ = 0;
//  deferredRelative_ = 0;
//  deferredTime_ = 0;
  scale_ = 10000;
  position_ = 0.0;
  previous_position_ = 0.0;
  previous_direction_ = 0;
//  amp_enabled_ = 0;
//  fatal_following_ = 0;
//  encoder_axis_ = 0;
//  limitsCheckDisable_ = 0;
  nowTimeSecs_ = 0.0;
  lastTimeSecs_ = 0.0;
  printNextError_ = false;
  moving_ = false;

  if (axisNo > 0){
    char var[16];
    // Request position readback
    sprintf(var, "&%dQ8%d", pC_->getCSNumber(), axisNo_);
    pC_->monitorPMACVariable(pmacMessageBroker::PMAC_FAST_READ, var);

    // Request position readback
    //sprintf(var, "#%dP", axisNo);
    //pC_->monitorPMACVariable(pmacMessageBroker::PMAC_FAST_READ, var);
    // Request following error readback
    //sprintf(var, "#%dF", axisNo);
    //pC_->monitorPMACVariable(pmacMessageBroker::PMAC_FAST_READ, var);
    // Request ixx24 readback
    //sprintf(var, "i%d24", axisNo);
    //pC_->monitorPMACVariable(pmacMessageBroker::PMAC_FAST_READ, var);
    pC_->registerForCallbacks(this, pmacMessageBroker::PMAC_FAST_READ);
  }

  /* Wake up the poller task which will make it do a poll,
   * updating values for this axis to use the new resolution (stepSize_) */
  pC_->wakeupPoller();

}

pmacCSAxis::~pmacCSAxis()
{
  // TODO Auto-generated destructor stub
}

asynStatus pmacCSAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
  asynStatus status = asynSuccess;
  char acc_buff[32]="\0";
  char command[128];
  char response[128];
  static const char *functionName = "move";

  char vel_buff[32]="";
  char buff[128];
  char commandtemp[128];
  double deviceUnits = 0.0;

  if (max_velocity != 0) {
      /* Isx89 = default feedrate in EGU/s */
      sprintf(vel_buff, "I%d89=%f ", (pC_->getCSNumber()+50), max_velocity / (double)scale_);
  }
  if (acceleration != 0) {
      if (max_velocity != 0) {
          /* Isx87 = accel time in msec */
          sprintf(acc_buff, "I%d87=%f ", (pC_->getCSNumber()+50),
                  (fabs(max_velocity/acceleration) * 1000.0));
      }
  }

  deviceUnits = position / (double)scale_;
  sprintf( command, "&%d%s%s"DEMAND"=%.12f", pC_->getCSNumber(), vel_buff, acc_buff, axisNo_, deviceUnits );

  debug(DEBUG_ERROR, functionName, "TODO: FIX Deferred Move");
//  if (pC_->movesDeferred){
//      deferredMove_ = 1;
//  }
//  else if (pAxis->program != 0) {
    if (pC_->getProgramNumber() != 0){
      // Abort current move to make sure axes are enabled
      sprintf(commandtemp, "&%dA", pC_->getCSNumber());
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

  return status;
}

asynStatus pmacCSAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration)
{
  static const char *functionName = "moveVelocity";
  debug(DEBUG_ERROR, functionName, "not implemented for CS axes");
  return asynError;
}

asynStatus pmacCSAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  static const char *functionName = "home";
  debug(DEBUG_ERROR, functionName, "not implemented for CS axes");
  return asynError;
}

asynStatus pmacCSAxis::stop(double acceleration)
{
  asynStatus status = asynSuccess;
  char acc_buff[32]="\0";
  char command[128];
  char response[128];
  static const char *functionName = "stop";

  sprintf(command, "&%d%sA "DEMAND"="READBACK, pC_->getCSNumber(), acc_buff, axisNo_, axisNo_);
  deferredMove_ = 0;

  debug(DEBUG_TRACE, functionName, "CS Stop command", command);
  status = pC_->axisWriteRead(command, response);
  return status;
}

bool pmacCSAxis::getMoving()
{
  return moving_;
}

double pmacCSAxis::getCurrentPosition()
{
  return position_;
}

void pmacCSAxis::callback(pmacCommandStore *sPtr, int type)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "callback";

  if (type == pmacMessageBroker::PMAC_FAST_READ){
    status = this->newGetAxisStatus(sPtr);
    if (status != asynSuccess) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "Controller %s Axis %d. %s: getAxisStatus failed to return asynSuccess.\n",
                pC_->portName, axisNo_, functionName);
    }
    callParamCallbacks();
  }
}

/**
 * Read the axis status and set axis related parameters.
 * @param moving Boolean flag to indicate if the axis is moving. This is set by this function
 * to indcate to the polling thread how quickly to poll for status.
 * @return asynStatus
 */
asynStatus pmacCSAxis::newGetAxisStatus(pmacCommandStore *sPtr)
{
    int done = 0;
    double position = 0;
    int nvals = 0;
    epicsUInt32 status[3] = {0, 0, 0};
    int axisProblemFlag = 0;
    bool printErrors = true;
    char key[16];
    std::string value = "";
    int homeSignal = 0;
    int direction = 0;
    int retStatus = asynSuccess;

    static const char *functionName = "pmacCSAxis::newGetAxisStatus";

//    printf("*** axis get status called\n");

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

    // Parse the status
    sprintf(key, "&%d??", pC_->getCSNumber());
    value = sPtr->readValue(key);
    nvals = sscanf(value.c_str(), "%6x%6x%6x", &status[0], &status[1], &status[2]);
    if (nvals != 3){
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s: Failed to parse status. Key: %s  Value: %s\n",
                functionName, key, value.c_str());
      retStatus |= asynError;
    }

    // Parse the position
    sprintf(key, "&%dQ8%d", pC_->getCSNumber(), axisNo_);
    value = sPtr->readValue(key);
    nvals = sscanf(value.c_str(), "%lf", &position);
    if (nvals != 1){
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


    if(deferredMove_ != 0){
      done = 0;
    } else {
      done = ((status[0] & pC_->CS_STATUS1_RUNNING_PROG) == 0)&&((status[1] & pC_->CS_STATUS2_IN_POSITION) != 0);
    }

    if (!done) {
      moving_ = true;
    } else {
      moving_ = false;
    }

    setIntegerParam(pC_->motorStatusDone_, done);
    setIntegerParam(pC_->motorStatusHighLimit_, ((status[2] & pC_->CS_STATUS3_LIMIT) != 0) );
    setIntegerParam(pC_->motorStatusHomed_, homeSignal);
    setIntegerParam(pC_->motorStatusMoving_, ((status[1] & pC_->CS_STATUS2_IN_POSITION) == 0) );
    setIntegerParam(pC_->motorStatusLowLimit_, ((status[2] & pC_->CS_STATUS3_LIMIT)!=0) );
    setIntegerParam(pC_->motorStatusFollowingError_,((status[1] & pC_->CS_STATUS2_FOLLOW_ERR) != 0) );
    setIntegerParam(pC_->motorStatusProblem_, ((status[1] & pC_->CS_STATUS2_AMP_FAULT) != 0) || ((status[1] & pC_->CS_STATUS2_RUNTIME_ERR) != 0));

    axisProblemFlag = 0;
    if (((status[1] & pC_->CS_STATUS2_AMP_FAULT) != 0) || ((status[1] & pC_->CS_STATUS2_RUNTIME_ERR) != 0)){
      if(printErrors){
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
          "*** Warning *** Coordinate System [%d] axis %d problem status0=%x status1=%x status3=%x\n",
          pC_->csNumber_, axisNo_, status[0], status[1], status[2]);
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
