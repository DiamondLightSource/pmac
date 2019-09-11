/*
 * pmacHardwarePower.cpp
 *
 *  Created on: 27 Oct 2016
 *      Author: gnx91527
 */

#include "pmacHardwarePower.h"
#include "pmacController.h"

const std::string pmacHardwarePower::GLOBAL_STATUS = "?";
const std::string pmacHardwarePower::AXIS_STATUS = "#%d?";
const std::string pmacHardwarePower::AXIS_CS_NUMBER = "Motor[%d].Coord";
const std::string pmacHardwarePower::CS_STATUS = "&%d?";
const std::string pmacHardwarePower::CS_INPOS = "Coord[%d].InPos";
const std::string pmacHardwarePower::CS_AMPENABLE = "Coord[%d].AmpEna";
const std::string pmacHardwarePower::CS_RUNNING = "Coord[%d].ProgRunning";
const std::string pmacHardwarePower::CS_VEL_CMD = "&%dQ70=%f ";
const std::string pmacHardwarePower::CS_ACCELERATION_CMD = "Coord[%d].Ta=%f Coord[%d].Td=%f";
// the trailing ; stops clashes from occurring (instead of comma on turbo)
const std::string pmacHardwarePower::CS_AXIS_MAPPING = "#%d->;";
const std::string pmacHardwarePower::CS_ENABLED_COUNT = "Sys.MaxCoords";

const int pmacHardwarePower::PMAC_STATUS1_TRIGGER_MOVE = (0x1 << 31);
const int pmacHardwarePower::PMAC_STATUS1_HOMING = (0x1 << 30);
const int pmacHardwarePower::PMAC_STATUS1_NEG_LIMIT_SET = (0x1 << 29);
const int pmacHardwarePower::PMAC_STATUS1_POS_LIMIT_SET = (0x1 << 28);
const int pmacHardwarePower::PMAC_STATUS1_WARN_FOLLOW_ERR = (0x1 << 27);
const int pmacHardwarePower::PMAC_STATUS1_ERR_FOLLOW_ERR = (0x1 << 26);
const int pmacHardwarePower::PMAC_STATUS1_LIMIT_STOP = (0x1 << 25);
const int pmacHardwarePower::PMAC_STATUS1_AMP_FAULT = (0x1 << 24);
const int pmacHardwarePower::PMAC_STATUS1_SOFT_MINUS_LIMIT = (0x1 << 23);
const int pmacHardwarePower::PMAC_STATUS1_SOFT_PLUS_LIMIT = (0x1 << 22);
const int pmacHardwarePower::PMAC_STATUS1_I2T_AMP_FAULT = (0x1 << 21);
const int pmacHardwarePower::PMAC_STATUS1_HOME_COMPLETE = (0x1 << 15);
const int pmacHardwarePower::PMAC_STATUS1_DESIRED_VELOCITY_ZERO = (0x1 << 14);
const int pmacHardwarePower::PMAC_STATUS1_CLOSED_LOOP = (0x1 << 13);
const int pmacHardwarePower::PMAC_STATUS1_AMP_ENABLED = (0x1 << 12);
const int pmacHardwarePower::PMAC_STATUS1_IN_POSITION = (0x1 << 11);
const int pmacHardwarePower::PMAC_STATUS1_BLOCK_REQUEST = (0x1 << 9);
const int pmacHardwarePower::PMAC_STATUS1_PHASED_MOTOR = (0x1 << 8);

pmacHardwarePower::pmacHardwarePower() : pmacDebugger("pmacHardwareTurbo") {
}

pmacHardwarePower::~pmacHardwarePower() {
  // TODO Auto-generated destructor stub
}

std::string pmacHardwarePower::getGlobalStatusCmd() {
  return GLOBAL_STATUS;
}

asynStatus
pmacHardwarePower::parseGlobalStatus(const std::string &statusString, globalStatus &globStatus) {
  asynStatus status = asynSuccess;
  int nvals = 0;
  static const char *functionName = "parseGlobalStatus";

  debug(DEBUG_VARIABLE, functionName, "Status string", statusString);
  if (statusString == "") {
    debug(DEBUG_ERROR, functionName, "Problem reading global status command, returned",
          statusString);
    status = asynError;
  } else {
    // PowerPMAC status parsing
    nvals = sscanf(statusString.c_str(), " $%8x", &globStatus.status_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading global status", GLOBAL_STATUS);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", statusString);
      globStatus.status_ = 0;
      status = asynError;
    }
    nvals = sscanf(statusString.c_str(), " $%4x%4x", &globStatus.stat1_, &globStatus.stat2_);
    if (nvals != 2) {
      debug(DEBUG_ERROR, functionName, "Error reading global status (16 bit)", GLOBAL_STATUS);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", statusString);
      globStatus.stat1_ = 0;
      globStatus.stat2_ = 0;
      status = asynError;
    }
    globStatus.stat3_ = 0;
  }
  return status;
}

std::string pmacHardwarePower::getAxisStatusCmd(int axis) {
  char cmd[8];
  static const char *functionName = "getAxisStatusCmd";

  debug(DEBUG_TRACE, functionName, "Axis", axis);
  sprintf(cmd, AXIS_STATUS.c_str(), axis);
  return std::string(cmd);
}

asynStatus pmacHardwarePower::setupAxisStatus(int axis) {
  asynStatus status = asynSuccess;
  char var[16];
  static const char *functionName = "setupAxisStatus";

  debug(DEBUG_TRACE, functionName, "Axis", axis);
  // Request coordinate system readback
  sprintf(var, AXIS_CS_NUMBER.c_str(), axis);
  pC_->monitorPMACVariable(pmacMessageBroker::PMAC_FAST_READ, var);
  return status;
}

asynStatus
pmacHardwarePower::parseAxisStatus(int axis, pmacCommandStore *sPtr, axisStatus &axStatus) {
  asynStatus status = asynSuccess;
  int nvals = 0;
  int dummyVal = 0;
  std::string statusString = "";
  std::string csString = "";
  char var[16];
  static const char *functionName = "parseAxisStatus";

  statusString = sPtr->readValue(this->getAxisStatusCmd(axis));

  // Response parsed for PowerPMAC
  debug(DEBUG_VARIABLE, functionName, "Status string", statusString);
  nvals = sscanf(statusString.c_str(), " $%8x%8x", &axStatus.status24Bit1_,
                 &axStatus.status24Bit2_);
  if (nvals != 2) {
    debug(DEBUG_ERROR, functionName, "Failed to parse axis status (24 bit)", statusString);
    axStatus.status24Bit1_ = 0;
    axStatus.status24Bit2_ = 0;
    status = asynError;
  }
  nvals = sscanf(statusString.c_str(), " $%4x%4x%4x%4x", &axStatus.status16Bit1_,
                 &axStatus.status16Bit2_, &axStatus.status16Bit3_, &dummyVal);
  if (nvals != 4) {
    debug(DEBUG_ERROR, functionName, "Failed to parse axis status (16 bit)", statusString);
    axStatus.status16Bit1_ = 0;
    axStatus.status16Bit2_ = 0;
    axStatus.status16Bit3_ = 0;
    status = asynError;
  }
  debug(DEBUG_VARIABLE, functionName, "Read status 24[0]", axStatus.status24Bit1_);
  debug(DEBUG_VARIABLE, functionName, "Read status 24[1]", axStatus.status24Bit2_);
  debug(DEBUG_VARIABLE, functionName, "Read status 16[0]", axStatus.status16Bit1_);
  debug(DEBUG_VARIABLE, functionName, "Read status 16[1]", axStatus.status16Bit2_);
  debug(DEBUG_VARIABLE, functionName, "Read status 16[2]", axStatus.status16Bit3_);

  if (status == asynSuccess) {
    axStatus.home_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_HOME_COMPLETE) != 0);

    axStatus.done_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_IN_POSITION) != 0);
    /*If we are not done, but amp has been disabled, then set done (to stop when we get following errors).*/
    if ((axStatus.done_ == 0) && ((axStatus.status24Bit1_ & PMAC_STATUS1_AMP_ENABLED) == 0)) {
      axStatus.done_ = 1;
    }

    axStatus.highLimit_ = (((axStatus.status24Bit1_ & PMAC_STATUS1_POS_LIMIT_SET) |
                            (axStatus.status24Bit1_ & PMAC_STATUS1_SOFT_PLUS_LIMIT)) != 0);
    // If desired_vel_zero is false && motor activated (ix00=1) && amplifier enabled, set moving=1.
    axStatus.moving_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_DESIRED_VELOCITY_ZERO) == 0) &&
                       ((axStatus.status24Bit1_ & PMAC_STATUS1_AMP_ENABLED) != 0);
    axStatus.lowLimit_ = (((axStatus.status24Bit1_ & PMAC_STATUS1_NEG_LIMIT_SET) |
                           (axStatus.status24Bit1_ & PMAC_STATUS1_SOFT_MINUS_LIMIT)) != 0);
    axStatus.followingError_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_ERR_FOLLOW_ERR) != 0);
    axStatus.power_ = (axStatus.status24Bit1_ & PMAC_STATUS1_CLOSED_LOOP);

    // Set amplifier enabled bit.
    axStatus.ampEnabled_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_AMP_ENABLED) != 0);
  }

  // Now read the coordinate system number for this axis
  sprintf(var, AXIS_CS_NUMBER.c_str(), axis);
  csString = sPtr->readValue(var);
  nvals = sscanf(csString.c_str(), "%d", &axStatus.currentCS_);
  if (nvals != 1) {
    debug(DEBUG_ERROR, functionName, "Failed to parse CS number", csString);
    axStatus.currentCS_ = 0;
    status = asynError;
  }

  return status;
}

asynStatus pmacHardwarePower::setupCSStatus(int csNo) {
  asynStatus status = asynSuccess;
  char var[30];
  static const char *functionName = "setupAxisStatus";

  debug(DEBUG_TRACE, functionName, "CS Number", csNo);
  // Add the CS status item to the fast update
  sprintf(var, CS_STATUS.c_str(), csNo);
  pC_->monitorPMACVariable(pmacMessageBroker::PMAC_PRE_FAST_READ, var);
  sprintf(var, CS_RUNNING.c_str(), csNo);
  pC_->monitorPMACVariable(pmacMessageBroker::PMAC_PRE_FAST_READ, var);
  sprintf(var, CS_INPOS.c_str(), csNo);
  pC_->monitorPMACVariable(pmacMessageBroker::PMAC_PRE_FAST_READ, var);
  sprintf(var, CS_AMPENABLE.c_str(), csNo);
  pC_->monitorPMACVariable(pmacMessageBroker::PMAC_PRE_FAST_READ, var);

  return status;
}

asynStatus
pmacHardwarePower::parseCSStatus(int csNo, pmacCommandStore *sPtr, csStatus &coordStatus) {
  asynStatus status = asynSuccess;
  int nvals = 0;
  int cs_amp_enabled = 0;
  std::string statusString = "";
  char var[30];
  static const char *functionName = "parseCSStatus";

  sprintf(var, CS_RUNNING.c_str(), csNo);
  statusString = sPtr->readValue(var);
  sscanf(statusString.c_str(), "%d", &coordStatus.running_);
  sprintf(var, CS_INPOS.c_str(), csNo);
  statusString = sPtr->readValue(var);
  sscanf(statusString.c_str(), "%d", &coordStatus.done_);
  sprintf(var, CS_AMPENABLE.c_str(), csNo);
  statusString = sPtr->readValue(var);
  sscanf(statusString.c_str(), "%d", &cs_amp_enabled);
  // we want to show Done Moving true when any of the CS motor's amps are disabled
  coordStatus.done_= coordStatus.done_|| !cs_amp_enabled;
  coordStatus.moving_ = !coordStatus.done_;

  sprintf(var, CS_STATUS.c_str(), csNo);
  statusString = sPtr->readValue(var);
  // Parse the status
  nvals = sscanf(statusString.c_str(), " $%8x%8x", &coordStatus.stat1_, &coordStatus.stat2_);
  if (nvals != 2) {
    debug(DEBUG_ERROR, functionName, "Failed to parse CS status. ", statusString);
    coordStatus.stat1_ = 0;
    coordStatus.stat2_ = 0;
    coordStatus.stat3_ = 0;
    status = asynError;
  }
  coordStatus.stat3_ = 0;
  if (status == asynSuccess) {
    coordStatus.highLimit_ = ((coordStatus.stat1_ & PMAC_STATUS1_POS_LIMIT_SET) != 0);
    coordStatus.lowLimit_ = ((coordStatus.stat1_ & PMAC_STATUS1_NEG_LIMIT_SET) != 0);
    coordStatus.followingError_ = ((coordStatus.stat1_ & PMAC_STATUS1_ERR_FOLLOW_ERR) != 0);
    coordStatus.problem_ = ((coordStatus.stat1_ & PMAC_STATUS1_AMP_FAULT) != 0);
  } else {
    coordStatus.done_ = 0;
    coordStatus.highLimit_ = 0;
    coordStatus.lowLimit_ = 0;
    coordStatus.followingError_ = 0;
    coordStatus.moving_ = 0;
    coordStatus.problem_ = 0;
  }
  return status;
}

std::string pmacHardwarePower::getCSVelocityCmd(int csNo, double velocity, double steps) {
  char cmd[64];
  static const char *functionName = "getCSVelocityCmd";
  double move_time = 0;

  debug(DEBUG_TRACE, functionName, "CS Number", csNo);
  debug(DEBUG_TRACE, functionName, "Velocity", velocity);
  // sets Q70 which PROG10 places into a TM command, so units are
  // converted to milliseconds for entire move
  // if velocity is 0 then set Q70 to 0 meaning use underlying real motor speeds
  if (velocity != 0) {
    move_time = steps / velocity * 1000;
  }
  sprintf(cmd, CS_VEL_CMD.c_str(), csNo, move_time);
  return std::string(cmd);
}

std::string pmacHardwarePower::getCSAccTimeCmd(int csNo, double time) {
  char cmd[64];
  static const char *functionName = "getCSAccTimeCmd";

  debug(DEBUG_FLOW, functionName, "CS Number", csNo);
  debug(DEBUG_TRACE, functionName, "time", time);
  sprintf(cmd, CS_ACCELERATION_CMD.c_str(), csNo, time, csNo, time);
  return std::string(cmd);
}

std::string pmacHardwarePower::getCSMappingCmd(int csNo, int axis) {
  char cmd[255];
  static const char *functionName = "getCSMappingCmd";

  debugf(DEBUG_FLOW, functionName, "CsNo %d, Axis %d", csNo, axis);
  sprintf(cmd, CS_AXIS_MAPPING.c_str(), axis);
  return std::string(cmd);
}

std::string pmacHardwarePower::getCSEnabledCountCmd(){
  return std::string(CS_ENABLED_COUNT);
}

std::string pmacHardwarePower::parseCSMappingResult(const std::string mappingResult) {
  std::string result;
  static const char *functionName = "parseCSMappingResult";
  debugf(DEBUG_FLOW, functionName, "command %s", mappingResult.c_str());

  if (mappingResult.length() > 0) {
    const char *mapping = mappingResult.substr(mappingResult.length() - 1, 1).c_str();
    char upper_mapping = (char) toupper(mapping[0]);
    result = std::string(1, upper_mapping);
  }

  return result;
}

void pmacHardwarePower::startTrajectoryTimePointsCmd(char *velCmd, char *userCmd,
                                                     char *timeCmd, int addr) {
  static const char *functionName = "startTrajectoryTimePointsCmd";

  debug(DEBUG_FLOW, functionName, "addr %d", addr);

  sprintf(velCmd, "Next_Vel(%d)=", addr);
  sprintf(userCmd, "Next_User(%d)=", addr);
  sprintf(timeCmd, "Next_Time(%d)=", addr);

}

void pmacHardwarePower::addTrajectoryTimePointCmd(char *velCmd, char *userCmd, char *timeCmd,
                                                  int velocityMode, int userFunc, int time,
                                                  bool firstVal) {
  static const char *functionName = "addTrajectoryTimePointCmd";

  debugf(DEBUG_FLOW, functionName, "velCmd %s\nuserCmd %s\ntimeCmd %s\nvel %d, user %d, time %d",
    velCmd, userCmd, timeCmd, velocityMode, userFunc, time);

  if(firstVal) {
    sprintf(velCmd, "%s%d", velCmd, velocityMode);
    sprintf(userCmd, "%s%d", userCmd, userFunc);
    sprintf(timeCmd, "%s%d", timeCmd, time);
  }
  else {
    sprintf(velCmd, "%s,%d", velCmd, velocityMode);
    sprintf(userCmd, "%s,%d", userCmd, userFunc);
    sprintf(timeCmd, "%s,%d", timeCmd, time);
  }
}

void pmacHardwarePower::startAxisPointsCmd(char *axisCmd, int axis, int addr, int ) {
  const char axes[] = "ABCUVWXYZ";
  static const char *functionName = "startAxisPointsCmd";

  debugf(DEBUG_FLOW, functionName, "cmd %s, axis %d, addr %d", axisCmd, axis, addr);

  sprintf(axisCmd, "Next_%c(%d)=", axes[axis], addr);
}

void pmacHardwarePower::addAxisPointCmd(char *axisCmd, int , double pos, int ,
                                        bool firstVal) {
  static const char *functionName = "addAxisPointCmd";

  debugf(DEBUG_FLOW, functionName, "cmd %s, pos %f, firstval %d", axisCmd,
          pos, firstVal);

  if(firstVal) {
    sprintf(axisCmd, "%s%g", axisCmd, pos);
  }
  else {
    sprintf(axisCmd, "%s,%g", axisCmd, pos);
  }
}

std::string pmacHardwarePower::getCSEnableCommand(int csNo) {
  char cmd[10];
  static const char *functionName = "getCSEnableCommand";

  debugf(DEBUG_FLOW, functionName, "cmd %s, CS %d", cmd, csNo);
  sprintf(cmd, "&%denable", csNo);
  return std::string(cmd);
}