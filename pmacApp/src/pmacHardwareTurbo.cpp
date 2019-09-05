/*
 * pmacHardwareTurbo.cpp
 *
 *  Created on: 27 Oct 2016
 *      Author: gnx91527
 */

#include "pmacHardwareTurbo.h"
#include "pmacController.h"

const std::string pmacHardwareTurbo::GLOBAL_STATUS = "???";
const std::string pmacHardwareTurbo::AXIS_STATUS = "#%d?";
const std::string pmacHardwareTurbo::CS_STATUS = "&%d??";
const std::string pmacHardwareTurbo::CS_VEL_CMD = "&%dQ70=%f ";
const std::string pmacHardwareTurbo::CS_ACCELERATION_CMD = "I%d87=%f";
const std::string pmacHardwareTurbo::CS_AXIS_MAPPING = "&%d#%d->,";
const std::string pmacHardwareTurbo::CS_ENABLED_COUNT = "I68";

const int pmacHardwareTurbo::PMAC_STATUS1_MAXRAPID_SPEED = (0x1 << 0);
const int pmacHardwareTurbo::PMAC_STATUS1_ALT_CMNDOUT_MODE = (0x1 << 1);
const int pmacHardwareTurbo::PMAC_STATUS1_SOFT_POS_CAPTURE = (0x1 << 2);
const int pmacHardwareTurbo::PMAC_STATUS1_ERROR_TRIGGER = (0x1 << 3);
const int pmacHardwareTurbo::PMAC_STATUS1_FOLLOW_ENABLE = (0x1 << 4);
const int pmacHardwareTurbo::PMAC_STATUS1_FOLLOW_OFFSET = (0x1 << 5);
const int pmacHardwareTurbo::PMAC_STATUS1_PHASED_MOTOR = (0x1 << 6);
const int pmacHardwareTurbo::PMAC_STATUS1_ALT_SRC_DEST = (0x1 << 7);
const int pmacHardwareTurbo::PMAC_STATUS1_USER_SERVO = (0x1 << 8);
const int pmacHardwareTurbo::PMAC_STATUS1_USER_PHASE = (0x1 << 9);
const int pmacHardwareTurbo::PMAC_STATUS1_HOMING = (0x1 << 10);
const int pmacHardwareTurbo::PMAC_STATUS1_BLOCK_REQUEST = (0x1 << 11);
const int pmacHardwareTurbo::PMAC_STATUS1_DECEL_ABORT = (0x1 << 12);
const int pmacHardwareTurbo::PMAC_STATUS1_DESIRED_VELOCITY_ZERO = (0x1 << 13);
const int pmacHardwareTurbo::PMAC_STATUS1_DATABLKERR = (0x1 << 14);
const int pmacHardwareTurbo::PMAC_STATUS1_DWELL = (0x1 << 15);
const int pmacHardwareTurbo::PMAC_STATUS1_INTEGRATE_MODE = (0x1 << 16);
const int pmacHardwareTurbo::PMAC_STATUS1_MOVE_TIME_ON = (0x1 << 17);
const int pmacHardwareTurbo::PMAC_STATUS1_OPEN_LOOP = (0x1 << 18);
const int pmacHardwareTurbo::PMAC_STATUS1_AMP_ENABLED = (0x1 << 19);
const int pmacHardwareTurbo::PMAC_STATUS1_X_SERVO_ON = (0x1 << 20);
const int pmacHardwareTurbo::PMAC_STATUS1_POS_LIMIT_SET = (0x1 << 21);
const int pmacHardwareTurbo::PMAC_STATUS1_NEG_LIMIT_SET = (0x1 << 22);
const int pmacHardwareTurbo::PMAC_STATUS1_MOTOR_ON = (0x1 << 23);

const int pmacHardwareTurbo::PMAC_STATUS2_IN_POSITION = (0x1 << 0);
const int pmacHardwareTurbo::PMAC_STATUS2_WARN_FOLLOW_ERR = (0x1 << 1);
const int pmacHardwareTurbo::PMAC_STATUS2_ERR_FOLLOW_ERR = (0x1 << 2);
const int pmacHardwareTurbo::PMAC_STATUS2_AMP_FAULT = (0x1 << 3);
const int pmacHardwareTurbo::PMAC_STATUS2_NEG_BACKLASH = (0x1 << 4);
const int pmacHardwareTurbo::PMAC_STATUS2_I2T_AMP_FAULT = (0x1 << 5);
const int pmacHardwareTurbo::PMAC_STATUS2_I2_FOLLOW_ERR = (0x1 << 6);
const int pmacHardwareTurbo::PMAC_STATUS2_TRIGGER_MOVE = (0x1 << 7);
const int pmacHardwareTurbo::PMAC_STATUS2_PHASE_REF_ERR = (0x1 << 8);
const int pmacHardwareTurbo::PMAC_STATUS2_PHASE_SEARCH = (0x1 << 9);
const int pmacHardwareTurbo::PMAC_STATUS2_HOME_COMPLETE = (0x1 << 10);
const int pmacHardwareTurbo::PMAC_STATUS2_POS_LIMIT_STOP = (0x1 << 11);
const int pmacHardwareTurbo::PMAC_STATUS2_DESIRED_STOP = (0x1 << 12);
const int pmacHardwareTurbo::PMAC_STATUS2_FORE_IN_POS = (0x1 << 13);
const int pmacHardwareTurbo::PMAC_STATUS2_NA14 = (0x1 << 14);
const int pmacHardwareTurbo::PMAC_STATUS2_ASSIGNED_CS = (0x1 << 15);

const int pmacHardwareTurbo::CS_STATUS1_RUNNING_PROG = (0x1 << 0);
const int pmacHardwareTurbo::CS_STATUS1_SINGLE_STEP_MODE = (0x1 << 1);
const int pmacHardwareTurbo::CS_STATUS1_CONTINUOUS_MODE = (0x1 << 2);
const int pmacHardwareTurbo::CS_STATUS1_MOVE_BY_TIME_MODE = (0x1 << 3);
const int pmacHardwareTurbo::CS_STATUS1_CONTINUOUS_REQUEST = (0x1 << 4);
const int pmacHardwareTurbo::CS_STATUS1_RADIUS_INC_MODE = (0x1 << 5);
const int pmacHardwareTurbo::CS_STATUS1_A_INC = (0x1 << 6);
const int pmacHardwareTurbo::CS_STATUS1_A_FEEDRATE = (0x1 << 7);
const int pmacHardwareTurbo::CS_STATUS1_B_INC = (0x1 << 8);
const int pmacHardwareTurbo::CS_STATUS1_B_FEEDRATE = (0x1 << 9);
const int pmacHardwareTurbo::CS_STATUS1_C_INC = (0x1 << 10);
const int pmacHardwareTurbo::CS_STATUS1_C_FEEDRATE = (0x1 << 11);
const int pmacHardwareTurbo::CS_STATUS1_U_INC = (0x1 << 12);
const int pmacHardwareTurbo::CS_STATUS1_U_FEEDRATE = (0x1 << 13);
const int pmacHardwareTurbo::CS_STATUS1_V_INC = (0x1 << 14);
const int pmacHardwareTurbo::CS_STATUS1_V_FEEDRATE = (0x1 << 15);
const int pmacHardwareTurbo::CS_STATUS1_W_INC = (0x1 << 16);
const int pmacHardwareTurbo::CS_STATUS1_W_FEEDRATE = (0x1 << 17);
const int pmacHardwareTurbo::CS_STATUS1_X_INC = (0x1 << 18);
const int pmacHardwareTurbo::CS_STATUS1_X_FEEDRATE = (0x1 << 19);
const int pmacHardwareTurbo::CS_STATUS1_Y_INC = (0x1 << 20);
const int pmacHardwareTurbo::CS_STATUS1_Y_FEEDRATE = (0x1 << 21);
const int pmacHardwareTurbo::CS_STATUS1_Z_INC = (0x1 << 22);
const int pmacHardwareTurbo::CS_STATUS1_Z_FEEDRATE = (0x1 << 23);

const int pmacHardwareTurbo::CS_STATUS2_CIRCLE_SPLINE_MODE = (0x1 << 0);
const int pmacHardwareTurbo::CS_STATUS2_CCW_RAPID_MODE = (0x1 << 1);
const int pmacHardwareTurbo::CS_STATUS2_2D_CUTTER_COMP = (0x1 << 2);
const int pmacHardwareTurbo::CS_STATUS2_2D_LEFT_3D_CUTTER = (0x1 << 3);
const int pmacHardwareTurbo::CS_STATUS2_PVT_SPLINE_MODE = (0x1 << 4);
const int pmacHardwareTurbo::CS_STATUS2_SEG_STOPPING = (0x1 << 5);
const int pmacHardwareTurbo::CS_STATUS2_SEG_ACCEL = (0x1 << 6);
const int pmacHardwareTurbo::CS_STATUS2_SEG_MOVING = (0x1 << 7);
const int pmacHardwareTurbo::CS_STATUS2_PRE_JOG = (0x1 << 8);
const int pmacHardwareTurbo::CS_STATUS2_CUTTER_MOVE_BUFFD = (0x1 << 9);
const int pmacHardwareTurbo::CS_STATUS2_CUTTER_STOP = (0x1 << 10);
const int pmacHardwareTurbo::CS_STATUS2_CUTTER_COMP_OUTSIDE = (0x1 << 11);
const int pmacHardwareTurbo::CS_STATUS2_DWELL_MOVE_BUFFD = (0x1 << 12);
const int pmacHardwareTurbo::CS_STATUS2_SYNCH_M_ONESHOT = (0x1 << 13);
const int pmacHardwareTurbo::CS_STATUS2_EOB_STOP = (0x1 << 14);
const int pmacHardwareTurbo::CS_STATUS2_DELAYED_CALC = (0x1 << 15);
const int pmacHardwareTurbo::CS_STATUS2_ROTARY_BUFF = (0x1 << 16);
const int pmacHardwareTurbo::CS_STATUS2_IN_POSITION = (0x1 << 17);
const int pmacHardwareTurbo::CS_STATUS2_FOLLOW_WARN = (0x1 << 18);
const int pmacHardwareTurbo::CS_STATUS2_FOLLOW_ERR = (0x1 << 19);
const int pmacHardwareTurbo::CS_STATUS2_AMP_FAULT = (0x1 << 20);
const int pmacHardwareTurbo::CS_STATUS2_MOVE_IN_STACK = (0x1 << 21);
const int pmacHardwareTurbo::CS_STATUS2_RUNTIME_ERR = (0x1 << 22);
const int pmacHardwareTurbo::CS_STATUS2_LOOKAHEAD = (0x1 << 23);

const int pmacHardwareTurbo::CS_STATUS3_LIMIT = (0x1 << 1);

pmacHardwareTurbo::pmacHardwareTurbo() : pmacDebugger("pmacHardwareTurbo") {
}

pmacHardwareTurbo::~pmacHardwareTurbo() {
}

std::string pmacHardwareTurbo::getGlobalStatusCmd() {
  return GLOBAL_STATUS;
}

asynStatus
pmacHardwareTurbo::parseGlobalStatus(const std::string &statusString, globalStatus &globStatus) {
  asynStatus status = asynSuccess;
  int nvals = 0;
  static const char *functionName = "parseGlobalStatus";

  if (statusString == "") {
    debug(DEBUG_ERROR, functionName, "Problem reading global status command, returned",
          statusString);
    status = asynError;
  } else {
    // Turbo PMAC status parsing
    nvals = sscanf(statusString.c_str(), "%6x", &globStatus.status_);
    if (nvals != 1) {
      debug(DEBUG_ERROR, functionName, "Error reading global status", GLOBAL_STATUS);
      debug(DEBUG_ERROR, functionName, "    nvals", nvals);
      debug(DEBUG_ERROR, functionName, "    response", statusString);
      status = asynError;
    }
    nvals = sscanf(statusString.c_str(), "%4x%4x%4x", &globStatus.stat1_, &globStatus.stat2_,
                   &globStatus.stat3_);
    if (nvals != 3) {
      status = asynError;
    }
  }
  return status;
}

std::string pmacHardwareTurbo::getAxisStatusCmd(int axis) {
  char cmd[8];
  static const char *functionName = "getAxisStatusCmd";

  debug(DEBUG_TRACE, functionName, "Axis", axis);
  sprintf(cmd, AXIS_STATUS.c_str(), axis);
  return std::string(cmd);
}

asynStatus pmacHardwareTurbo::setupAxisStatus(int axis) {
  asynStatus status = asynSuccess;
  static const char *functionName = "setupAxisStatus";

  debug(DEBUG_TRACE, functionName, "Axis", axis);
  // No-op for turbo
  return status;
}

asynStatus
pmacHardwareTurbo::parseAxisStatus(int axis, pmacCommandStore *sPtr, axisStatus &axStatus) {
  asynStatus status = asynSuccess;
  int nvals = 0;
  std::string statusString = "";
  static const char *functionName = "parseAxisStatus";

  statusString = sPtr->readValue(this->getAxisStatusCmd(axis));

  nvals = sscanf(statusString.c_str(), "%6x%6x", &axStatus.status24Bit1_, &axStatus.status24Bit2_);
  if (nvals != 2) {
    debug(DEBUG_ERROR, functionName, "Failed to parse axis status", statusString);
    status = asynError;
  }
  debug(DEBUG_VARIABLE, functionName, "Read status[0]", axStatus.status24Bit1_);
  debug(DEBUG_VARIABLE, functionName, "Read status[1]", axStatus.status24Bit2_);
  nvals = sscanf(statusString.c_str(), "%4x%4x%4x", &axStatus.status16Bit1_,
                 &axStatus.status16Bit2_, &axStatus.status16Bit3_);
  if (nvals != 3) {
    debug(DEBUG_ERROR, functionName, "Failed to parse axis status", statusString);
    status = asynError;
  }

  if (status == asynSuccess) {
    axStatus.home_ = ((axStatus.status24Bit2_ & PMAC_STATUS2_HOME_COMPLETE) != 0);

    axStatus.done_ = (((axStatus.status24Bit2_ & PMAC_STATUS2_IN_POSITION) != 0) ||
                      ((axStatus.status24Bit1_ & PMAC_STATUS1_MOTOR_ON) == 0));
    // If we are not done, but amp has been disabled, then set done (to stop when we get following errors).
    if ((axStatus.done_ == 0) && ((axStatus.status24Bit1_ & PMAC_STATUS1_AMP_ENABLED) == 0)) {
      axStatus.done_ = 1;
    }

    // Read the currently assigned CS for the axis, and whether it is assigned at all
    if ((axStatus.status24Bit2_ & PMAC_STATUS2_ASSIGNED_CS) != 0) {
      axStatus.currentCS_ = axStatus.status24Bit2_ >> 20;
      axStatus.currentCS_++;
    } else {
      axStatus.currentCS_ = 0;
    }

    axStatus.highLimit_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_POS_LIMIT_SET) != 0);
    axStatus.lowLimit_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_NEG_LIMIT_SET) != 0);
    axStatus.followingError_ = ((axStatus.status24Bit2_ & PMAC_STATUS2_ERR_FOLLOW_ERR) != 0);
    axStatus.power_ = (!(axStatus.status24Bit1_ & PMAC_STATUS1_OPEN_LOOP));
    // If desired_vel_zero is false && motor activated (ix00=1) && amplifier enabled, set moving=1.
    axStatus.moving_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_DESIRED_VELOCITY_ZERO) == 0) &&
                       ((axStatus.status24Bit1_ & PMAC_STATUS1_MOTOR_ON) != 0) &&
                       ((axStatus.status24Bit1_ & PMAC_STATUS1_AMP_ENABLED) != 0);
    if ((axStatus.status24Bit1_ & PMAC_STATUS1_AMP_ENABLED) != 0) {
      axStatus.ampEnabled_ = 1;
    } else {
      axStatus.ampEnabled_ = 0;
    }
  }

  return status;
}

asynStatus pmacHardwareTurbo::setupCSStatus(int csNo) {
  asynStatus status = asynSuccess;
  char var[16];
  static const char *functionName = "setupCSStatus";

  debug(DEBUG_TRACE, functionName, "CS Number", csNo);
  // Add the CS status item to the fast update
  sprintf(var, CS_STATUS.c_str(), csNo);
  // status goes into PreFast poller which will be guaranteed to be called before
  // Fast poller (with the parameter lock held). Thus if motors have just stopped
  // the readback will happen after the stop status is picked up
  pC_->monitorPMACVariable(pmacMessageBroker::PMAC_PRE_FAST_READ, var);

  return status;
}

asynStatus
pmacHardwareTurbo::parseCSStatus(int csNo, pmacCommandStore *sPtr, csStatus &coordStatus) {
  asynStatus status = asynSuccess;
  int nvals = 0;
  std::string statusString = "";
  char var[16];
  static const char *functionName = "parseCSStatus";

  sprintf(var, CS_STATUS.c_str(), csNo);
  statusString = sPtr->readValue(var);
  // Parse the status
  nvals = sscanf(statusString.c_str(), "%6x%6x%6x", &coordStatus.stat1_, &coordStatus.stat2_,
                 &coordStatus.stat3_);
  if (nvals != 3) {
    debug(DEBUG_ERROR, functionName, "Failed to parse CS status. ", statusString);
    coordStatus.stat1_ = 0;
    coordStatus.stat2_ = 0;
    coordStatus.stat3_ = 0;
    status = asynError;
  }
  if (status == asynSuccess) {
    coordStatus.running_ = (coordStatus.stat1_ & CS_STATUS1_RUNNING_PROG) != 0;
    coordStatus.done_ = ((coordStatus.stat1_ & CS_STATUS1_RUNNING_PROG) == 0) &&
                        ((coordStatus.stat2_ & CS_STATUS2_IN_POSITION) != 0);
    coordStatus.highLimit_ = ((coordStatus.stat3_ & CS_STATUS3_LIMIT) != 0);
    coordStatus.lowLimit_ = ((coordStatus.stat3_ & CS_STATUS3_LIMIT) != 0);
    coordStatus.followingError_ = ((coordStatus.stat2_ & CS_STATUS2_FOLLOW_ERR) != 0);
    coordStatus.moving_ = ((coordStatus.stat2_ & CS_STATUS2_IN_POSITION) == 0);
    coordStatus.problem_ = (((coordStatus.stat2_ & CS_STATUS2_AMP_FAULT) != 0) ||
                            ((coordStatus.stat2_ & CS_STATUS2_RUNTIME_ERR) != 0));
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

std::string pmacHardwareTurbo::getCSVelocityCmd(int csNo, double velocity, double steps) {
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

std::string pmacHardwareTurbo::getCSAccTimeCmd(int csNo, double time) {
  char cmd[64];
  static const char *functionName = "getCSAccTimeCmd";

  debug(DEBUG_FLOW, functionName, "CS Number", csNo);
  debug(DEBUG_FLOW, functionName, "time", time);
  sprintf(cmd, CS_ACCELERATION_CMD.c_str(), csNo + 50, time);
  return std::string(cmd);
}

std::string pmacHardwareTurbo::getCSMappingCmd(int csNo, int axis) {
  char cmd[10];
  static const char *functionName = "getCSMappingCmd";

  debugf(DEBUG_FLOW, functionName, "CsNo %d, Axis %d", csNo, axis);
  sprintf(cmd, CS_AXIS_MAPPING.c_str(), csNo, axis);
  return std::string(cmd);
}

std::string pmacHardwareTurbo::getCSEnabledCountCmd(){
  return std::string(CS_ENABLED_COUNT);
}

std::string pmacHardwareTurbo::parseCSMappingResult(const std::string mappingResult) {
  return mappingResult;
}

void pmacHardwareTurbo::startTrajectoryTimePointsCmd(char *vel_cmd, char *user_cmd,
                                                     char *time_cmd, int addr) {
  static const char *functionName = "startTrajectoryTimePointsCmd";

  debug(DEBUG_FLOW, functionName, "addr %d", addr);

  sprintf(vel_cmd, "WL:$%X", addr);
  user_cmd[0] = time_cmd[0] = 0;
}

void pmacHardwareTurbo::addTrajectoryTimePointCmd(char *velCmd, char *userCmd, char *timeCmd,
                                                  int velocityMode, int userFunc, int time,
                                                  bool ) {
  static const char *functionName = "addTrajectoryTimePointCmd";

  debugf(DEBUG_FLOW, functionName, "velCmd %s\nuserCmd %s\ntimeCmd %s\nvel %d, user %d, time %d",
         velCmd, userCmd, timeCmd, velocityMode, userFunc, time);

  sprintf(velCmd, "%s,$%01X%01X%06X", velCmd, velocityMode, userFunc, time);
  userCmd[0] = timeCmd[0] = 0;
}

void pmacHardwareTurbo::startAxisPointsCmd(char *axisCmd, int axis, int addr, int buffSize) {
  static const char *functionName = "startAxisPointsCmd";

  debugf(DEBUG_FLOW, functionName, "cmd %s, axis %d, addr %d", axisCmd, axis, addr);
  sprintf(axisCmd, "WL:$%X", addr + ((axis + 1) * buffSize));
}

void pmacHardwareTurbo::addAxisPointCmd(char *axisCmd, int , double pos, int ,
        bool ) {
  int64_t ival = 0;
  static const char *functionName = "addAxisPointCmd";

  debugf(DEBUG_FLOW, functionName, "cmd %s, pos %f", axisCmd, pos);
  doubleToPMACFloat(pos, &ival);
  sprintf(axisCmd, "%s,$%lX", axisCmd, (long) ival);
}

std::string pmacHardwareTurbo::getCSEnableCommand(int csNo) {
  char cmd[10];
  static const char *functionName = "getCSEnableCommand";

  debugf(DEBUG_FLOW, functionName, "cmd %s, CS %d", cmd, csNo);
  sprintf(cmd, "&%de", csNo);
  return std::string(cmd);
}


asynStatus pmacHardwareTurbo::doubleToPMACFloat(double value, int64_t *representation) {
  asynStatus status = asynSuccess;
  double absVal = value;
  int negative = 0;
  int exponent = 0;
  double expVal = 0.0;
  int64_t intVal = 0;
  int64_t tVal = 0;
  double mantissaVal = 0.0;
  double maxMantissa = 34359738368.0;  // 0x800000000
  const char *functionName = "doubleToPMACFloat";

  debug(DEBUG_FLOW, functionName);
  debugf(DEBUG_VARIABLE, functionName, "Value : %20.10lf\n", value);

  // Check for special case 0.0
  if (absVal == 0.0) {
    // Set value accordingly
    tVal = 0x0;
  } else {
    // Check for a negative number, and get the absolute
    if (absVal < 0.0) {
      absVal = absVal * -1.0;
      negative = 1;
    }
    expVal = absVal;
    mantissaVal = absVal;

    // Work out the exponent required to normalise
    // Normalised should be between 1 and 2
    while (expVal >= 2.0) {
      expVal = expVal / 2.0;
      exponent++;
    }
    while (expVal < 1.0) {
      expVal = expVal * 2.0;
      exponent--;
    }
    // Offset exponent to provide +-2048 range
    exponent += 0x800;

    // Get the mantissa into correct format, this might not be
    // the most efficient way to do this
    while (mantissaVal < maxMantissa) {
      mantissaVal *= 2.0;
    }
    mantissaVal = mantissaVal / 2.0;
    // Get the integer representation for the altered mantissa
    intVal = (int64_t) mantissaVal;

    // If negative value then subtract altered mantissa from max
    if (negative == 1) {
      intVal = 0xFFFFFFFFFLL - intVal;
    }

    // Shift the altered mantissa by 12 bits and then set those
    // 12 bits to the offset exponent
    tVal = intVal << 12;
    tVal += exponent;
  }

  *representation = tVal;

  debugf(DEBUG_VARIABLE, functionName, "Prepared value: %12lX\n", tVal);

  return status;
}
