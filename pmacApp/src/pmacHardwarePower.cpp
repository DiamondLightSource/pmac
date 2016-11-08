/*
 * pmacHardwarePower.cpp
 *
 *  Created on: 27 Oct 2016
 *      Author: gnx91527
 */

#include "pmacHardwarePower.h"

const std::string pmacHardwarePower::GLOBAL_STATUS = "?";
const std::string pmacHardwarePower::AXIS_STATUS = "#%d?";

const int pmacHardwarePower::PMAC_STATUS1_TRIGGER_MOVE           = (0x1<<31);
const int pmacHardwarePower::PMAC_STATUS1_HOMING                 = (0x1<<30);
const int pmacHardwarePower::PMAC_STATUS1_NEG_LIMIT_SET          = (0x1<<29);
const int pmacHardwarePower::PMAC_STATUS1_POS_LIMIT_SET          = (0x1<<28);
const int pmacHardwarePower::PMAC_STATUS1_WARN_FOLLOW_ERR        = (0x1<<27);
const int pmacHardwarePower::PMAC_STATUS1_ERR_FOLLOW_ERR         = (0x1<<26);
const int pmacHardwarePower::PMAC_STATUS1_LIMIT_STOP             = (0x1<<25);
const int pmacHardwarePower::PMAC_STATUS1_AMP_FAULT              = (0x1<<24);
const int pmacHardwarePower::PMAC_STATUS1_SOFT_MINUS_LIMIT       = (0x1<<23);
const int pmacHardwarePower::PMAC_STATUS1_SOFT_PLUS_LIMIT        = (0x1<<22);
const int pmacHardwarePower::PMAC_STATUS1_I2T_AMP_FAULT          = (0x1<<21);
const int pmacHardwarePower::PMAC_STATUS1_HOME_COMPLETE          = (0x1<<15);
const int pmacHardwarePower::PMAC_STATUS1_DESIRED_VELOCITY_ZERO  = (0x1<<14);
const int pmacHardwarePower::PMAC_STATUS1_OPEN_LOOP              = (0x1<<13);
const int pmacHardwarePower::PMAC_STATUS1_AMP_ENABLED            = (0x1<<12);
const int pmacHardwarePower::PMAC_STATUS1_IN_POSITION            = (0x1<<11);
const int pmacHardwarePower::PMAC_STATUS1_BLOCK_REQUEST          = (0x1<<9);
const int pmacHardwarePower::PMAC_STATUS1_PHASED_MOTOR           = (0x1<<8);

pmacHardwarePower::pmacHardwarePower() : pmacDebugger("pmacHardwareTurbo")
{
}

pmacHardwarePower::~pmacHardwarePower()
{
  // TODO Auto-generated destructor stub
}

std::string pmacHardwarePower::getGlobalStatusCmd()
{
  return GLOBAL_STATUS;
}

asynStatus pmacHardwarePower::parseGlobalStatus(const std::string& statusString, globalStatus &globStatus)
{
  asynStatus status = asynSuccess;
  int nvals = 0;
  static const char *functionName = "parseGlobalStatus";

  debug(DEBUG_VARIABLE, functionName, "Status string", statusString);
  if (statusString == ""){
    debug(DEBUG_ERROR, functionName, "Problem reading global status command, returned", statusString);
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
    if (nvals != 2){
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

std::string pmacHardwarePower::getAxisStatusCmd(int axis)
{
  char cmd[8];
  static const char *functionName = "getAxisStatusCmd";

  debug(DEBUG_TRACE, functionName, "Axis", axis);
  sprintf(cmd, AXIS_STATUS.c_str(), axis);
  return std::string(cmd);
}

asynStatus pmacHardwarePower::parseAxisStatus(const std::string& statusString, axisStatus& axStatus)
{
  asynStatus status = asynSuccess;
  int nvals = 0;
  int dummyVal = 0;
  static const char *functionName = "parseAxisStatus";

  // Response parsed for PowerPMAC
  debug(DEBUG_VARIABLE, functionName, "Status string", statusString);
  nvals = sscanf(statusString.c_str(), " $%8x%8x", &axStatus.status24Bit1_, &axStatus.status24Bit2_);
  if (nvals != 2){
    debug(DEBUG_ERROR, functionName, "Failed to parse axis status (24 bit)", statusString);
    axStatus.status24Bit1_ = 0;
    axStatus.status24Bit2_ = 0;
    status = asynError;
  }
  nvals = sscanf(statusString.c_str(), " $%4x%4x%4x%4x", &axStatus.status16Bit1_, &axStatus.status16Bit2_, &axStatus.status16Bit3_, &dummyVal);
  if (nvals != 4){
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

  if (status == asynSuccess){
    axStatus.home_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_HOME_COMPLETE) != 0);

    axStatus.done_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_IN_POSITION) != 0);
    /*If we are not done, but amp has been disabled, then set done (to stop when we get following errors).*/
    if ((axStatus.done_ == 0) && ((axStatus.status24Bit1_ & PMAC_STATUS1_AMP_ENABLED) == 0)) {
      axStatus.done_ = 1;
    }

    axStatus.highLimit_ = (((axStatus.status24Bit1_ & PMAC_STATUS1_POS_LIMIT_SET) | (axStatus.status24Bit1_ & PMAC_STATUS1_SOFT_PLUS_LIMIT)) != 0);
    // If desired_vel_zero is false && motor activated (ix00=1) && amplifier enabled, set moving=1.
    axStatus.moving_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_DESIRED_VELOCITY_ZERO) == 0) && ((axStatus.status24Bit1_ & PMAC_STATUS1_AMP_ENABLED) != 0);
    axStatus.lowLimit_ = (((axStatus.status24Bit1_ & PMAC_STATUS1_NEG_LIMIT_SET) | (axStatus.status24Bit1_ & PMAC_STATUS1_SOFT_MINUS_LIMIT))!=0);
    axStatus.followingError_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_ERR_FOLLOW_ERR) != 0);

    // Set amplifier enabled bit.
    axStatus.ampEnabled_ = ((axStatus.status24Bit1_ & PMAC_STATUS1_AMP_ENABLED) != 0);
  }
  return status;
}
