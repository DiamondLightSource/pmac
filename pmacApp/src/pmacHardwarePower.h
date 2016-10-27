/*
 * pmacHardwarePower.h
 *
 *  Created on: 27 Oct 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_PMACHARDWAREPOWER_H_
#define PMACAPP_SRC_PMACHARDWAREPOWER_H_

#include "pmacHardwareInterface.h"
#include "pmacDebugger.h"

class pmacHardwarePower : public pmacHardwareInterface, pmacDebugger
{
  public:
    pmacHardwarePower();
    virtual ~pmacHardwarePower();
    std::string getGlobalStatusCmd();
    asynStatus parseGlobalStatus(const std::string& statusString, globalStatus &globStatus);
    std::string getAxisStatusCmd(int axis);
    asynStatus parseAxisStatus(const std::string& statusString, axisStatus &axStatus);

  private:
    static const std::string GLOBAL_STATUS;
    static const std::string AXIS_STATUS;

    static const int PMAC_STATUS1_TRIGGER_MOVE;
    static const int PMAC_STATUS1_HOMING;
    static const int PMAC_STATUS1_NEG_LIMIT_SET;
    static const int PMAC_STATUS1_POS_LIMIT_SET;
    static const int PMAC_STATUS1_WARN_FOLLOW_ERR;
    static const int PMAC_STATUS1_ERR_FOLLOW_ERR;
    static const int PMAC_STATUS1_LIMIT_STOP;
    static const int PMAC_STATUS1_AMP_FAULT;
    static const int PMAC_STATUS1_SOFT_MINUS_LIMIT;
    static const int PMAC_STATUS1_SOFT_PLUS_LIMIT;
    static const int PMAC_STATUS1_I2T_AMP_FAULT;
    static const int PMAC_STATUS1_HOME_COMPLETE;
    static const int PMAC_STATUS1_DESIRED_VELOCITY_ZERO;
    static const int PMAC_STATUS1_OPEN_LOOP;
    static const int PMAC_STATUS1_AMP_ENABLED;
    static const int PMAC_STATUS1_IN_POSITION;
    static const int PMAC_STATUS1_BLOCK_REQUEST;
    static const int PMAC_STATUS1_PHASED_MOTOR;

};

#endif /* PMACAPP_SRC_PMACHARDWAREPOWER_H_ */
