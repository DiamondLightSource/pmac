/*
 * pmacHardwareTurbo.h
 *
 *  Created on: 27 Oct 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_PMACHARDWARETURBO_H_
#define PMACAPP_SRC_PMACHARDWARETURBO_H_

#include "pmacHardwareInterface.h"
#include "pmacDebugger.h"

class pmacHardwareTurbo : public pmacHardwareInterface, pmacDebugger
{
  public:
    pmacHardwareTurbo();
    virtual ~pmacHardwareTurbo();
    std::string getGlobalStatusCmd();
    asynStatus parseGlobalStatus(const std::string& statusString, globalStatus &globStatus);
    std::string getAxisStatusCmd(int axis);
    asynStatus parseAxisStatus(const std::string& statusString, axisStatus &axStatus);

  private:
    static const std::string GLOBAL_STATUS;
    static const std::string AXIS_STATUS;

    static const int PMAC_STATUS1_MAXRAPID_SPEED;
    static const int PMAC_STATUS1_ALT_CMNDOUT_MODE;
    static const int PMAC_STATUS1_SOFT_POS_CAPTURE;
    static const int PMAC_STATUS1_ERROR_TRIGGER;
    static const int PMAC_STATUS1_FOLLOW_ENABLE;
    static const int PMAC_STATUS1_FOLLOW_OFFSET;
    static const int PMAC_STATUS1_PHASED_MOTOR;
    static const int PMAC_STATUS1_ALT_SRC_DEST;
    static const int PMAC_STATUS1_USER_SERVO;
    static const int PMAC_STATUS1_USER_PHASE;
    static const int PMAC_STATUS1_HOMING;
    static const int PMAC_STATUS1_BLOCK_REQUEST;
    static const int PMAC_STATUS1_DECEL_ABORT;
    static const int PMAC_STATUS1_DESIRED_VELOCITY_ZERO;
    static const int PMAC_STATUS1_DATABLKERR;
    static const int PMAC_STATUS1_DWELL;
    static const int PMAC_STATUS1_INTEGRATE_MODE;
    static const int PMAC_STATUS1_MOVE_TIME_ON;
    static const int PMAC_STATUS1_OPEN_LOOP;
    static const int PMAC_STATUS1_AMP_ENABLED;
    static const int PMAC_STATUS1_X_SERVO_ON;
    static const int PMAC_STATUS1_POS_LIMIT_SET;
    static const int PMAC_STATUS1_NEG_LIMIT_SET;
    static const int PMAC_STATUS1_MOTOR_ON;

    static const int PMAC_STATUS2_IN_POSITION;
    static const int PMAC_STATUS2_WARN_FOLLOW_ERR;
    static const int PMAC_STATUS2_ERR_FOLLOW_ERR;
    static const int PMAC_STATUS2_AMP_FAULT;
    static const int PMAC_STATUS2_NEG_BACKLASH;
    static const int PMAC_STATUS2_I2T_AMP_FAULT;
    static const int PMAC_STATUS2_I2_FOLLOW_ERR;
    static const int PMAC_STATUS2_TRIGGER_MOVE;
    static const int PMAC_STATUS2_PHASE_REF_ERR;
    static const int PMAC_STATUS2_PHASE_SEARCH;
    static const int PMAC_STATUS2_HOME_COMPLETE;
    static const int PMAC_STATUS2_POS_LIMIT_STOP;
    static const int PMAC_STATUS2_DESIRED_STOP;
    static const int PMAC_STATUS2_FORE_IN_POS;
    static const int PMAC_STATUS2_NA14;
    static const int PMAC_STATUS2_ASSIGNED_CS;

};

#endif /* PMACAPP_SRC_PMACHARDWARETURBO_H_ */
