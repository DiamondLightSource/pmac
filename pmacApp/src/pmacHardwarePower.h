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

class pmacHardwarePower : public pmacHardwareInterface, pmacDebugger {
public:
    pmacHardwarePower();

    virtual ~pmacHardwarePower();

    std::string getGlobalStatusCmd();

    asynStatus parseGlobalStatus(const std::string &statusString, globalStatus &globStatus);

    std::string getAxisStatusCmd(int axis);

    asynStatus setupAxisStatus(int axis);

    asynStatus parseAxisStatus(int axis, pmacCommandStore *sPtr, axisStatus &axStatus);

    asynStatus setupCSStatus(int csNo);

    asynStatus parseCSStatus(int csNo, pmacCommandStore *sPtr, csStatus &coordStatus);

    std::string getCSVelocityCmd(int csNo, double velocity);

    std::string getCSAccTimeCmd(int csNo, double time);

private:
    static const std::string GLOBAL_STATUS;
    static const std::string AXIS_STATUS;
    static const std::string AXIS_CS_NUMBER;
    static const std::string CS_STATUS;
    static const std::string CS_VEL_CMD;
    static const std::string CS_ACCELERATION_CMD;

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
