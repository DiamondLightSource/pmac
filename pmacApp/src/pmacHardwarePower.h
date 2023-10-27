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

    int getGlobalStatusError();

    asynStatus parseGlobalStatus(const std::string &statusString, globalStatus &globStatus);

    std::string getAxisStatusCmd(int axis);

    asynStatus setupAxisStatus(int axis);

    asynStatus parseAxisStatus(int axis, pmacCommandStore *sPtr, axisStatus &axStatus);

    asynStatus setupCSStatus(int csNo);

    asynStatus parseCSStatus(int csNo, pmacCommandStore *sPtr, csStatus &coordStatus);

    std::string getCSVelocityCmd(int csNo, double velocity, double steps);

    std::string getCSAccTimeCmd(int csNo, double time);

    std::string getCSMappingCmd(int csNo, int axis);

    std::string getCSEnabledCountCmd();

    std::string parseCSMappingResult(const std::string mappingResult);

    void startTrajectoryTimePointsCmd(char *user_cmd, char *time_cmd,
                                      int addr);

    void addTrajectoryTimePointCmd(char *userCmd, char *timeCmd,
                                   int userFunc, int time,
                                   bool firstVal);

    void startAxisPointsCmd(char *axis_cmd, int axis, int addr, int buffSize, bool posCmd);

    void addAxisPointCmd(char *axis_cmd, int axis, double pos, int buffSize,
                                 bool firstVal);

    std::string getCSEnableCommand(int csNo);


private:
    static const std::string GLOBAL_STATUS;
    static const std::string AXIS_STATUS;
    static const std::string AXIS_CS_NUMBER;
    static const std::string CS_STATUS;
    static const std::string CS_INPOS;
    static const std::string CS_AMPENABLE;
    static const std::string CS_RUNNING;
    static const std::string CS_VEL_CMD;
    static const std::string CS_ACCELERATION_CMD;
    static const std::string CS_AXIS_MAPPING;
    static const std::string CS_ENABLED_COUNT;

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
    static const int PMAC_STATUS1_CLOSED_LOOP;
    static const int PMAC_STATUS1_AMP_ENABLED;
    static const int PMAC_STATUS1_IN_POSITION;
    static const int PMAC_STATUS1_BLOCK_REQUEST;
    static const int PMAC_STATUS1_PHASED_MOTOR;

    /*Global status ?*/
    static const int PMAC_GSTATUS_FOREGROUND_WDT_FAULT;     // (0x1 << 0);
    static const int PMAC_GSTATUS_BACKGROUND_WDT_FAULT;     // (0x1 << 1);
    static const int PMAC_GSTATUS_PWR_ON_FAULT;             // (0x1 << 2);
    static const int PMAC_GSTATUS_PROJECT_LOAD_ERROR;       // (0x1 << 3);
    static const int PMAC_GSTATUS_CONFIG_LOAD_ERROR;        // (0x1 << 4);
    static const int PMAC_GSTATUS_HW_CHANGE_ERROR;          // (0x1 << 5);
    static const int PMAC_GSTATUS_FILE_CONFIG_ERROR;        // (0x1 << 6);
    static const int PMAC_GSTATUS_DEFAULT;                  // (0x1 << 7);
    static const int PMAC_GSTATUS_NO_CLOCKS;                // (0x1 << 8);
    static const int PMAC_GSTATUS_ABORTALL;                 // (0x1 << 9);
    static const int PMAC_GSTATUS_BUF_SIZE_ERROR;           // (0x1 << 10);
    static const int PMAC_GSTATUS_FLASH_SIZE_ERROR;         // (0x1 << 11);
    static const int PMAC_GSTATUS_CK3W_CONFIG_ERROR0;       // (0x1 << 12);
    static const int PMAC_GSTATUS_CK3W_CONFIG_ERROR1;       // (0x1 << 13);
    static const int PMAC_GSTATUS_CK3W_CONFIG_ERROR2;       // (0x1 << 14);
    static const int PMAC_GSTATUS_CK3W_HW_CHANGE;           // (0x1 << 15);

    static const int PMAC_HARDWARE_PROB;    // (PMAC_GSTATUS_FOREGROUND_WDT_FAULT |
                                            // PMAC_GSTATUS_BACKGROUND_WDT_FAULT |
                                            // PMAC_GSTATUS_PROJECT_LOAD_ERROR |
                                            // PMAC_GSTATUS_CONFIG_LOAD_ERROR |
                                            // PMAC_GSTATUS_HW_CHANGE_ERROR |
                                            // PMAC_GSTATUS_FILE_CONFIG_ERROR |
                                            // PMAC_GSTATUS_NO_CLOCKS |
                                            // //PMAC_GSTATUS_ABORTALL |
                                            // PMAC_GSTATUS_BUF_SIZE_ERROR |
                                            // PMAC_GSTATUS_FLASH_SIZE_ERROR);

};

#endif /* PMACAPP_SRC_PMACHARDWAREPOWER_H_ */
