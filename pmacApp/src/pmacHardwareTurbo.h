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

class pmacHardwareTurbo : public pmacHardwareInterface, pmacDebugger {
public:
    pmacHardwareTurbo();

    virtual ~pmacHardwareTurbo();

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
    asynStatus doubleToPMACFloat(double value, int64_t *representation);

    static const std::string GLOBAL_STATUS;
    static const std::string AXIS_STATUS;
    static const std::string CS_STATUS;
    static const std::string CS_VEL_CMD;
    static const std::string CS_ACCELERATION_CMD;
    static const std::string CS_AXIS_MAPPING;
    static const std::string CS_ENABLED_COUNT;

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

    static const int CS_STATUS1_RUNNING_PROG;        // (0x1<<0)
    static const int CS_STATUS1_SINGLE_STEP_MODE;    // (0x1<<1)
    static const int CS_STATUS1_CONTINUOUS_MODE;     // (0x1<<2)
    static const int CS_STATUS1_MOVE_BY_TIME_MODE;   // (0x1<<3)
    static const int CS_STATUS1_CONTINUOUS_REQUEST;  // (0x1<<4)
    static const int CS_STATUS1_RADIUS_INC_MODE;     // (0x1<<5)
    static const int CS_STATUS1_A_INC;               // (0x1<<6)
    static const int CS_STATUS1_A_FEEDRATE;          // (0x1<<7)
    static const int CS_STATUS1_B_INC;               // (0x1<<8)
    static const int CS_STATUS1_B_FEEDRATE;          // (0x1<<9)
    static const int CS_STATUS1_C_INC;               // (0x1<<10)
    static const int CS_STATUS1_C_FEEDRATE;          // (0x1<<11)
    static const int CS_STATUS1_U_INC;               // (0x1<<12)
    static const int CS_STATUS1_U_FEEDRATE;          // (0x1<<13)
    static const int CS_STATUS1_V_INC;               // (0x1<<14)
    static const int CS_STATUS1_V_FEEDRATE;          // (0x1<<15)
    static const int CS_STATUS1_W_INC;               // (0x1<<16)
    static const int CS_STATUS1_W_FEEDRATE;          // (0x1<<17)
    static const int CS_STATUS1_X_INC;               // (0x1<<18)
    static const int CS_STATUS1_X_FEEDRATE;          // (0x1<<19)
    static const int CS_STATUS1_Y_INC;               // (0x1<<20)
    static const int CS_STATUS1_Y_FEEDRATE;          // (0x1<<21)
    static const int CS_STATUS1_Z_INC;               // (0x1<<22)
    static const int CS_STATUS1_Z_FEEDRATE;          // (0x1<<23)

    static const int CS_STATUS2_CIRCLE_SPLINE_MODE;  // (0x1<<0)
    static const int CS_STATUS2_CCW_RAPID_MODE;      // (0x1<<1)
    static const int CS_STATUS2_2D_CUTTER_COMP;      // (0x1<<2)
    static const int CS_STATUS2_2D_LEFT_3D_CUTTER;   // (0x1<<3)
    static const int CS_STATUS2_PVT_SPLINE_MODE;     // (0x1<<4)
    static const int CS_STATUS2_SEG_STOPPING;        // (0x1<<5)
    static const int CS_STATUS2_SEG_ACCEL;           // (0x1<<6)
    static const int CS_STATUS2_SEG_MOVING;          // (0x1<<7)
    static const int CS_STATUS2_PRE_JOG;             // (0x1<<8)
    static const int CS_STATUS2_CUTTER_MOVE_BUFFD;   // (0x1<<9)
    static const int CS_STATUS2_CUTTER_STOP;         // (0x1<<10)
    static const int CS_STATUS2_CUTTER_COMP_OUTSIDE; // (0x1<<11)
    static const int CS_STATUS2_DWELL_MOVE_BUFFD;    // (0x1<<12)
    static const int CS_STATUS2_SYNCH_M_ONESHOT;     // (0x1<<13)
    static const int CS_STATUS2_EOB_STOP;            // (0x1<<14)
    static const int CS_STATUS2_DELAYED_CALC;        // (0x1<<15)
    static const int CS_STATUS2_ROTARY_BUFF;         // (0x1<<16)
    static const int CS_STATUS2_IN_POSITION;         // (0x1<<17)
    static const int CS_STATUS2_FOLLOW_WARN;         // (0x1<<18)
    static const int CS_STATUS2_FOLLOW_ERR;          // (0x1<<19)
    static const int CS_STATUS2_AMP_FAULT;           // (0x1<<20)
    static const int CS_STATUS2_MOVE_IN_STACK;       // (0x1<<21)
    static const int CS_STATUS2_RUNTIME_ERR;         // (0x1<<22)
    static const int CS_STATUS2_LOOKAHEAD;           // (0x1<<23)

    static const int CS_STATUS3_LIMIT;               // (0x1<<1)

    /*Global status ???*/
    static const int PMAC_GSTATUS_CARD_ADDR;                // (0x1 << 0)
    static const int PMAC_GSTATUS_ALL_CARD_ADDR;            // (0x1 << 1)
    static const int PMAC_GSTATUS_RESERVED;                 // (0x1 << 2)
    static const int PMAC_GSTATUS_PHASE_CLK_MISS;           // (0x1 << 3)
    static const int PMAC_GSTATUS_MACRO_RING_ERRORCHECK;    // (0x1 << 4)
    static const int PMAC_GSTATUS_MACRO_RING_COMMS;         // (0x1 << 5)
    static const int PMAC_GSTATUS_TWS_PARITY_ERROR;         // (0x1 << 6)
    static const int PMAC_GSTATUS_CONFIG_ERROR;             // (0x1 << 7)
    static const int PMAC_GSTATUS_ILLEGAL_LVAR;             // (0x1 << 8)
    static const int PMAC_GSTATUS_REALTIME_INTR;            // (0x1 << 9)
    static const int PMAC_GSTATUS_FLASH_ERROR;              // (0x1 << 10)
    static const int PMAC_GSTATUS_DPRAM_ERROR;              // (0x1 << 11)
    static const int PMAC_GSTATUS_CKSUM_ACTIVE;             // (0x1 << 12)
    static const int PMAC_GSTATUS_CKSUM_ERROR;              // (0x1 << 13)
    static const int PMAC_GSTATUS_LEADSCREW_COMP;           // (0x1 << 14)
    static const int PMAC_GSTATUS_WATCHDOG;                 // (0x1 << 15)
    static const int PMAC_GSTATUS_SERVO_REQ;                // (0x1 << 16)
    static const int PMAC_GSTATUS_DATA_GATHER_START;        // (0x1 << 17)
    static const int PMAC_GSTATUS_RESERVED2;                // (0x1 << 18)
    static const int PMAC_GSTATUS_DATA_GATHER_ON;           // (0x1 << 19)
    static const int PMAC_GSTATUS_SERVO_ERROR;              // (0x1 << 20)
    static const int PMAC_GSTATUS_CPUTYPE;                  // (0x1 << 21)
    static const int PMAC_GSTATUS_REALTIME_INTR_RE;         // (0x1 << 22)
    static const int PMAC_GSTATUS_RESERVED3;                // (0x1 << 23)

    static const int PMAC_HARDWARE_PROB;    // (PMAC_GSTATUS_REALTIME_INTR |
                                            //  PMAC_GSTATUS_FLASH_ERROR |
                                            //  PMAC_GSTATUS_DPRAM_ERROR |
                                            //  PMAC_GSTATUS_CKSUM_ERROR |
                                            //  PMAC_GSTATUS_WATCHDOG |
                                            //  PMAC_GSTATUS_SERVO_ERROR);

};

#endif /* PMACAPP_SRC_PMACHARDWARETURBO_H_ */
