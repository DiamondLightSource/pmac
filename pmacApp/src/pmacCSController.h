/********************************************
 *  pmacCSController.h
 *
 *  PMAC Asyn Coordinate System Axes based on
 *  the asynMotorController class.
 *
 *  For instructions see class file
 *  pmacCSController.cpp
 *
 *  Alan Greer
 *  23 February 2016
 *
 ********************************************/

#ifndef pmacCSController_H
#define pmacCSController_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "pmacCSAxis.h"
#include "pmacAxis.h"
#include "pmacCallbackInterface.h"
#include "pmacDebugger.h"
#include "pmacHardwareInterface.h"

// asyn parameter strings
#define PMAC_CS_FirstParamString           "PMAC_CS_FIRSTPARAM"
#define PMAC_CS_LastParamString            "PMAC_CS_LASTPARAM"
#define PMAC_CS_CsMoveTimeString           "PMAC_C_CS_MOVE_TIME"
#define PMAC_CS_CsAbortString              "PMAC_C_ABORT"
// the following 2 parameters are axis parameters for both pmacController and pmacCSController
#define PMAC_CS_RealMotorNumberString      "PMAC_REAL_MOTOR_NUMBER"
#define PMAC_CS_MotorScaleString           "PMAC_MOTOR_SCALE"
#define PMAC_CS_MotorResString             "PMAC_MRES"
#define PMAC_CS_MotorOffsetString          "PMAC_OFFSET"
#define PMAC_CS_ForwardKinematicString     "PMAC_CS_FWD_KIN"
#define PMAC_CS_InverseKinematicString     "PMAC_CS_INV_KIN"
#define PMAC_CS_QVariablesString           "PMAC_CS_Q_VARIABLES"

// direct moves
#define PMAC_CS_DirectMoveString           "PMAC_C_DIRECT_MOVE"
// The following 2 parameters are the direct mapped resolution and offset
// These will be provided by the motor record if the axis is used for a kinematic
// or else they will reflect a directly mapped axis resolution and offset.
// Readback only parameters
#define PMAC_CS_DirectResString             "PMAC_DIRECT_MRES"
#define PMAC_CS_DirectOffsetString          "PMAC_DIRECT_OFFSET"

#define PMAC_CS_MAXBUF 1024
#define PMAC_CS_AXES_COUNT 9

class pmacCSController
        : public asynMotorController, public pmacCallbackInterface, public pmacDebugger {

public:
    pmacCSController(const char *portName, const char *controllerPortName, int csNo, int program);
    virtual ~pmacCSController();
    void initComplete();
    void badConnection();
    bool initialised(void);
    std::string getPortName();
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    void setDebugLevel(int level, int axis);
    bool getMoving();
    int getCSNumber();
    double getAxisResolution(int axis);
    double getAxisOffset(int axis);
    int getProgramNumber();
    csStatus getStatus();
    std::string getVelocityCmd(double velocity, double steps);
    std::string getCSAccTimeCmd(double time);
    void callback(pmacCommandStore *sPtr, int type);
    asynStatus immediateWriteRead(const char *command, char *response);
    asynStatus axisWriteRead(const char *command, char *response);
    pmacCSAxis *getAxis(asynUser *pasynUser);
    pmacCSAxis *getAxis(int axisNo);
    pmacAxis *getRawAxis(int axisNo);

    // Registration for callbacks
    asynStatus registerForCallbacks(pmacCallbackInterface *cbPtr, int type);

    // Add PMAC variable/status item to monitor
    asynStatus monitorPMACVariable(int poll_speed, const char *var);
    asynStatus tScanCheckForErrors();
    std::string tScanGetErrorMessage();
    asynStatus tScanCheckProgramRunning(int *running);

    // Ensure CS demands (Q71..9) are consistent after a motor move or CS change
    asynStatus makeCSDemandsConsistent();
    asynStatus pmacSetAxisScale(int axis, int scale);
    asynStatus wakeupPoller();
    asynStatus pmacCSSetAxisDirectMapping(int axis, int mappedAxis);
    int pmacCSGetAxisDirectMapping(int axis);

    // Read in the kinematics
    asynStatus storeKinematics();
    asynStatus listKinematic(int csNo, const std::string &type, char *buffer, size_t size);

    asynStatus updateCsDemands();

protected:
    pmacCSAxis **pAxes_; // Array of pointers to axis objects

    int PMAC_CS_FirstParam_;
#define FIRST_PMAC_CS_PARAM PMAC_CS_FirstParam_
    int PMAC_CS_CsMoveTime_;
    int PMAC_CS_RealMotorNumber_;
    int PMAC_CS_MotorScale_;
    int PMAC_CS_MotorRes_;
    int PMAC_CS_MotorOffset_;
    int PMAC_CS_Abort_;
    int PMAC_CS_LastParam_;
    int PMAC_CS_ForwardKinematic_;
    int PMAC_CS_InverseKinematic_;
    int PMAC_CS_QVariables_;
    int PMAC_CS_DirectMove_;
    int PMAC_CS_DirectRes_;
    int PMAC_CS_DirectOffset_;
#define LAST_PMAC_CS_PARAM PMAC_CS_LastParam_

private:
    std::string portName_;
    int csNumber_;
    int progNumber_;
    epicsUInt32 movesDeferred_;
    int status_[3];
    csStatus cStatus_;
    pmacController *pC_;
    // csMoveTime_ defines how long a programmed move takes. The value of this parameter is always
    // put into Q70 before a call to PROG10 on the brick. A value of 0 means that the motion
    // will execute as fast as possible and the slowest motor will determine the time.
    // A value of -1 (default) means that the default CS move time is used this is the legacy
    // behaviour in which the VELO parameters of the CS motors are used
    double csMoveTime_;

    asynStatus processDeferredMoves(void);

    static const epicsUInt32 PMAC_ERROR_PRINT_TIME_;

    static const epicsUInt32 CS_STATUS1_RUNNING_PROG;        // (0x1<<0)
    static const epicsUInt32 CS_STATUS1_SINGLE_STEP_MODE;    // (0x1<<1)
    static const epicsUInt32 CS_STATUS1_CONTINUOUS_MODE;     // (0x1<<2)
    static const epicsUInt32 CS_STATUS1_MOVE_BY_TIME_MODE;   // (0x1<<3)
    static const epicsUInt32 CS_STATUS1_CONTINUOUS_REQUEST;  // (0x1<<4)
    static const epicsUInt32 CS_STATUS1_RADIUS_INC_MODE;     // (0x1<<5)
    static const epicsUInt32 CS_STATUS1_A_INC;               // (0x1<<6)
    static const epicsUInt32 CS_STATUS1_A_FEEDRATE;          // (0x1<<7)
    static const epicsUInt32 CS_STATUS1_B_INC;               // (0x1<<8)
    static const epicsUInt32 CS_STATUS1_B_FEEDRATE;          // (0x1<<9)
    static const epicsUInt32 CS_STATUS1_C_INC;               // (0x1<<10)
    static const epicsUInt32 CS_STATUS1_C_FEEDRATE;          // (0x1<<11)
    static const epicsUInt32 CS_STATUS1_U_INC;               // (0x1<<12)
    static const epicsUInt32 CS_STATUS1_U_FEEDRATE;          // (0x1<<13)
    static const epicsUInt32 CS_STATUS1_V_INC;               // (0x1<<14)
    static const epicsUInt32 CS_STATUS1_V_FEEDRATE;          // (0x1<<15)
    static const epicsUInt32 CS_STATUS1_W_INC;               // (0x1<<16)
    static const epicsUInt32 CS_STATUS1_W_FEEDRATE;          // (0x1<<17)
    static const epicsUInt32 CS_STATUS1_X_INC;               // (0x1<<18)
    static const epicsUInt32 CS_STATUS1_X_FEEDRATE;          // (0x1<<19)
    static const epicsUInt32 CS_STATUS1_Y_INC;               // (0x1<<20)
    static const epicsUInt32 CS_STATUS1_Y_FEEDRATE;          // (0x1<<21)
    static const epicsUInt32 CS_STATUS1_Z_INC;               // (0x1<<22)
    static const epicsUInt32 CS_STATUS1_Z_FEEDRATE;          // (0x1<<23)

    static const epicsUInt32 CS_STATUS2_CIRCLE_SPLINE_MODE;  // (0x1<<0)
    static const epicsUInt32 CS_STATUS2_CCW_RAPID_MODE;      // (0x1<<1)
    static const epicsUInt32 CS_STATUS2_2D_CUTTER_COMP;      // (0x1<<2)
    static const epicsUInt32 CS_STATUS2_2D_LEFT_3D_CUTTER;   // (0x1<<3)
    static const epicsUInt32 CS_STATUS2_PVT_SPLINE_MODE;     // (0x1<<4)
    static const epicsUInt32 CS_STATUS2_SEG_STOPPING;        // (0x1<<5)
    static const epicsUInt32 CS_STATUS2_SEG_ACCEL;           // (0x1<<6)
    static const epicsUInt32 CS_STATUS2_SEG_MOVING;          // (0x1<<7)
    static const epicsUInt32 CS_STATUS2_PRE_JOG;             // (0x1<<8)
    static const epicsUInt32 CS_STATUS2_CUTTER_MOVE_BUFFD;   // (0x1<<9)
    static const epicsUInt32 CS_STATUS2_CUTTER_STOP;         // (0x1<<10)
    static const epicsUInt32 CS_STATUS2_CUTTER_COMP_OUTSIDE; // (0x1<<11)
    static const epicsUInt32 CS_STATUS2_DWELL_MOVE_BUFFD;    // (0x1<<12)
    static const epicsUInt32 CS_STATUS2_SYNCH_M_ONESHOT;     // (0x1<<13)
    static const epicsUInt32 CS_STATUS2_EOB_STOP;            // (0x1<<14)
    static const epicsUInt32 CS_STATUS2_DELAYED_CALC;        // (0x1<<15)
    static const epicsUInt32 CS_STATUS2_ROTARY_BUFF;         // (0x1<<16)
    static const epicsUInt32 CS_STATUS2_IN_POSITION;         // (0x1<<17)
    static const epicsUInt32 CS_STATUS2_FOLLOW_WARN;         // (0x1<<18)
    static const epicsUInt32 CS_STATUS2_FOLLOW_ERR;          // (0x1<<19)
    static const epicsUInt32 CS_STATUS2_AMP_FAULT;           // (0x1<<20)
    static const epicsUInt32 CS_STATUS2_MOVE_IN_STACK;       // (0x1<<21)
    static const epicsUInt32 CS_STATUS2_RUNTIME_ERR;         // (0x1<<22)
    static const epicsUInt32 CS_STATUS2_LOOKAHEAD;           // (0x1<<23)

    static const epicsUInt32 CS_STATUS3_LIMIT;               // (0x1<<1)

    static const std::string CS_RUNTIME_ERRORS[];
    friend class pmacCSAxis;

    friend class pmacCsGroups;
};

#define NUM_PMAC_CS_PARAMS (&LAST_PMAC_CS_PARAM - &FIRST_PMAC_CS_PARAM + 1)

#endif // pmacCSController_H


