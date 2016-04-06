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
#include "pmacCallbackInterface.h"
#include "pmacDebugger.h"

#define PMAC_C_ProfileUserString       "PMAC_PROFILE_USER"    // User buffer for trajectory scan
#define PMAC_C_ProfileVelModeString    "PMAC_PROFILE_VELMODE" // Velocity mode buffer for trajectory scan

class pmacCSController : public asynMotorController, public pmacCallbackInterface, public pmacDebugger
{

  public:
    pmacCSController(const char *portName, const char *controllerPortName, int csNo, int program);
    virtual ~pmacCSController();
    asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
    asynStatus writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements);
    bool getMoving();
    int getCSNumber();
    int getProgramNumber();

    void callback(pmacCommandStore *sPtr, int type);

    asynStatus immediateWriteRead(const char *command, char *response);

    pmacCSAxis *getAxis(int axisNo);

    // Registration for callbacks
    asynStatus registerForCallbacks(pmacCallbackInterface *cbPtr, int type);

    // Add PMAC variable/status item to monitor
    asynStatus monitorPMACVariable(int poll_speed, const char *var);

    virtual asynStatus initializeProfile(size_t maxProfilePoints);
    virtual asynStatus buildProfile();
    virtual asynStatus executeProfile();
    asynStatus tScanBuildTimeArray(double *profileTimes, int *numPoints, int maxPoints);
    asynStatus tScanIncludedAxes(int *axisMask);
    asynStatus tScanBuildProfileArray(double *positions, int axis, int numPoints);
    asynStatus tScanBuildUserArray(int *userArray, int *numPoints, int maxPoints);
    asynStatus tScanBuildVelModeArray(int *velModeArray, int *numPoints, int maxPoints);
    asynStatus tScanCheckForErrors();
    asynStatus tScanCheckProgramRunning(int *running);

  protected:
    pmacCSAxis **pAxes_; // Array of pointers to axis objects

    int PMAC_CS_FirstParam_;
    #define FIRST_PMAC_CS_PARAM PMAC_CS_FirstParam_
    int PMAC_C_ProfileUser_;
    int PMAC_C_ProfileVelMode_;
    int PMAC_CS_LastParam_;
    #define LAST_PMAC_CS_PARAM PMAC_CS_LastParam_

  private:
    int csNumber_;
    int progNumber_;
    int status_[3];
    void *pC_;
    bool profileInitialized_;
    int *profileUser_;
    int *profileVelMode_;

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

    friend class pmacCSAxis;
    friend class pmacCsGroups;
};

#define NUM_PMAC_CS_PARAMS (&LAST_PMAC_CS_PARAM - &FIRST_PMAC_CS_PARAM + 1)

#endif // pmacCSController_H


