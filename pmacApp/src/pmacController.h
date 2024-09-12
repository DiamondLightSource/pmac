/********************************************
 *  pmacController.h
 *
 *  PMAC Asyn motor based on the
 *  asynMotorController class.
 *
 *  Matthew Pearson
 *  23 May 2012
 *
 ********************************************/

#ifndef pmacController_H
#define pmacController_H

#include "shareLib.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "pmacAxis.h"
#include "pmacCsGroups.h"
#include "pmacMessageBroker.h"
#include "pmacTrajectory.h"
#include "pmacHardwareTurbo.h"
#include "pmacHardwarePower.h"
#include "IntegerHashtable.h"

#define PMAC_C_FirstParamString           "PMAC_C_FIRSTPARAM"
#define PMAC_C_LastParamString            "PMAC_C_LASTPARAM"

#define PMAC_C_PollAllNowString           "PMAC_C_POLLALLNOW"
#define PMAC_C_StopAllString              "PMAC_C_STOPALL"
#define PMAC_C_KillAllString              "PMAC_C_KILLALL"
#define PMAC_C_GlobalStatusString         "PMAC_C_GLOBALSTATUS"
#define PMAC_C_CommsErrorString           "PMAC_C_COMMSERROR"

#define PMAC_C_FeedRateString             "PMAC_C_FEEDRATE"
#define PMAC_C_FeedRateLimitString        "PMAC_C_FEEDRATE_LIMIT"
#define PMAC_C_FeedRatePollString         "PMAC_C_FEEDRATE_POLL"
#define PMAC_C_FeedRateProblemString      "PMAC_C_FEEDRATE_PROBLEM"
#define PMAC_C_FeedRateCSString           "PMAC_C_FEEDRATE_CS"
#define PMAC_C_CoordSysGroup              "PMAC_C_COORDINATE_SYS_GROUP"

#define PMAC_C_GroupCSPortString          "PMAC_C_GROUP_CS_PORT"
#define PMAC_C_GroupCSPortRBVString       "PMAC_C_GROUP_CS_PORT_RBV"
#define PMAC_C_GroupAssignString          "PMAC_C_GROUP_ASSIGN"
#define PMAC_C_GroupAssignRBVString       "PMAC_C_GROUP_ASSIGN_RBV"
#define PMAC_C_GroupExecuteString         "PMAC_C_GROUP_EXECUTE"

#define PMAC_C_DebugLevelString           "PMAC_C_DEBUG_LEVEL"
#define PMAC_C_DebugAxisString            "PMAC_C_DEBUG_AXIS"
#define PMAC_C_DebugCSString              "PMAC_C_DEBUG_CS"
#define PMAC_C_DebugCmdString             "PMAC_C_DEBUG_CMD"
#define PMAC_C_DisablePollingString       "PMAC_C_DEBUG_POLL_OFF"

#define PMAC_C_FastUpdateTimeString       "PMAC_C_FAST_UPDATE_TIME"

#define PMAC_C_CpuNumCoresString          "PMAC_C_CPU_NUM_CORES"
#define PMAC_C_CpuUsage0String            "PMAC_C_CPU_USAGE_0"
#define PMAC_C_CpuUsage1String            "PMAC_C_CPU_USAGE_1"
#define PMAC_C_CpuUsage2String            "PMAC_C_CPU_USAGE_2"
#define PMAC_C_CpuUsage3String            "PMAC_C_CPU_USAGE_3"
#define PMAC_C_AxisCSString               "PMAC_C_AXIS_CS"
#define PMAC_C_AxisReadonlyString         "PMAC_C_AXIS_READONLY"
#define PMAC_C_WriteCmdString             "PMAC_C_WRITE_CMD"
#define PMAC_C_KillAxisString             "PMAC_C_KILL_AXIS"
#define PMAC_C_PLCBits00String            "PMAC_C_PLC_BITS00"
#define PMAC_C_PLCBits01String            "PMAC_C_PLC_BITS01"
#define PMAC_C_StatusBits01String         "PMAC_C_STATUS_BITS01"
#define PMAC_C_StatusBits02String         "PMAC_C_STATUS_BITS02"
#define PMAC_C_StatusBits03String         "PMAC_C_STATUS_BITS03"
#define PMAC_C_GpioInputsString           "PMAC_C_GPIO_INPUTS"
#define PMAC_C_GpioOutputsString          "PMAC_C_GPIO_OUTPUTS"
#define PMAC_C_ProgBitsString             "PMAC_C_PROG_BITS"
#define PMAC_C_AxisBits01String           "PMAC_C_AXIS_BITS01"
#define PMAC_C_AxisBits02String           "PMAC_C_AXIS_BITS02"
#define PMAC_C_AxisBits03String           "PMAC_C_AXIS_BITS03"

#define PMAC_C_NoOfMsgsString             "PMAC_C_NO_OF_MSGS"
#define PMAC_C_TotalBytesWrittenString    "PMAC_C_TBYTES_WRITE"
#define PMAC_C_TotalBytesReadString       "PMAC_C_TBYTES_READ"
#define PMAC_C_MsgBytesWrittenString      "PMAC_C_MBYTES_WRITE"
#define PMAC_C_MsgBytesReadString         "PMAC_C_MBYTES_READ"
#define PMAC_C_MsgTimeString              "PMAC_C_MSG_TIME"
#define PMAC_C_MaxBytesWrittenString      "PMAC_C_MAX_BYTES_WRITE"
#define PMAC_C_MaxBytesReadString         "PMAC_C_MAX_BYTES_READ"
#define PMAC_C_MaxTimeString              "PMAC_C_MAX_TIME"
#define PMAC_C_AveBytesWrittenString      "PMAC_C_AVE_BYTES_WRITE"
#define PMAC_C_AveBytesReadString         "PMAC_C_AVE_BYTES_READ"
#define PMAC_C_AveTimeString              "PMAC_C_AVE_TIME"

#define PMAC_C_FastStoreString            "PMAC_C_FAST_STORE"
#define PMAC_C_MediumStoreString          "PMAC_C_MEDIUM_STORE"
#define PMAC_C_SlowStoreString            "PMAC_C_SLOW_STORE"
#define PMAC_C_ReportFastString           "PMAC_C_REPORT_FAST"
#define PMAC_C_ReportMediumString         "PMAC_C_REPORT_MEDIUM"
#define PMAC_C_ReportSlowString           "PMAC_C_REPORT_SLOW"
#define PMAC_C_HomingStatusString         "HOMING_STATUS"
// the following 4 parameters are axis parameters for both pmacController and pmacCSController
#define PMAC_C_RealMotorNumberString      "PMAC_REAL_MOTOR_NUMBER"
#define PMAC_C_MotorScaleString           "PMAC_MOTOR_SCALE"
#define PMAC_C_MotorResString             "PMAC_MRES"
#define PMAC_C_MotorOffsetString          "PMAC_OFFSET"
#define PMAC_C_IVariablesString           "PMAC_I_VARIABLES"
#define PMAC_C_MVariablesString           "PMAC_M_VARIABLES"
#define PMAC_C_PVariablesString           "PMAC_P_VARIABLES"

// direct moves
#define PMAC_C_DirectMoveString           "PMAC_C_DIRECT_MOVE"
#define PMAC_C_DirectResString            "PMAC_DIRECT_MRES"
#define PMAC_C_DirectOffsetString         "PMAC_DIRECT_OFFSET"


#define PMAC_C_ProfileUseAxisAString      "PROFILE_USE_AXIS_A"
#define PMAC_C_ProfileUseAxisBString      "PROFILE_USE_AXIS_B"
#define PMAC_C_ProfileUseAxisCString      "PROFILE_USE_AXIS_C"
#define PMAC_C_ProfileUseAxisUString      "PROFILE_USE_AXIS_U"
#define PMAC_C_ProfileUseAxisVString      "PROFILE_USE_AXIS_V"
#define PMAC_C_ProfileUseAxisWString      "PROFILE_USE_AXIS_W"
#define PMAC_C_ProfileUseAxisXString      "PROFILE_USE_AXIS_X"
#define PMAC_C_ProfileUseAxisYString      "PROFILE_USE_AXIS_Y"
#define PMAC_C_ProfileUseAxisZString      "PROFILE_USE_AXIS_Z"
#define PMAC_C_ProfilePositionsAString    "PROFILE_POSITIONS_A"
#define PMAC_C_ProfilePositionsBString    "PROFILE_POSITIONS_B"
#define PMAC_C_ProfilePositionsCString    "PROFILE_POSITIONS_C"
#define PMAC_C_ProfilePositionsUString    "PROFILE_POSITIONS_U"
#define PMAC_C_ProfilePositionsVString    "PROFILE_POSITIONS_V"
#define PMAC_C_ProfilePositionsWString    "PROFILE_POSITIONS_W"
#define PMAC_C_ProfilePositionsXString    "PROFILE_POSITIONS_X"
#define PMAC_C_ProfilePositionsYString    "PROFILE_POSITIONS_Y"
#define PMAC_C_ProfilePositionsZString    "PROFILE_POSITIONS_Z"
#define PMAC_C_ProfileVelocitiesAString   "PROFILE_VELOCITIES_A"
#define PMAC_C_ProfileVelocitiesBString   "PROFILE_VELOCITIES_B"
#define PMAC_C_ProfileVelocitiesCString   "PROFILE_VELOCITIES_C"
#define PMAC_C_ProfileVelocitiesUString   "PROFILE_VELOCITIES_U"
#define PMAC_C_ProfileVelocitiesVString   "PROFILE_VELOCITIES_V"
#define PMAC_C_ProfileVelocitiesWString   "PROFILE_VELOCITIES_W"
#define PMAC_C_ProfileVelocitiesXString   "PROFILE_VELOCITIES_X"
#define PMAC_C_ProfileVelocitiesYString   "PROFILE_VELOCITIES_Y"
#define PMAC_C_ProfileVelocitiesZString   "PROFILE_VELOCITIES_Z"
#define PMAC_C_CompTable0_WString         "PMAC_C_COMP_0"
#define PMAC_C_CompTable1_WString         "PMAC_C_COMP_1"
#define PMAC_C_CompTable2_WString         "PMAC_C_COMP_2"
#define PMAC_C_CompTable3_WString         "PMAC_C_COMP_3"
#define PMAC_C_CompTable4_WString         "PMAC_C_COMP_4"
#define PMAC_C_CompTable5_WString         "PMAC_C_COMP_5"
#define PMAC_C_CompTable6_WString         "PMAC_C_COMP_6"
#define PMAC_C_CompTable7_WString         "PMAC_C_COMP_7"
#define PMAC_C_ProfileAppendString        "PROFILE_APPEND"
#define PMAC_C_ProfileAppendStateString   "PROFILE_APPEND_STATE"
#define PMAC_C_ProfileAppendStatusString  "PROFILE_APPEND_STATUS"
#define PMAC_C_ProfileAppendMessageString "PROFILE_APPEND_MESSAGE"
#define PMAC_C_ProfileNumBuildString      "PROFILE_NUM_BUILD"
#define PMAC_C_ProfileBuiltPointsString   "PROFILE_POINTS_BUILT"

#define PMAC_C_ProfileUserString          "PMAC_PROFILE_USER"    // User buffer for trajectory scan
#define PMAC_C_ProfileVelModeString       "PMAC_PROFILE_VELMODE" // Velocity mode buffer for trajectory scan

#define PMAC_C_TrajBufferLengthString     "PMAC_C_TRAJ_LENGTH"  // Length of a single buffer e.g. AX, AY
#define PMAC_C_TrajTotalPointsString      "PMAC_C_TRAJ_POINTS"  // Total number of points scanned through
#define PMAC_C_TrajStatusString           "PMAC_C_TRAJ_STATUS"  // Current status reported by the PMAC
#define PMAC_C_TrajCurrentIndexString     "PMAC_C_TRAJ_INDEX"   // Current index position in buffers
#define PMAC_C_TrajCurrentBufferString    "PMAC_C_TRAJ_CBUFF"   // Current buffer specifier - 0: A, 1: B
#define PMAC_C_TrajBuffAdrAString         "PMAC_C_TRAJ_ADRA"    // Start index of buffer A
#define PMAC_C_TrajBuffAdrBString         "PMAC_C_TRAJ_ADRB"    // Start index of buffer B
#define PMAC_C_TrajBuffFillAString        "PMAC_C_TRAJ_FILLA"   // Fill level of buffer A
#define PMAC_C_TrajBuffFillBString        "PMAC_C_TRAJ_FILLB"   // Fill level of buffer B
#define PMAC_C_TrajRunTimeString          "PMAC_C_TRAJ_TIME"    // Current run time of scan (s)
#define PMAC_C_TrajCSNumberString         "PMAC_C_TRAJ_CS"      // Current CS scan is executing on
#define PMAC_C_TrajCSPortString           "PMAC_C_TRAJ_CS_PORT" // Desired CS port to execute
#define PMAC_C_TrajPercentString          "PMAC_C_TRAJ_PERCENT" // Percentage of scan complete
#define PMAC_C_TrajEStatusString          "PMAC_C_TRAJ_ESTATUS" // Our report of tScan status
#define PMAC_C_TrajProgString             "PMAC_C_TRAJ_PROG"    // Which motion program to execute
#define PMAC_C_TrajCalcVelString          "PMAC_TRAJ_CALCVEL"   // Velocity array calculation - 0: No calculation - receives velocities directly; 1: Calculate - calculate the velocities base on time, position and velocity mode
#define PMAC_C_TrajProgVersionString      "PMAC_C_TRAJ_PROG_V"  // Motion program version number
#define PMAC_C_TrajCodeVersionString      "PMAC_C_TRAJ_CODE_V"  // Version of this control code

#define PMAC_TRAJECTORY_VERSION 4

#define PMAC_CPU_GEO_240MHZ               "DSP56321"            // Approved geobrick for trajectory scans
#define PMAC_CPU_CLIPPER                  "DSP56303"            // Allowed for trajectory scans

#define PMAC_MAXBUF 1024

#define PMAC_MAX_PARAMETERS 1000

#define PMAC_MAX_CS 16
#define PMAC_MAX_CS_AXES 9

#define PMAC_MAX_TRAJECTORY_POINTS 10000000

#define PMAC_POINTS_PER_WRITE 17

#define PMAC_MEDIUM_LOOP_TIME 2000
#define PMAC_SLOW_LOOP_TIME   5000

#define PMAC_PVT_TIME_MODE       "I42"   // PVT Time Control Mode (0=4,095 ms max time, 1=8,388,607 ms max time)

#define PMAC_CPU_PHASE_INTR      "M70"   // Time between phase interrupts (CPU cycles/2)
#define PMAC_CPU_PHASE_TIME      "M71"   // Time for phase tasks (CPU cycles/2)
#define PMAC_CPU_SERVO_TIME      "M72"   // Time for servo tasks (CPU cycles/2)
#define PMAC_CPU_RTI_TIME        "M73"   // Time for RTI tasks (CPU cycles/2)
#define PMAC_CPU_I8              "I8"    // RTI period (Servo cycles - 1)
#define PMAC_CPU_I7002           "I7002" // Servo clock divider (ServoClockFreq = PhaseClockFreq/(ServoClockDiv + 1) )

#define PPMAC_CPU_FPHASE_TIME    "Sys.FltrPhaseTime"
#define PPMAC_CPU_FSERVO_TIME    "Sys.FltrServoTime"
#define PPMAC_CPU_FRTI_TIME      "Sys.FltrRtIntTime"
#define PPMAC_CPU_FBG_TIME       "Sys.FltrBgTime"
#define PPMAC_CPU_PHASED_TIME    "Sys.PhaseDeltaTime"
#define PPMAC_CPU_SERVOD_TIME    "Sys.ServoDeltaTime"
#define PPMAC_CPU_RTID_TIME      "Sys.RtIntDeltaTime"
#define PPMAC_CPU_BGD_TIME       "Sys.BgDeltaTime"

#define PPMAC_CPU_MAXCORES      4 // Maximum number of cores currently supported
#define PPMAC_CPU_TASKS_NUM     4 // Number of tasks supported by the PowerPMAC core management
#define PPMAC_CPU_PHASETASK     0 // Highest priority task
#define PPMAC_CPU_SERVOTASK     1 // ...
#define PPMAC_CPU_RTTASK        2 // ...
#define PPMAC_CPU_BGTASK        3 // Lowest priority task

#define PMAC_TRAJ_STATUS         "M4034" // Status of motion program for EPICS - 0: Idle, 1: Running, 2: Finished, 3: Error
#define PMAC_TRAJ_ABORT          "M4035" // Abort trigger for EPICS
#define PMAC_TRAJ_AXES           "M4036" // An int between 1 and 511 specifying which axes to use
#define PMAC_TRAJ_BUFFER_LENGTH  "M4037" // Length of a single buffer e.g. AX, AY
#define PMAC_TRAJ_TOTAL_POINTS   "M4038" // Total number of points scanned through
#define PMAC_TRAJ_CURRENT_INDEX  "M4039" // Current index position in buffers
#define PMAC_TRAJ_CURRENT_BUFFER "M4040" // Current buffer specifier - 0: A, 1: B
#define PMAC_TRAJ_BUFF_ADR_A     "M4041" // Start index of buffer A
#define PMAC_TRAJ_BUFF_ADR_B     "M4042" // Start index of buffer B
#define PMAC_TRAJ_CURR_ADR       "M4043" // A or B buffer address
#define PMAC_TRAJ_BUFF_FILL_A    "M4044" // Fill level of buffer A
#define PMAC_TRAJ_BUFF_FILL_B    "M4045" // Fill level of buffer B
#define PMAC_TRAJ_CURR_FILL      "M4046" // The indexes that current buffer has been filled up to
#define PMAC_TRAJ_PROG_VERSION   "M4049" // Trajectory program version

#define PMAC_TRAJ_BUFFER_A 0
#define PMAC_TRAJ_BUFFER_B 1

#define PMAC_TRAJ_STATUS_RUNNING 1
#define PMAC_TRAJ_STATUS_FINISHED 2

#define PMAC_TRAJ_VELOCITY_PROVIDED 0
#define PMAC_TRAJ_VELOCITY_CALCULATED 1

class pmacCSMonitor;

class pmacCSController;

class pmacController
        : public asynMotorController, public pmacCallbackInterface, public pmacDebugger {

public:
    pmacController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress,
                   int numAxes, double movingPollPeriod,
                   double idlePollPeriod);
    virtual ~pmacController();
    asynStatus checkConnection();
    asynStatus initialSetup();
    bool initialised(void);
    void createAsynParams(void);
    void initAsynParams(void);
    void pollAllNow(void);
    void setupBrokerVariables(void);
    void startPMACPolling();
    void setDebugLevel(int level, int axis, int csNo);

    asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
    asynStatus processDrvInfo(char *input, char *output);
    virtual void callback(pmacCommandStore *sPtr, int type);
    asynStatus slowUpdate(pmacCommandStore *sPtr);
    asynStatus mediumUpdate(pmacCommandStore *sPtr);
    asynStatus prefastUpdate(pmacCommandStore *sPtr);
    asynStatus fastUpdate(pmacCommandStore *sPtr);
    asynStatus parseIntegerVariable(const std::string &command,
                                    const std::string &response,
                                    const std::string &desc,
                                    int &value);
    asynStatus parseDoubleVariable(const std::string &command,
                                    const std::string &response,
                                    const std::string &desc,
                                    double &value);

    //asynStatus printConnectedStatus(void);
    asynStatus immediateWriteRead(const char *command, char *response);
    asynStatus axisWriteRead(const char *command, char *response);

    /* These are the methods that we override */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
    asynStatus writeInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements);
    asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual);

    asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], size_t nElements,
             size_t *nIn);

    void report(FILE *fp, int level);
    pmacAxis *getAxis(asynUser *pasynUser);
    pmacAxis *getAxis(int axisNo);
    asynStatus poll();

    // Trajectory scanning methods
    asynStatus initializeProfile(size_t maxPoints);
    asynStatus handleBufferRollover(int *numPointsToBuild);
    asynStatus buildProfile();
    asynStatus buildProfile(int csNo);
    asynStatus appendToProfile();
    asynStatus preparePMAC();
    asynStatus executeProfile();
    asynStatus executeProfile(int csNo);
    asynStatus abortProfile();

    void trajectoryTask();
    void setBuildStatus(int state, int status, const std::string &message);
    void setAppendStatus(int state, int status, const std::string &message);
    void setProfileStatus(int state, int status, const std::string &message);
    asynStatus sendTrajectoryDemands(int buffer);

    //Disable the check for disabled hardware limits.
    asynStatus pmacDisableLimitsCheck(int axis);
    asynStatus pmacDisableLimitsCheck(void);

    //Set the axis scale factor.
    asynStatus pmacSetAxisScale(int axis, int scale);

    //Set the open loop encoder axis
    asynStatus pmacSetOpenLoopEncoderAxis(int axis, int encoder_axis);

    // Registration for callbacks
    asynStatus registerForCallbacks(pmacCallbackInterface *cbPtr, int type);

    // Add PMAC variable/status item to monitor
    asynStatus monitorPMACVariable(int poll_speed, const char *var);

    // Register a coordinate system with this controller
    asynStatus registerCS(pmacCSController *csPtr, const char *portName, int csNo);

    // Initialise the hardware and connection for a coordinate system
    asynStatus initCSHardware(int csNo);

    // Ensure CS demands (Q71..9) are consistent after a motor move or CS change
    asynStatus makeCSDemandsConsistent();

    // Read out the device type (cid)
    asynStatus readDeviceType();

    // Set number of CPU cores based on CPU type
    asynStatus getCpuNumCores();

    // Get CPU core for each task
    asynStatus getTasksCore();

    // List PLC program
    asynStatus listPLCProgram(int plcNo, char *buffer, size_t size);
    asynStatus executeManualGroup();
    asynStatus updateCsAssignmentParameters();
    asynStatus copyCsReadbackToDemand(bool manual);
    asynStatus tScanBuildProfileArray(double *positions, double *velocities, double *times, int axis, int numPoints);
    asynStatus tScanGetPreviousPoint(double *previousPos, double *previousVel, const double *positions, const double *velocities, int numPoints, int index, int csNum, int axis);
    asynStatus tScanCalculateVelocityArray(double *positions, double *velocities, double *times, int numPoints, int index, int csNum, int axis);
    asynStatus tScanIncludedAxes(int *axisMask);
    void registerForLock(asynPortDriver *controller);

protected:
    pmacAxis **pAxes_;       /**< Array of pointers to axis objects */

    int PMAC_C_FirstParam_;
#define FIRST_PMAC_PARAM PMAC_C_FirstParam_
    int PMAC_C_StopAll_;
    int PMAC_C_KillAll_;
    int PMAC_C_PollAllNow_;
    int PMAC_C_GlobalStatus_;
    int PMAC_C_CommsError_;
    int PMAC_C_FeedRate_;
    int PMAC_C_FeedRateLimit_;
    int PMAC_C_FeedRatePoll_;
    int PMAC_C_FeedRateProblem_;
    int PMAC_C_FeedRateCS_;
    int PMAC_C_CoordSysGroup_;
    int PMAC_C_GroupCSPort_;
    int PMAC_C_GroupCSPortRBV_;
    int PMAC_C_GroupAssign_;
    int PMAC_C_GroupAssignRBV_;
    int PMAC_C_GroupExecute_;
    int PMAC_C_DebugLevel_;
    int PMAC_C_DebugAxis_;
    int PMAC_C_DebugCS_;
    int PMAC_C_DebugCmd_;
    int PMAC_C_DisablePolling_;
    int PMAC_C_FastUpdateTime_;
    int PMAC_C_CpuNumCores_;
    int PMAC_C_CpuUsage0_;
    int PMAC_C_CpuUsage1_;
    int PMAC_C_CpuUsage2_;
    int PMAC_C_CpuUsage3_;
    int PMAC_C_AxisCS_;
    int PMAC_C_AxisReadonly_;
    int PMAC_C_WriteCmd_;
    int PMAC_C_KillAxis_;
    int PMAC_C_PLCBits00_;
    int PMAC_C_PLCBits01_;
    int PMAC_C_StatusBits01_;
    int PMAC_C_StatusBits02_;
    int PMAC_C_StatusBits03_;
    int PMAC_C_GpioInputs_;
    int PMAC_C_GpioOutputs_;
    int PMAC_C_ProgBits_;
    int PMAC_C_AxisBits01_;
    int PMAC_C_AxisBits02_;
    int PMAC_C_AxisBits03_;
    int PMAC_C_ProfileUseAxisA_;
    int PMAC_C_ProfileUseAxisB_;
    int PMAC_C_ProfileUseAxisC_;
    int PMAC_C_ProfileUseAxisU_;
    int PMAC_C_ProfileUseAxisV_;
    int PMAC_C_ProfileUseAxisW_;
    int PMAC_C_ProfileUseAxisX_;
    int PMAC_C_ProfileUseAxisY_;
    int PMAC_C_ProfileUseAxisZ_;
    int PMAC_C_ProfilePositionsA_;
    int PMAC_C_ProfilePositionsB_;
    int PMAC_C_ProfilePositionsC_;
    int PMAC_C_ProfilePositionsU_;
    int PMAC_C_ProfilePositionsV_;
    int PMAC_C_ProfilePositionsW_;
    int PMAC_C_ProfilePositionsX_;
    int PMAC_C_ProfilePositionsY_;
    int PMAC_C_ProfilePositionsZ_;
    int PMAC_C_ProfileVelocitiesA_;
    int PMAC_C_ProfileVelocitiesB_;
    int PMAC_C_ProfileVelocitiesC_;
    int PMAC_C_ProfileVelocitiesU_;
    int PMAC_C_ProfileVelocitiesV_;
    int PMAC_C_ProfileVelocitiesW_;
    int PMAC_C_ProfileVelocitiesX_;
    int PMAC_C_ProfileVelocitiesY_;
    int PMAC_C_ProfileVelocitiesZ_;
    int PMAC_C_CompTable0_W;
    int PMAC_C_CompTable1_W;
    int PMAC_C_CompTable2_W;
    int PMAC_C_CompTable3_W;
    int PMAC_C_CompTable4_W;
    int PMAC_C_CompTable5_W;
    int PMAC_C_CompTable6_W;
    int PMAC_C_CompTable7_W;
    int PMAC_C_ProfileAppend_;
    int PMAC_C_ProfileAppendState_;
    int PMAC_C_ProfileAppendStatus_;
    int PMAC_C_ProfileAppendMessage_;
    int PMAC_C_ProfileNumBuild_;
    int PMAC_C_ProfileBuiltPoints_;
    int PMAC_C_ProfileUser_;
    int PMAC_C_ProfileVelMode_;
    int PMAC_C_TrajBufferLength_;
    int PMAC_C_TrajTotalPoints_;
    int PMAC_C_TrajStatus_;
    int PMAC_C_TrajCurrentIndex_;
    int PMAC_C_TrajCurrentBuffer_;
    int PMAC_C_TrajBuffAdrA_;
    int PMAC_C_TrajBuffAdrB_;
    int PMAC_C_TrajBuffFillA_;
    int PMAC_C_TrajBuffFillB_;
    int PMAC_C_TrajRunTime_;
    int PMAC_C_TrajCSNumber_;
    int PMAC_C_TrajCSPort_;
    int PMAC_C_TrajPercent_;
    int PMAC_C_TrajEStatus_;
    int PMAC_C_TrajProg_;
    int PMAC_C_TrajCalcVel_;
    int PMAC_C_TrajProgVersion_;
    int PMAC_C_TrajCodeVersion_;
    int PMAC_C_NoOfMsgs_;
    int PMAC_C_TotalBytesWritten_;
    int PMAC_C_TotalBytesRead_;
    int PMAC_C_MsgBytesWritten_;
    int PMAC_C_MsgBytesRead_;
    int PMAC_C_MsgTime_;
    int PMAC_C_MaxBytesWritten_;
    int PMAC_C_MaxBytesRead_;
    int PMAC_C_MaxTime_;
    int PMAC_C_AveBytesWritten_;
    int PMAC_C_AveBytesRead_;
    int PMAC_C_AveTime_;
    int PMAC_C_FastStore_;
    int PMAC_C_MediumStore_;
    int PMAC_C_SlowStore_;
    int PMAC_C_ReportFast_;
    int PMAC_C_ReportMedium_;
    int PMAC_C_ReportSlow_;
    int PMAC_C_HomingStatus_;
    int PMAC_C_RealMotorNumber_;
    int PMAC_C_MotorScale_;
    int PMAC_C_MotorRes_;
    int PMAC_C_MotorOffset_;
    int PMAC_C_DirectMove_;
    int PMAC_C_DirectRes_;
    int PMAC_C_DirectOffset_;
    int PMAC_I_Variables_;
    int PMAC_M_Variables_;
    int PMAC_P_Variables_;
    int PMAC_C_LastParam_;
#define LAST_PMAC_PARAM PMAC_C_LastParam_
    int parameters[PMAC_MAX_PARAMETERS];

public:
    pmacCsGroups *pGroupList;
    bool useCsVelocity;
    pmacHardwareInterface *pHardware_;

    void addBrokerVariables(const std::string &monitorVariables);

private:
    int connected_;
    int initialised_;
    int cid_;
    std::string cpu_;
    int cpuNumCores_;
    int cpuCoreTasks_[PPMAC_CPU_TASKS_NUM];
    int parameterIndex_;
    pmacMessageBroker *pBroker_;
    pmacTrajectory *pTrajectory_;
    IntegerHashtable *pPortToCs_;
    IntegerHashtable *pIntParams_;
    IntegerHashtable *pHexParams_;
    IntegerHashtable *pDoubleParams_;
    IntegerHashtable *pStringParams_;
    StringHashtable *pWriteParams_;
    pmacCSMonitor *pAxisZero;
    pmacCSController **pCSControllers_;
    asynUser *lowLevelPortUser_;
    epicsUInt32 movesDeferred_;
    epicsTimeStamp nowTime_;
    epicsFloat64 nowTimeSecs_;
    epicsFloat64 lastTimeSecs_;
    epicsTimeStamp lastMediumTime_;
    epicsTimeStamp lastSlowTime_;
    bool printNextError_;
    bool feedRatePoll_;
    double movingPollPeriod_;
    double idlePollPeriod_;
    int i8_;
    int i7002_;
    bool csResetAllDemands;
    int csCount;


    // Trajectory scan variables
    int pvtTimeMode_;
    bool profileInitialized_;
    bool profileBuilt_;
    bool appendAvailable_;
    bool tScanShortScan_;           // Is the scan a short scan (< 3.0 seconds)
    int tScanExecuting_;            // Is a scan executing
    int tScanCSNo_;                 // The CS number of the executing scan
    int tScanNumPoints_;            // Total number of points in the scan
    int tScanAxisMask_;             // Mask describing which axes are used in the scan
    int tScanPointCtr_;             // Counter of scan points written
    int tScanPmacBufferPtr_;
    int tScanPmacTotalPts_;
    int tScanPmacStatus_;
    int tScanPmacBufferNumber_;     // Which half buffer (A=0,B=1) is the PMAC reading
    int tScanPmacBufferAddressA_;
    int tScanPmacBufferAddressB_;
    int tScanPmacBufferSize_;
    double tScanPmacProgVersion_;
    double **eguProfilePositions_;  // 2D array of profile positions in EGU (1 array for each axis)
    double **tScanPositions_;       // 2D array of profile positions (1 array for each axis)
    double **eguProfileVelocities_; // 2D array of profile velocities in EGU (1 array for each axis)
    double **tScanVelocities_;      // 2D array of profile velocities (1 array for each axis)
    int *profileUser_;              // Array of profile user values
    int *profileVelMode_;           // Array of profile velocity modes
    enum ProfileVelocityMode {      // Profile Velocity Modes - used in the velocity calculation
        AVERAGE_PREVIOUS_NEXT,      // 0
        REAL_PREVIOUS_CURRENT,      // 1
        AVERAGE_PREVIOUS_CURRENT,   // 2
        ZERO_VELOCITY,              // 3
        AVERAGE_CURRENT_NEXT        // 4
    };
    // Used to handle the buffer rollover when the velocities
    // are calculated from profileVelMode_
    int tScanPendingPoint_;
    int tScanPendingPointReady_;
    double **tScanPrevBufferPositions;  // 2D array for storing the last 2 positions of the buffer (1 array for each axis)
    double *tScanPrevBufferVelocity;    // Array for storing the last position of the buffer of each axis
    double tScanPrevBufferTime;         // Stores the last time of the buffer

    epicsEventId startEventId_;
    epicsEventId stopEventId_;

    asynStatus lowLevelWriteRead(const char *command, char *response);

    asynStatus updateStatistics();

    asynStatus processDeferredMoves(void);

    //static class data members

    static const epicsUInt32 PMAC_MAXBUF_;
    static const epicsFloat64 PMAC_TIMEOUT_;
    static const epicsUInt32 PMAC_FEEDRATE_LIM_;
    static const epicsUInt32 PMAC_FEEDRATE_DEADBAND_;
    static const epicsUInt32 PMAC_ERROR_PRINT_TIME_;
    static const epicsUInt32 PMAC_FORCED_FAST_POLLS_;
    static const epicsUInt32 PMAC_OK_;
    static const epicsUInt32 PMAC_ERROR_;
    static const epicsInt32 PMAC_CID_PMAC_;
    static const epicsInt32 PMAC_CID_GEOBRICK_;
    static const epicsInt32 PMAC_CID_CLIPPER_;
    static const epicsInt32 PMAC_CID_POWER_;

    static const epicsUInt32 PMAC_STATUS1_MAXRAPID_SPEED;
    static const epicsUInt32 PMAC_STATUS1_ALT_CMNDOUT_MODE;
    static const epicsUInt32 PMAC_STATUS1_SOFT_POS_CAPTURE;
    static const epicsUInt32 PMAC_STATUS1_ERROR_TRIGGER;
    static const epicsUInt32 PMAC_STATUS1_FOLLOW_ENABLE;
    static const epicsUInt32 PMAC_STATUS1_FOLLOW_OFFSET;
    static const epicsUInt32 PMAC_STATUS1_PHASED_MOTOR;
    static const epicsUInt32 PMAC_STATUS1_ALT_SRC_DEST;
    static const epicsUInt32 PMAC_STATUS1_USER_SERVO;
    static const epicsUInt32 PMAC_STATUS1_USER_PHASE;
    static const epicsUInt32 PMAC_STATUS1_HOMING;
    static const epicsUInt32 PMAC_STATUS1_BLOCK_REQUEST;
    static const epicsUInt32 PMAC_STATUS1_DECEL_ABORT;
    static const epicsUInt32 PMAC_STATUS1_DESIRED_VELOCITY_ZERO;
    static const epicsUInt32 PMAC_STATUS1_DATABLKERR;
    static const epicsUInt32 PMAC_STATUS1_DWELL;
    static const epicsUInt32 PMAC_STATUS1_INTEGRATE_MODE;
    static const epicsUInt32 PMAC_STATUS1_MOVE_TIME_ON;
    static const epicsUInt32 PMAC_STATUS1_OPEN_LOOP;
    static const epicsUInt32 PMAC_STATUS1_AMP_ENABLED;
    static const epicsUInt32 PMAC_STATUS1_X_SERVO_ON;
    static const epicsUInt32 PMAC_STATUS1_POS_LIMIT_SET;
    static const epicsUInt32 PMAC_STATUS1_NEG_LIMIT_SET;
    static const epicsUInt32 PMAC_STATUS1_MOTOR_ON;

    static const epicsUInt32 PMAC_STATUS2_IN_POSITION;
    static const epicsUInt32 PMAC_STATUS2_WARN_FOLLOW_ERR;
    static const epicsUInt32 PMAC_STATUS2_ERR_FOLLOW_ERR;
    static const epicsUInt32 PMAC_STATUS2_AMP_FAULT;
    static const epicsUInt32 PMAC_STATUS2_NEG_BACKLASH;
    static const epicsUInt32 PMAC_STATUS2_I2T_AMP_FAULT;
    static const epicsUInt32 PMAC_STATUS2_I2_FOLLOW_ERR;
    static const epicsUInt32 PMAC_STATUS2_TRIGGER_MOVE;
    static const epicsUInt32 PMAC_STATUS2_PHASE_REF_ERR;
    static const epicsUInt32 PMAC_STATUS2_PHASE_SEARCH;
    static const epicsUInt32 PMAC_STATUS2_HOME_COMPLETE;
    static const epicsUInt32 PMAC_STATUS2_POS_LIMIT_STOP;
    static const epicsUInt32 PMAC_STATUS2_DESIRED_STOP;
    static const epicsUInt32 PMAC_STATUS2_FORE_IN_POS;
    static const epicsUInt32 PMAC_STATUS2_NA14;
    static const epicsUInt32 PMAC_STATUS2_ASSIGNED_CS;

    static const epicsUInt32 PMAX_AXIS_GENERAL_PROB1;
    static const epicsUInt32 PMAX_AXIS_GENERAL_PROB2;

    friend class pmacAxis;

    friend class pmacCsGroups;

    friend class pmacCSController;

    friend class pmacHardwareInterface;
};

#define NUM_PMAC_PARAMS (&LAST_PMAC_PARAM - &FIRST_PMAC_PARAM + 1 + PMAC_MAX_PARAMETERS)

#endif /* pmacController_H */
