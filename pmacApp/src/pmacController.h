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

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "pmacAxis.h"
#include "pmacCsGroups.h"
#include "pmacMessageBroker.h"
#include "IntegerHashtable.h"

#define PMAC_C_FirstParamString        "PMAC_C_FIRSTPARAM"
#define PMAC_C_LastParamString         "PMAC_C_LASTPARAM"

#define PMAC_C_GlobalStatusString      "PMAC_C_GLOBALSTATUS"
#define PMAC_C_CommsErrorString        "PMAC_C_COMMSERROR"

#define PMAC_C_FeedRateString          "PMAC_C_FEEDRATE"
#define PMAC_C_FeedRateLimitString     "PMAC_C_FEEDRATE_LIMIT"
#define PMAC_C_FeedRatePollString      "PMAC_C_FEEDRATE_POLL"
#define PMAC_C_FeedRateProblemString   "PMAC_C_FEEDRATE_PROBLEM"
#define PMAC_C_CoordSysGroup  		     "PMAC_C_COORDINATE_SYS_GROUP"

#define PMAC_C_FastUpdateTimeString    "PMAC_C_FAST_UPDATE_TIME"

#define PMAC_C_AxisCSString            "PMAC_C_AXIS_CS"
#define PMAC_C_WriteCmdString          "PMAC_C_WRITE_CMD"
#define PMAC_C_KillAxisString          "PMAC_C_KILL_AXIS"
#define PMAC_C_PLCBits00String         "PMAC_C_PLC_BITS00"
#define PMAC_C_PLCBits01String         "PMAC_C_PLC_BITS01"
#define PMAC_C_StatusBits01String      "PMAC_C_STATUS_BITS01"
#define PMAC_C_StatusBits02String      "PMAC_C_STATUS_BITS02"
#define PMAC_C_StatusBits03String      "PMAC_C_STATUS_BITS03"
#define PMAC_C_GpioInputsString        "PMAC_C_GPIO_INPUTS"
#define PMAC_C_GpioOutputsString       "PMAC_C_GPIO_OUTPUTS"
#define PMAC_C_ProgBitsString          "PMAC_C_PROG_BITS"
#define PMAC_C_AxisBits01String        "PMAC_C_AXIS_BITS01"
#define PMAC_C_AxisBits02String        "PMAC_C_AXIS_BITS02"
#define PMAC_C_AxisBits03String        "PMAC_C_AXIS_BITS03"

#define PMAC_C_NoOfMsgsString          "PMAC_C_NO_OF_MSGS"
#define PMAC_C_TotalBytesWrittenString "PMAC_C_TBYTES_WRITE"
#define PMAC_C_TotalBytesReadString    "PMAC_C_TBYTES_READ"
#define PMAC_C_MsgBytesWrittenString   "PMAC_C_MBYTES_WRITE"
#define PMAC_C_MsgBytesReadString      "PMAC_C_MBYTES_READ"
#define PMAC_C_MsgTimeString           "PMAC_C_MSG_TIME"
#define PMAC_C_MaxBytesWrittenString   "PMAC_C_MAX_BYTES_WRITE"
#define PMAC_C_MaxBytesReadString      "PMAC_C_MAX_BYTES_READ"
#define PMAC_C_MaxTimeString           "PMAC_C_MAX_TIME"
#define PMAC_C_AveBytesWrittenString   "PMAC_C_AVE_BYTES_WRITE"
#define PMAC_C_AveBytesReadString      "PMAC_C_AVE_BYTES_READ"
#define PMAC_C_AveTimeString           "PMAC_C_AVE_TIME"

#define PMAC_C_TrajBufferLengthString  "PMAC_C_TRAJ_LENGTH"  // Length of a single buffer e.g. AX, AY
#define PMAC_C_TrajTotalPointsString   "PMAC_C_TRAJ_POINTS"  // Total number of points scanned through
#define PMAC_C_TrajStatusString        "PMAC_C_TRAJ_STATUS"  // Current status reported by the PMAC
#define PMAC_C_TrajCurrentIndexString  "PMAC_C_TRAJ_INDEX"   // Current index position in buffers
#define PMAC_C_TrajCurrentBufferString "PMAC_C_TRAJ_CBUFF"   // Current buffer specifier - 0: A, 1: B
#define PMAC_C_TrajBuffAdrAString      "PMAC_C_TRAJ_ADRA"    // Start index of buffer A
#define PMAC_C_TrajBuffAdrBString      "PMAC_C_TRAJ_ADRB"    // Start index of buffer B
#define PMAC_C_TrajBuffFillAString     "PMAC_C_TRAJ_FILLA"   // Fill level of buffer A
#define PMAC_C_TrajBuffFillBString     "PMAC_C_TRAJ_FILLB"   // Fill level of buffer B
#define PMAC_C_TrajRunTimeString       "PMAC_C_TRAJ_TIME"    // Current run time of scan (s)
#define PMAC_C_TrajCSNumberString      "PMAC_C_TRAJ_CS"      // Current CS scan is executing on
#define PMAC_C_TrajPercentString       "PMAC_C_TRAJ_PERCENT" // Percentage of scan complete
#define PMAC_C_TrajEStatusString       "PMAC_C_TRAJ_ESTATUS" // Our report of tScan status

#define PMAC_MAXBUF 1024

#define PMAC_MAX_PARAMETERS 1000

#define PMAC_MAX_CS 16
#define PMAC_MAX_CS_AXES 9

#define PMAC_MAX_TRAJECTORY_POINTS 10000000

#define PMAC_POINTS_PER_WRITE 18

#define PMAC_MEDIUM_LOOP_TIME 2000
#define PMAC_SLOW_LOOP_TIME   5000

#define DEFERRED_FAST_MOVES 1
#define DEFERRED_COORDINATED_MOVES 2

#define PMAC_TRAJ_STATUS         "P4001" // Status of motion program for EPICS - 0: Idle, 1: Running, 2: Finished, 3: Error
#define PMAC_TRAJ_ABORT          "P4002" // Abort trigger for EPICS
#define PMAC_TRAJ_AXES           "P4003" // An int between 1 and 511 specifying which axes to use
#define PMAC_TRAJ_BUFFER_LENGTH  "P4004" // Length of a single buffer e.g. AX, AY
#define PMAC_TRAJ_TOTAL_POINTS   "P4005" // Total number of points scanned through
#define PMAC_TRAJ_CURRENT_INDEX  "P4006" // Current index position in buffers
#define PMAC_TRAJ_CURRENT_BUFFER "P4007" // Current buffer specifier - 0: A, 1: B
#define PMAC_TRAJ_BUFF_ADR_A     "P4008" // Start index of buffer A
#define PMAC_TRAJ_BUFF_ADR_B     "P4009" // Start index of buffer B
#define PMAC_TRAJ_CURR_ADR       "P4010" // A or B buffer address
#define PMAC_TRAJ_BUFF_FILL_A    "P4011" // Fill level of buffer A
#define PMAC_TRAJ_BUFF_FILL_B    "P4012" // Fill level of buffer B
#define PMAC_TRAJ_CURR_FILL      "P4013" // The indexes that current buffer has been filled up to

#define PMAC_TRAJ_BUFFER_A 0
#define PMAC_TRAJ_BUFFER_B 1

#define PMAC_TRAJ_STATUS_RUNNING 1
#define PMAC_TRAJ_STATUS_FINISHED 2

class pmacCSMonitor;
class pmacCSController;

class pmacController : public asynMotorController, public pmacCallbackInterface, public pmacDebugger
{

 public:
  pmacController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, int numAxes, double movingPollPeriod, 
		 double idlePollPeriod);

  virtual ~pmacController();

  void startPMACPolling();

  void setDebugLevel(int level, int axis);
  asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
  virtual void callback(pmacCommandStore *sPtr, int type);
  asynStatus slowUpdate(pmacCommandStore *sPtr);
  asynStatus mediumUpdate(pmacCommandStore *sPtr);

  //asynStatus printConnectedStatus(void);
  asynStatus immediateWriteRead(const char *command, char *response);

  /* These are the methods that we override */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual);

  void report(FILE *fp, int level);
  pmacAxis* getAxis(asynUser *pasynUser);
  pmacAxis* getAxis(int axisNo);
  asynStatus poll();

  // Trajectory scanning methods
  asynStatus initializeProfile(size_t maxPoints);
  asynStatus buildProfile(int csNo);
  asynStatus executeProfile(int csNo);
  asynStatus abortProfile();
  void trajectoryTask();
  asynStatus sendTrajectoryDemands(int buffer);
  asynStatus doubleToPMACFloat(double value, int64_t *representation);

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
  asynStatus registerCS(pmacCSController *csPtr, int csNo);

  // Read out the device type (cid)
  asynStatus readDeviceType();

  // List PLC program
  asynStatus listPLCProgram(int plcNo, char *buffer, size_t size);

  asynStatus storeKinematics();
  asynStatus listKinematic(int csNo, const std::string& type, char *buffer, size_t size);

 protected:
  pmacAxis **pAxes_;       /**< Array of pointers to axis objects */

  int PMAC_C_FirstParam_;
  #define FIRST_PMAC_PARAM PMAC_C_FirstParam_
  int PMAC_C_GlobalStatus_;
  int PMAC_C_CommsError_;
  int PMAC_C_FeedRate_;
  int PMAC_C_FeedRateLimit_;
  int PMAC_C_FeedRatePoll_;
  int PMAC_C_FeedRateProblem_;
  int PMAC_C_CoordSysGroup_;
  int PMAC_C_FastUpdateTime_;
  int PMAC_C_AxisCS_;
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
  int PMAC_C_TrajPercent_;
  int PMAC_C_TrajEStatus_;
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
  int PMAC_C_ForwardKinematic_[PMAC_MAX_CS];
  int PMAC_C_InverseKinematic_[PMAC_MAX_CS];
  int PMAC_C_LastParam_;
  #define LAST_PMAC_PARAM PMAC_C_LastParam_
  int parameters[PMAC_MAX_PARAMETERS];

 public:
  pmacCsGroups *pGroupList;

 private:
  int cid_;
  int parameterIndex_;
  pmacMessageBroker *pBroker_;
  IntegerHashtable *pIntParams_;
  IntegerHashtable *pHexParams_;
  IntegerHashtable *pDoubleParams_;
  IntegerHashtable *pStringParams_;
  StringHashtable *pWriteParams_;
  pmacCSMonitor *pAxisZero;
  pmacCSController **pCSControllers_;
  asynUser* lowLevelPortUser_;
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

  // Trajectory scan variables
  bool profileInitialized_;
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
  double **tScanPositions_;       // 2D array of profile positions (1 array for each axis)
  int *profileUser_;              // Array of profile user values
  int *profileVelMode_;           // Array of profile velocity modes
  epicsEventId startEventId_;
  epicsEventId stopEventId_;

  asynStatus lowLevelWriteRead(const char *command, char *response);
//  asynStatus lowLevelPortConnect(const char *port, int addr, asynUser **ppasynUser, char *inputEos, char *outputEos);

  asynStatus newGetGlobalStatus(pmacCommandStore *sPtr);
//  asynStatus getGlobalStatus(epicsUInt32 *globalStatus, int *feedrate, int feedrate_poll);

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

 /*Global status ???*/
  static const epicsUInt32 PMAC_GSTATUS_CARD_ADDR;             
  static const epicsUInt32 PMAC_GSTATUS_ALL_CARD_ADDR;         
  static const epicsUInt32 PMAC_GSTATUS_RESERVED;              
  static const epicsUInt32 PMAC_GSTATUS_PHASE_CLK_MISS;        
  static const epicsUInt32 PMAC_GSTATUS_MACRO_RING_ERRORCHECK; 
  static const epicsUInt32 PMAC_GSTATUS_MACRO_RING_COMMS;      
  static const epicsUInt32 PMAC_GSTATUS_TWS_PARITY_ERROR;      
  static const epicsUInt32 PMAC_GSTATUS_CONFIG_ERROR;          
  static const epicsUInt32 PMAC_GSTATUS_ILLEGAL_LVAR;          
  static const epicsUInt32 PMAC_GSTATUS_REALTIME_INTR;         
  static const epicsUInt32 PMAC_GSTATUS_FLASH_ERROR;           
  static const epicsUInt32 PMAC_GSTATUS_DPRAM_ERROR;           
  static const epicsUInt32 PMAC_GSTATUS_CKSUM_ACTIVE;          
  static const epicsUInt32 PMAC_GSTATUS_CKSUM_ERROR;           
  static const epicsUInt32 PMAC_GSTATUS_LEADSCREW_COMP;        
  static const epicsUInt32 PMAC_GSTATUS_WATCHDOG;              
  static const epicsUInt32 PMAC_GSTATUS_SERVO_REQ;             
  static const epicsUInt32 PMAC_GSTATUS_DATA_GATHER_START;     
  static const epicsUInt32 PMAC_GSTATUS_RESERVED2;             
  static const epicsUInt32 PMAC_GSTATUS_DATA_GATHER_ON;        
  static const epicsUInt32 PMAC_GSTATUS_SERVO_ERROR;           
  static const epicsUInt32 PMAC_GSTATUS_CPUTYPE;               
  static const epicsUInt32 PMAC_GSTATUS_REALTIME_INTR_RE;      
  static const epicsUInt32 PMAC_GSTATUS_RESERVED3;             

  static const epicsUInt32 PMAC_HARDWARE_PROB;
  static const epicsUInt32 PMAX_AXIS_GENERAL_PROB1;
  static const epicsUInt32 PMAX_AXIS_GENERAL_PROB2;

  static const char *PMAC_C_ForwardKinematicString[];
  static const char *PMAC_C_InverseKinematicString[];

  friend class pmacAxis;
  friend class pmacCsGroups;
};

#define NUM_PMAC_PARAMS (&LAST_PMAC_PARAM - &FIRST_PMAC_PARAM + 1 + PMAC_MAX_PARAMETERS)

#endif /* pmacController_H */
