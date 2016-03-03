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

#define PMAC_C_FirstParamString "PMAC_C_FIRSTPARAM"
#define PMAC_C_LastParamString "PMAC_C_LASTPARAM"

#define PMAC_C_GlobalStatusString "PMAC_C_GLOBALSTATUS"
#define PMAC_C_CommsErrorString "PMAC_C_COMMSERROR"

#define PMAC_C_FeedRateString         "PMAC_C_FEEDRATE"
#define PMAC_C_FeedRateLimitString    "PMAC_C_FEEDRATE_LIMIT"
#define PMAC_C_FeedRatePollString     "PMAC_C_FEEDRATE_POLL"
#define PMAC_C_FeedRateProblemString  "PMAC_C_FEEDRATE_PROBLEM"
#define PMAC_C_CoordSysGroup  		  "PMAC_C_COORDINATE_SYS_GROUP"

#define PMAC_C_FastUpdateTimeString   "PMAC_C_FAST_UPDATE_TIME"

#define PMAC_C_AxisCSString           "PMAC_C_AXIS_CS"

#define PMAC_MAXBUF 1024

#define PMAC_MAX_PARAMETERS 1000

#define PMAC_MEDIUM_LOOP_TIME 2000
#define PMAC_SLOW_LOOP_TIME   5000

#define DEFERRED_FAST_MOVES 1
#define DEFERRED_COORDINATED_MOVES 2

class pmacCSMonitor;
class pmacCSController;

class pmacController : public asynMotorController, public pmacCallbackInterface, public pmacDebugger
{

 public:
  pmacController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, int numAxes, double movingPollPeriod, 
		 double idlePollPeriod);

  virtual ~pmacController();

  void setDebugLevel(int level, int axis);
  asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
  virtual void callback(pmacCommandStore *sPtr, int type);

  //asynStatus printConnectedStatus(void);
  asynStatus immediateWriteRead(const char *command, char *response);

  /* These are the methods that we override */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  void report(FILE *fp, int level);
  pmacAxis* getAxis(asynUser *pasynUser);
  pmacAxis* getAxis(int axisNo);
  asynStatus poll();


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
  int PMAC_C_LastParam_;
  #define LAST_PMAC_PARAM PMAC_C_LastParam_
  int parameters[PMAC_MAX_PARAMETERS];

 public:
  pmacCsGroups *pGroupList;

 private:
  int parameterIndex_;
  pmacMessageBroker *pBroker_;
  IntegerHashtable *pIntParams_;
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

  asynStatus lowLevelWriteRead(const char *command, char *response);
//  asynStatus lowLevelPortConnect(const char *port, int addr, asynUser **ppasynUser, char *inputEos, char *outputEos);

  asynStatus newGetGlobalStatus(pmacCommandStore *sPtr);
//  asynStatus getGlobalStatus(epicsUInt32 *globalStatus, int *feedrate, int feedrate_poll);

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

  friend class pmacAxis;
  friend class pmacCsGroups;
};

#define NUM_PMAC_PARAMS (&LAST_PMAC_PARAM - &FIRST_PMAC_PARAM + 1 + PMAC_MAX_PARAMETERS)

#endif /* pmacController_H */
