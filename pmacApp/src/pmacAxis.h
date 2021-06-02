/********************************************
 *  pmacAxis.cpp
 * 
 *  PMAC Asyn motor based on the 
 *  asynMotorAxis class.
 * 
 *  Matthew Pearson
 *  23 May 2012
 * 
 ********************************************/

#ifndef pmacAxis_H
#define pmacAxis_H

#include "shareLib.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "pmacHardwareInterface.h"
#include "pmacCallbackInterface.h"
#include "pmacCommandStore.h"
#include "pmacDebugger.h"

class pmacController;

class pmacAxis : public asynMotorAxis, pmacCallbackInterface, public pmacDebugger {
public:
    /* These are the methods we override from the base class */
    pmacAxis(pmacController *pController, int axisNo);

    virtual ~pmacAxis();

    double getScale();

    void badConnection();

    void goodConnection();

    void setResolution(double new_resolution);

    double getResolution();

    void setOffset(double new_offset);

    double getOffset();

    void initialSetup(int axisNo);

    axisStatus getMotorStatus();

    asynStatus directMove(double position, double min_velocity, double max_velocity, double acceleration);

    asynStatus move(double position, int relative, double min_velocity, double max_velocity,
                    double acceleration);

    asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);

    asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);

    asynStatus stop(double acceleration);

    asynStatus poll(bool *moving);

    asynStatus setPosition(double position);

    asynStatus setClosedLoop(bool closedLoop);

    virtual void callback(pmacCommandStore *sPtr, int type);

    void debug(int level, const std::string &method);

    void debug(int level, const std::string &method, const std::string &message);

    void debug(int level, const std::string &method, const std::string &message,
               const std::string &value);

    void debug(int level, const std::string &method, const std::string &message, int value);

    void debug(int level, const std::string &method, const std::string &message, double value);

private:
    pmacController *pC_;

    asynStatus getAxisStatus(pmacCommandStore *sPtr);

    asynStatus getAxisInitialStatus(void);

    int getAxisCSNo();

    double getCachedPosition();

    double getPosition();

    int assignedCS_;
    double resolution_;
    double offset_;
    double setpointPosition_;
    double encoderPosition_;
    double currentVelocity_;
    double velocity_;
    double accel_;
    double highLimit_;
    double lowLimit_;
    int limitsDisabled_;
    double stepSize_;
    double deferredPosition_;
    double cachedPosition_;
    int deferredMove_;
    int deferredRelative_;
    double deferredTime_;
    int scale_;
    double rawPosition_;
    bool initiatedMove_;
    bool csRawMoveInitiated_;
    double previous_position_;
    int previous_direction_;
    int amp_enabled_;
    int amp_enabled_prev_;
    int fatal_following_;
    int encoder_axis_;
    int limitsCheckDisable_;
    epicsTimeStamp nowTime_;
    epicsFloat64 nowTimeSecs_;
    epicsFloat64 lastTimeSecs_;
    bool printNextError_;
    bool moving_; // only valid within poll time - used as a hint for validating deferred coordinated moves
    axisStatus status_;

    bool connected_; // Current connection status of the hardware
    bool initialised_; // We need to keep a record of this in case the software starts up without a connection

    friend class pmacController;

    friend class pmacCsGroups;
};


#endif /* pmacAxis_H */
