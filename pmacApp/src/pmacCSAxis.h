/*
 * pmacCSAxis.h
 *
 *  Created on: 29 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_PMACCSAXIS_H_
#define PMACAPP_SRC_PMACCSAXIS_H_

#include "shareLib.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "pmacCallbackInterface.h"
#include "pmacCommandStore.h"
#include "pmacDebugger.h"

class pmacCSController;

class pmacCSAxis : public asynMotorAxis, pmacCallbackInterface, public pmacDebugger {
public:
    pmacCSAxis(pmacCSController *pController, int axisNo);

    virtual ~pmacCSAxis();

    void badConnection();
    void goodConnection();

    void setKinematicResolution(double new_resolution);
    double getResolution();

    void setKinematicOffset(double new_offset);
    double getOffset();

    asynStatus directMove(double position, double min_velocity, double max_velocity, double acceleration);

    asynStatus move(double position, int relative, double min_velocity, double max_velocity,
                    double acceleration);

    asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);

    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);

    asynStatus stop(double acceleration);

    bool getMoving();

    double getCurrentPosition();

    virtual void callback(pmacCommandStore *sPtr, int type);

private:
    pmacCSController *pC_;

    asynStatus getAxisStatus(pmacCommandStore *sPtr);

    int deferredMove_;
    int motorPosChanged_;
    char deferredCommand_[128];
    int scale_;
    double kinematic_resolution_;
    double kinematic_offset_;
    double position_;
    double previous_position_;
    int previous_direction_;
    epicsTimeStamp nowTime_;
    epicsFloat64 nowTimeSecs_;
    epicsFloat64 lastTimeSecs_;
    bool printNextError_;
    bool moving_; // only valid within poll time - used as a hint for validating deferred coordinated moves
    bool connected_;
    bool initialized_;

    friend class pmacCSController;

    friend class pmacCsGroups;

};

#endif /* PMACAPP_SRC_PMACCSAXIS_H_ */
