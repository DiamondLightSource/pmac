/*
 * pmacCSMonitor.h
 *
 *  Created on: 24 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_PMACCSMONITOR_H_
#define PMACAPP_SRC_PMACCSMONITOR_H_

#include "shareLib.h"
#include "asynMotorController.h"
#include "asynMotorAxis.h"

class pmacController;

class pmacCSController;

class pmacCSMonitor : public asynMotorAxis {
public:
    pmacCSMonitor(pmacController *pController);

    virtual ~pmacCSMonitor();

    // Register a coordinate system with this controller
    // return false if it is already registered
    bool registerCS(pmacCSController *csPtr, int csNo);

    asynStatus poll(bool *moving);

private:
    pmacCSController **pCSControllers_;
    bool moving_;

    friend class pmacController;
};

#endif /* PMACAPP_SRC_PMACCSMONITOR_H_ */
