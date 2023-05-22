/*
 * pmacTrajectory.h
 *
 *  Created on: 26 Oct 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_PMACTRAJECTORY_H_
#define PMACAPP_SRC_PMACTRAJECTORY_H_

#include <asynDriver.h>
#include "pmacDebugger.h"

class pmacTrajectory : public pmacDebugger {
public:
    pmacTrajectory();

    virtual ~pmacTrajectory();

    asynStatus initialise(int noOfPoints);

    asynStatus appendVelMode(double **positions, double *times, int *user, int *velocityMode, int noOfPoints);
    
    asynStatus appendVelArray(double **positions, double **velocities, double *times, int *user, int noOfPoints);

    int getNoOfAxes();

    int getTotalNoOfPoints();

    int getNoOfValidPoints();

    int getTypeOfVelocityProfile();

    asynStatus getTime(int index, int *time);

    asynStatus getUserMode(int index, int *user);

    asynStatus getVelocityMode(int index, int *velocityMode);

    asynStatus getPosition(int axis, int index, double *position);

    asynStatus getVelocity(int axis, int index, double *velocity);

    void report();

private:
    int noOfAxes_;
    int totalNoOfPoints_;           // Total number of points in the scan
    int noOfValidPoints_;           // Number of prepared points in the scan (based on delta times)
    int typeOfVelocityProfile_;     // TODO: ADD comment [lmds]
    double **profilePositions_;     // 2D array of profile positions (1 array for each axis)
    double **profileVelocities_;    // 2D array of profile velocities (1 array for each axis)
    int *profileTimes_;             // Array of profile delta times for scan
    int *profileUser_;              // Array of profile user values
    int *profileVelMode_;           // Array of profile velocity modes
};
    
#endif /* PMACAPP_SRC_PMACTRAJECTORY_H_ */
