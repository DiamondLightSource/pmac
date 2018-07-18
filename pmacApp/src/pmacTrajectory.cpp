/*
 * pmacTrajectory.cpp
 *
 *  Created on: 26 Oct 2016
 *      Author: gnx91527
 */

#include "pmacTrajectory.h"

pmacTrajectory::pmacTrajectory() : pmacDebugger("pmacTrajectory") {
  noOfAxes_ = 9;
  totalNoOfPoints_ = 0;
  noOfValidPoints_ = 0;
  profilePositions_ = NULL;
  profileTimes_ = NULL;
  profileUser_ = NULL;
  profileVelMode_ = NULL;
  static const char *functionName = "pmacTrajectory";

  debug(DEBUG_FLOW, functionName);
}

pmacTrajectory::~pmacTrajectory() {
  int axis = 0;
  if (profilePositions_) {
    for (axis = 0; axis < noOfAxes_; axis++) {
      if (profilePositions_[axis]) {
        free(profilePositions_[axis]);
      }
    }
    free(profilePositions_);
  }
  if (profileTimes_) {
    free(profileTimes_);
  }
  if (profileUser_) {
    free(profileUser_);
  }
  if (profileVelMode_) {
    free(profileVelMode_);
  }
}

asynStatus pmacTrajectory::initialise(int noOfPoints) {
  asynStatus status = asynSuccess;
  int axis = 0;
  static const char *functionName = "initialise";

  debug(DEBUG_TRACE, functionName, "Called with noOfPoints", noOfPoints);

  // Initialise the position array
  if (profilePositions_) {
    for (axis = 0; axis < noOfAxes_; axis++) {
      if (profilePositions_[axis]) {
        free(profilePositions_[axis]);
      }
    }
    free(profilePositions_);
  }
  profilePositions_ = (double **) malloc(sizeof(double *) * noOfAxes_);
  if (profilePositions_ == NULL) {
    debug(DEBUG_ERROR, functionName, "Unable to allocate memory for the trajectory scan positions");
    status = asynError;
  }
  if (status == asynSuccess) {
    for (axis = 0; axis < noOfAxes_; axis++) {
      profilePositions_[axis] = (double *) malloc(sizeof(double) * noOfPoints);
      if (profilePositions_[axis] == NULL) {
        debug(DEBUG_ERROR, functionName,
              "Unable to allocate memory for the trajectory scan positions");
        status = asynError;
      }
    }
  }

  // Initialise the time array
  if (status == asynSuccess) {
    if (profileTimes_) {
      free(profileTimes_);
    }
    profileTimes_ = (int *) malloc(sizeof(int) * noOfPoints);
    if (profileTimes_ == NULL) {
      debug(DEBUG_ERROR, functionName, "Unable to allocate memory for the trajectory scan times");
      status = asynError;
    }
  }

  // Initialise the user mode array
  if (status == asynSuccess) {
    if (profileUser_) {
      free(profileUser_);
    }
    profileUser_ = (int *) malloc(sizeof(int) * noOfPoints);
    if (profileUser_ == NULL) {
      debug(DEBUG_ERROR, functionName,
            "Unable to allocate memory for the trajectory scan user mode array");
      status = asynError;
    }
  }

  // Initialise the velocity mode array
  if (status == asynSuccess) {
    if (profileVelMode_) {
      free(profileVelMode_);
    }
    profileVelMode_ = (int *) malloc(sizeof(int) * noOfPoints);
    if (profileVelMode_ == NULL) {
      debug(DEBUG_ERROR, functionName,
            "Unable to allocate memory for the trajectory scan velocity mode array");
      status = asynError;
    }
  }

  // If all allocation was successful then set the number of points in this scan
  if (status == asynSuccess) {
    totalNoOfPoints_ = noOfPoints;
    noOfValidPoints_ = 0;
  }

  return status;
}

asynStatus pmacTrajectory::append(double **positions, double *times, int *user, int *velocity,
                                  int noOfPoints) {
  asynStatus status = asynSuccess;
  int axis = 0;
  int counter = 0;
  static const char *functionName = "append";

  debug(DEBUG_TRACE, functionName, "Called with noOfPoints", noOfPoints);

  // First check that we aren't being asked to append more points that we have room for
  if ((noOfValidPoints_ + noOfPoints) > totalNoOfPoints_) {
    debug(DEBUG_ERROR, functionName, "Not enough storage to append all of these points");
    status = asynError;
  }

  // Check that the supplied times are not out of range (> 24 bit)
  if (status == asynSuccess) {
    counter = 0;
    // Check for any invalid times
    while (counter < noOfPoints && status == asynSuccess) {
      // Profile times must be less than 24bit
      if (((int) times[counter]) > 0xFFFFFF) {
        debug(DEBUG_ERROR, functionName, "Invalid profile time value (> 24 bit)", times[counter]);
        status = asynError;
      }
      counter++;
    }
  }

  // Check that the supplied user mode values are not out of range (> 4 bit)
  if (status == asynSuccess) {
    counter = 0;
    // Check for any invalid user mode values
    while (counter < noOfPoints && status == asynSuccess) {
      // User mode values must be less than 4bit
      if (user[counter] > 0xF) {
        debug(DEBUG_ERROR, functionName, "Invalid user mode value (> 4 bit)", user[counter]);
        status = asynError;
      }
      counter++;
    }
  }

  // Check that the supplied velocity mode values are not out of range (> 4 bit)
  if (status == asynSuccess) {
    counter = 0;
    // Check for any invalid velocity mode values
    while (counter < noOfPoints && status == asynSuccess) {
      // Velocity mode values must be less than 4bit
      if (velocity[counter] > 0xF) {
        debug(DEBUG_ERROR, functionName, "Invalid velocity mode value (> 4 bit)",
              velocity[counter]);
        status = asynError;
      }
      counter++;
    }
  }

  // Memory copy the positions into the correct locations
  if (status == asynSuccess) {
    for (axis = 0; axis < noOfAxes_; axis++) {
      double *pPtr = &profilePositions_[axis][noOfValidPoints_];
      memcpy(pPtr, positions[axis], (noOfPoints * sizeof(double)));
    }
  }

  // Copy the times into the correct locations
  if (status == asynSuccess) {
    int *tPtr = &profileTimes_[noOfValidPoints_];
    for (counter = 0; counter < noOfPoints; counter++) {
      *tPtr = (int) (times[counter]);
      tPtr++;
    }
  }

  // Memory copy the user modes into the correct locations
  if (status == asynSuccess) {
    int *uPtr = &profileUser_[noOfValidPoints_];
    memcpy(uPtr, user, (noOfPoints * sizeof(int)));
  }

  // Memory copy the velocity modes into the correct locations
  if (status == asynSuccess) {
    int *vPtr = &profileVelMode_[noOfValidPoints_];
    memcpy(vPtr, velocity, (noOfPoints * sizeof(int)));
  }

  // Set the number of valid points
  if (status == asynSuccess) {
    noOfValidPoints_ += noOfPoints;
  }

  return status;
}

int pmacTrajectory::getNoOfAxes() {
  static const char *functionName = "getNoOfAxes";
  debug(DEBUG_TRACE, functionName, "noOfAxes_", noOfAxes_);
  return noOfAxes_;
}

int pmacTrajectory::getTotalNoOfPoints() {
  static const char *functionName = "getTotalNoOfPoints";
  debug(DEBUG_TRACE, functionName, "totalNoOfPoints_", totalNoOfPoints_);
  return totalNoOfPoints_;
}

int pmacTrajectory::getNoOfValidPoints() {
  static const char *functionName = "getNoOfValidPoints";
  debug(DEBUG_TRACE, functionName, "noOfValidPoints_", noOfValidPoints_);
  return noOfValidPoints_;
}

asynStatus pmacTrajectory::getTime(int index, int *time) {
  asynStatus status = asynSuccess;
  static const char *functionName = "readTime";

  debug(DEBUG_TRACE, functionName, "Called with index", index);

  // Check the index is valid
  if (index < 0 || index >= noOfValidPoints_) {
    debug(DEBUG_ERROR, functionName, "Invalid index requested", index);
    status = asynError;
  }

  if (status == asynSuccess) {
    *time = profileTimes_[index];
  }

  return status;
}

asynStatus pmacTrajectory::getUserMode(int index, int *user) {
  asynStatus status = asynSuccess;
  static const char *functionName = "readUserMode";

  debug(DEBUG_TRACE, functionName, "Called with index", index);

  // Check the index is valid
  if (index < 0 || index >= noOfValidPoints_) {
    debug(DEBUG_ERROR, functionName, "Invalid index requested", index);
    status = asynError;
  }

  if (status == asynSuccess) {
    *user = profileUser_[index];
  }

  return status;
}

asynStatus pmacTrajectory::getVelocityMode(int index, int *velocity) {
  asynStatus status = asynSuccess;
  static const char *functionName = "readVelocityMode";

  debug(DEBUG_TRACE, functionName, "Called with index", index);

  // Check the index is valid
  if (index < 0 || index >= noOfValidPoints_) {
    debug(DEBUG_ERROR, functionName, "Invalid index requested", index);
    status = asynError;
  }

  if (status == asynSuccess) {
    *velocity = profileVelMode_[index];
  }

  return status;
}

asynStatus pmacTrajectory::getPosition(int axis, int index, double *position) {
  asynStatus status = asynSuccess;
  static const char *functionName = "readPosition";

  debug(DEBUG_TRACE, functionName, "Called with axis", axis);
  debug(DEBUG_TRACE, functionName, "Called with index", index);

  // Check the axis is valid
  if (axis < 0 || axis >= noOfAxes_) {
    debug(DEBUG_ERROR, functionName, "Invalid axis requested", axis);
    status = asynError;
  }

  // Check the index is valid
  if (index < 0 || index >= noOfValidPoints_) {
    debug(DEBUG_ERROR, functionName, "Invalid index requested", index);
    status = asynError;
  }

  if (status == asynSuccess) {
    *position = profilePositions_[axis][index];
  }

  return status;
}

void pmacTrajectory::report() {
  static const char *functionName = "report";
  debug(DEBUG_ERROR, functionName, "totalNoOfPoints_", totalNoOfPoints_);
  debug(DEBUG_ERROR, functionName, "noOfValidPoints_", noOfValidPoints_);
  for (int index = 0; index < noOfValidPoints_; index++) {
    debug(DEBUG_ERROR, functionName, "INDEX", index);
    debug(DEBUG_ERROR, functionName, "Time", profileTimes_[index]);
    debug(DEBUG_ERROR, functionName, "User", profileUser_[index]);
    debug(DEBUG_ERROR, functionName, "Velocity", profileVelMode_[index]);
    debug(DEBUG_ERROR, functionName, "Axis[0]", profilePositions_[0][index]);
    debug(DEBUG_ERROR, functionName, "Axis[1]", profilePositions_[1][index]);
  }
}
