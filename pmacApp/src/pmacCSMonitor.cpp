/*
 * pmacCSMonitor.cpp
 *
 *  Created on: 24 Feb 2016
 *      Author: gnx91527
 */

#include "pmacCSMonitor.h"
#include "pmacController.h"
#include "pmacCSController.h"
#include <stdlib.h>

pmacCSMonitor::pmacCSMonitor(pmacController *pController) :
        asynMotorAxis((asynMotorController *) pController, 0),
        moving_(false) {
  int index = 0;
  // Initialise the table of CS controller pointers
  pCSControllers_ = (pmacCSController **) malloc(16 * sizeof(pmacCSController *));
  for (index = 0; index < 16; index++) {
    pCSControllers_[index] = NULL;
  }
}

pmacCSMonitor::~pmacCSMonitor() {
}

bool pmacCSMonitor::registerCS(pmacCSController *csPtr, int csNo) {
  if(pCSControllers_[csNo] == NULL) {
    // Add the CS to the list
    pCSControllers_[csNo] = csPtr;
    return true;
  } else {
    return false;
  }
}

asynStatus pmacCSMonitor::poll(bool *moving) {
  int i = 0;
  bool anyMoving = false;

  for (i = 0; i < 16; i++) {
    if (!pCSControllers_[i]) {
      continue;
    }
    if (pCSControllers_[i]->getMoving()) {
      anyMoving = true;
    }
  }
  *moving = anyMoving;
  return asynSuccess;
}
