/*
 * pmacHardwareInterface.cpp
 *
 *  Created on: 27 Oct 2016
 *      Author: gnx91527
 */

#include "pmacHardwareInterface.h"
#include "pmacController.h"

pmacHardwareInterface::pmacHardwareInterface() :
        pC_(0) {
}

pmacHardwareInterface::~pmacHardwareInterface() {
}

void pmacHardwareInterface::registerController(pmacController *pController) {
  pC_ = pController;
}
