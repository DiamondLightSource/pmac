/*
 * MockPMACAsynDriver.cpp
 *
 *  Created on: 29 Mar 2016
 *      Author: gnx91527
 */

#include "MockPMACAsynDriver.h"
#include <stdio.h>
#include <string>


MockPMACAsynDriver::MockPMACAsynDriver(const char *portName, double delay, int noAutoConnect) :
  asynPortDriver(portName,
                 0,
                 0,
                 asynOctetMask,
                 asynOctetMask,
                 ASYN_CANBLOCK,
                 1,
                 0,
                 0)
{
  delay_ = delay;
  waitingForResponse_ = false;
  response_ = "";
}

MockPMACAsynDriver::~MockPMACAsynDriver()
{

}

asynStatus MockPMACAsynDriver::readOctet(asynUser *pasynUser,
                                         char *value,
                                         size_t maxChars,
                                         size_t *nActual,
                                         int *eomReason)
{
  if (waitingForResponse_ == true){
    if (response_ != ""){
      strncpy(value, response_.c_str(), maxChars);
      if (response_.length() > maxChars){
        *nActual = maxChars;
      } else {
        *nActual = response_.length();
      }
    }
    *eomReason = 2;
  }
  waitingForResponse_ = false;
  return asynSuccess;
}

asynStatus MockPMACAsynDriver::writeOctet(asynUser *pasynUser,
                                          const char *value,
                                          size_t maxChars,
                                          size_t *nActual)
{
  std::string input(value);
  writes_.push_back(input);
  waitingForResponse_ = true;
  epicsThreadSleep(delay_);
  return asynSuccess;
}

void MockPMACAsynDriver::setResponse(const std::string& response)
{
  response_ = response;
}

void MockPMACAsynDriver::clearStore()
{
  writes_.clear();
}

bool MockPMACAsynDriver::checkForWrite(const std::string& item)
{
  bool found = false;
  // loop over stored writes_ and look for string
  std::vector<std::string>::iterator iter;
  for (iter = writes_.begin(); iter != writes_.end(); ++iter){
    if (*iter == item){
      found = true;
    }
  }
  return found;
}

bool MockPMACAsynDriver::checkForWrite(const std::string& item, int index)
{
  bool found = false;
  // loop over stored writes_ and look for string
  if (writes_[index] == item){
    found = true;
  }
  return found;
}

