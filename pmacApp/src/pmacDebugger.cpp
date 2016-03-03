/*
 * pmacDebugger.cpp
 *
 *  Created on: 17 Feb 2016
 *      Author: gnx91527
 */

#include "pmacDebugger.h"

const int pmacDebugger::DEBUG_ERROR    = 0;
const int pmacDebugger::DEBUG_VARIABLE = 1;
const int pmacDebugger::DEBUG_TRACE    = 2;
const int pmacDebugger::DEBUG_FLOW     = 4;
const int pmacDebugger::DEBUG_TIMING   = 8;

pmacDebugger::pmacDebugger(const std::string& owner) :
  owner_(owner),
  level_(0)
{
}

pmacDebugger::~pmacDebugger()
{
}

void pmacDebugger::setLevel(int newLevel)
{
  level_ = newLevel;
}

int pmacDebugger::getLevel()
{
  return level_;
}

void pmacDebugger::debug(int level, const std::string& method)
{
  if (level == 0 || (level & level_) > 0){
    printf("%s::%s called\n", owner_.c_str(), method.c_str());
  }
}

void pmacDebugger::debug(int level, const std::string& method, const std::string& message)
{
  if (level == 0 || (level & level_) > 0){
    printf("%s::%s %s\n", owner_.c_str(), method.c_str(), message.c_str());
  }
}

void pmacDebugger::debugf(int level, const std::string& method, const char *pformat, ...)
{
  va_list pvar;
  if (level == 0 || (level & level_) > 0){
    printf("%s::%s ", owner_.c_str(), method.c_str());
    va_start(pvar,pformat);
    vprintf(pformat, pvar);
    printf("\n");
    va_end(pvar);
  }
}

void pmacDebugger::debug(int level, const std::string& method, const std::string& message, const std::string& value)
{
  if (level == 0 || (level & level_) > 0){
    printf("%s::%s %s => %s\n", owner_.c_str(), method.c_str(), message.c_str(), value.c_str());
  }
}

void pmacDebugger::debug(int level, const std::string& method, const std::string& message, int value)
{
  if (level == 0 || (level & level_) > 0){
    printf("%s::%s %s => %d\n", owner_.c_str(), method.c_str(), message.c_str(), value);
  }
}

void pmacDebugger::debug(int level, const std::string& method, const std::string& message, double value)
{
  if (level == 0 || (level & level_) > 0){
    printf("%s::%s %s => %f\n", owner_.c_str(), method.c_str(), message.c_str(), value);
  }
}

void pmacDebugger::startTimer(int level, const std::string& method)
{
  if (level == 0 || (level & level_) > 0){
    epicsTimeStamp ts;
    epicsTimeGetCurrent(&ts);
    timerSeconds_.insert(method, ts.secPastEpoch);
    timerNanoSeconds_.insert(method, ts.nsec);
  }
}

void pmacDebugger::stopTimer(int level, const std::string& method, const std::string& message)
{
  if (level == 0 || (level & level_) > 0){
    epicsTimeStamp ts1, ts2;
    epicsTimeGetCurrent(&ts2);
    if (timerSeconds_.hasKey(method)){
      ts1.secPastEpoch = timerSeconds_.lookup(method);
      ts1.nsec = timerNanoSeconds_.lookup(method);
      double dt = epicsTimeDiffInSeconds(&ts2, &ts1);
      printf("%s::%s : %s => %lf\n", owner_.c_str(), method.c_str(), message.c_str(), dt);
    }
  }
}
