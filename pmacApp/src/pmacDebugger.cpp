/*
 * pmacDebugger.cpp
 *
 *  Created on: 17 Feb 2016
 *      Author: gnx91527
 */

#include "pmacDebugger.h"

const int pmacDebugger::DEBUG_ERROR = 0;
const int pmacDebugger::DEBUG_VARIABLE = 1;
const int pmacDebugger::DEBUG_TRACE = 2;
const int pmacDebugger::DEBUG_FLOW = 4;
const int pmacDebugger::DEBUG_TIMING = 8;
const int pmacDebugger::DEBUG_PMAC = 16;
const int pmacDebugger::DEBUG_PMAC_POLL = 32;

pmacDebugger::pmacDebugger(const std::string &owner) :
        owner_(owner),
        level_(0) {
}

pmacDebugger::~pmacDebugger() {
}

void pmacDebugger::setLevel(int newLevel) {
  level_ = newLevel;
}

int pmacDebugger::getLevel() {
  return level_;
}

void pmacDebugger::debug(int level, const std::string &method) {
  char tBuff[32];
  epicsTimeStamp ts;
  if (level == 0 || (level & level_) > 0) {
    epicsTimeGetCurrent(&ts);
    epicsTimeToStrftime(tBuff, 32, "%Y/%m/%d %H:%M:%S.%03f", &ts);
    printf("%s %s::%s called\n", tBuff, owner_.c_str(), method.c_str());
  }
}

void pmacDebugger::debug(int level, const std::string &method, const std::string &message) {
  char tBuff[32];
  epicsTimeStamp ts;
  if (level == 0 || (level & level_) > 0) {
    epicsTimeGetCurrent(&ts);
    epicsTimeToStrftime(tBuff, 32, "%Y/%m/%d %H:%M:%S.%03f", &ts);
    printf("%s %s::%s %s\n", tBuff, owner_.c_str(), method.c_str(), message.c_str());
  }
}

void pmacDebugger::debugf(int level, const std::string &method, const char *pformat, ...) {
  char tBuff[32];
  epicsTimeStamp ts;
  va_list pvar;
  if (level == 0 || (level & level_) > 0) {
    epicsTimeGetCurrent(&ts);
    epicsTimeToStrftime(tBuff, 32, "%Y/%m/%d %H:%M:%S.%03f", &ts);
    printf("%s %s::%s ", tBuff, owner_.c_str(), method.c_str());
    va_start(pvar, pformat);
    vprintf(pformat, pvar);
    printf("\n");
    va_end(pvar);
  }
}

void pmacDebugger::debug(int level, const std::string &method, const std::string &message,
                         const std::string &value) {
  char tBuff[32];
  size_t pos;
  epicsTimeStamp ts;
  std::string val = value;

  if (level == 0 || (level & level_) > 0) {
    epicsTimeGetCurrent(&ts);
    // allow printing of response from geobrick containing several \r
    while((pos = val.find('\r')) != std::string::npos) {
      val.replace(pos, (size_t) 1, " ");
    }
    epicsTimeToStrftime(tBuff, 32, "%Y/%m/%d %H:%M:%S.%03f", &ts);
    printf("%s %s::%s %s => %s\n", tBuff, owner_.c_str(), method.c_str(), message.c_str(),
           val.c_str());
  }
}

void
pmacDebugger::debug(int level, const std::string &method, const std::string &message, int value) {
  char tBuff[32];
  epicsTimeStamp ts;
  if (level == 0 || (level & level_) > 0) {
    epicsTimeGetCurrent(&ts);
    epicsTimeToStrftime(tBuff, 32, "%Y/%m/%d %H:%M:%S.%03f", &ts);
    printf("%s %s::%s %s => %d\n", tBuff, owner_.c_str(), method.c_str(), message.c_str(), value);
  }
}

void pmacDebugger::debug(int level, const std::string &method, const std::string &message,
                         double value) {
  char tBuff[32];
  epicsTimeStamp ts;
  if (level == 0 || (level & level_) > 0) {
    epicsTimeGetCurrent(&ts);
    epicsTimeToStrftime(tBuff, 32, "%Y/%m/%d %H:%M:%S.%03f", &ts);
    printf("%s %s::%s %s => %f\n", tBuff, owner_.c_str(), method.c_str(), message.c_str(), value);
  }
}

void pmacDebugger::startTimer(int level, const std::string &method) {
  if (level == 0 || (level & level_) > 0) {
    epicsTimeStamp ts;
    epicsTimeGetCurrent(&ts);
    timerSeconds_.insert(method, ts.secPastEpoch);
    timerNanoSeconds_.insert(method, ts.nsec);
  }
}

void pmacDebugger::stopTimer(int level, const std::string &method, const std::string &message) {
  if (level == 0 || (level & level_) > 0) {
    epicsTimeStamp ts1, ts2;
    epicsTimeGetCurrent(&ts2);
    if (timerSeconds_.hasKey(method)) {
      ts1.secPastEpoch = timerSeconds_.lookup(method);
      ts1.nsec = timerNanoSeconds_.lookup(method);
      double dt = epicsTimeDiffInSeconds(&ts2, &ts1);
      printf("%s::%s : %s => %f\n", owner_.c_str(), method.c_str(), message.c_str(), dt);
    }
  }
}
