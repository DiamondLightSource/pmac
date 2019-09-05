/*
 * pmacDebugger.h
 *
 *  Created on: 17 Feb 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_SRC_PMACDEBUGGER_H_
#define PMACAPP_SRC_PMACDEBUGGER_H_

#include <string>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <epicsTime.h>
#include "IntegerHashtable.h"

class pmacDebugger {
public:
    static const int DEBUG_ERROR;
    static const int DEBUG_VARIABLE;
    static const int DEBUG_TRACE;
    static const int DEBUG_FLOW;
    static const int DEBUG_TIMING;
    static const int DEBUG_PMAC;
    static const int DEBUG_PMAC_POLL;

    pmacDebugger(const std::string &owner);

    virtual ~pmacDebugger();

    void setLevel(int newLevel);

    int getLevel();

    void debug(int level, const std::string &method);

    void debugf(int level, const std::string &method, const char *pformat, ...);

    void debug(int level, const std::string &method, const std::string &message);

    void debug(int level, const std::string &method, const std::string &message,
               const std::string &value);

    void debug(int level, const std::string &method, const std::string &message, int value);

    void debug(int level, const std::string &method, const std::string &message, double value);

    void startTimer(int level, const std::string &method);

    void stopTimer(int level, const std::string &method, const std::string &message);

private:
    std::string owner_;
    int level_;
    IntegerHashtable timerSeconds_;
    IntegerHashtable timerNanoSeconds_;
};

#endif /* PMACAPP_SRC_PMACDEBUGGER_H_ */
