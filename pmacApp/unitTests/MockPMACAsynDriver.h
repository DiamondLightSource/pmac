/*
 * MockPMACAsynDriver.h
 *
 *  Created on: 29 Mar 2016
 *      Author: gnx91527
 */

#ifndef PMACAPP_UNITTESTS_MOCKPMACASYNDRIVER_H_
#define PMACAPP_UNITTESTS_MOCKPMACASYNDRIVER_H_

#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string>
#include <map>
#include <vector>

#include <cantProceed.h>
#include <epicsStdio.h>
#include <epicsThread.h>
#include <iocsh.h>

#include <asynPortDriver.h>
#include <asynOctet.h>
#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#define BUFFERSIZE 4096
#define NUM_DEVICES 1

class MockPMACAsynDriver : public asynPortDriver
{
public:
  MockPMACAsynDriver(const char *portName, double delay, int noAutoConnect);
  virtual ~MockPMACAsynDriver();

  virtual asynStatus readOctet(asynUser *pasynUser,
                               char *value,
                               size_t maxChars,
                               size_t *nActual,
                               int *eomReason);
  virtual asynStatus writeOctet(asynUser *pasynUser,
                                const char *value,
                                size_t maxChars,
                                size_t *nActual);

  void setResponse(const std::string& response);
  void setOnceOnly();
  void clearStore();
  bool checkForWrite(const std::string& item);
  bool checkForWrite(const std::string& item, int index);

private:
  bool waitingForResponse_;
  double delay_;
  std::vector<std::string> writes_;
  std::string response_;
  bool only_once_;

};

#endif /* PMACAPP_UNITTESTS_MOCKPMACASYNDRIVER_H_ */
