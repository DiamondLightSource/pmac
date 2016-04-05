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

#include <cantProceed.h>
#include <epicsStdio.h>
#include <epicsThread.h>
#include <iocsh.h>

#include <asynDriver.h>
#include <asynOctet.h>
#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#define BUFFERSIZE 4096
#define NUM_DEVICES 1

class MockPMACAsynDriver;

typedef struct deviceBuffer {
    char buffer[BUFFERSIZE];
    size_t  nchars;
} deviceBuffer;

typedef struct deviceInfo {
    deviceBuffer buffer;
    int          connected;
} deviceInfo;

typedef struct mPmacPvt {
    deviceInfo    device[NUM_DEVICES];
    const char    *portName;
    MockPMACAsynDriver *driver;
    int           connected;
    double        delay;
    asynInterface common;
    asynInterface octet;
    char          eos[2];
    int           eoslen;
    void          *pasynPvt;   /*For registerInterruptSource*/
} mPmacPvt;


/* asynOctet methods */
/*static asynStatus echoWrite(void *drvPvt,asynUser *pasynUser,
    const char *data,size_t numchars,size_t *nbytesTransfered);
static asynStatus echoRead(void *drvPvt,asynUser *pasynUser,
    char *data,size_t maxchars,size_t *nbytesTransfered,int *eomReason);
static asynStatus echoFlush(void *drvPvt,asynUser *pasynUser);
static asynStatus setEos(void *drvPvt,asynUser *pasynUser,
    const char *eos,int eoslen);
static asynStatus getEos(void *drvPvt,asynUser *pasynUser,
    char *eos, int eossize, int *eoslen);
*/

class MockPMACAsynDriver
{
public:
  MockPMACAsynDriver(const char *dn, double delay, int noAutoConnect);
  virtual ~MockPMACAsynDriver();
  void report(void *drvPvt,FILE *fp,int details);
  asynStatus connect(void *drvPvt,asynUser *pasynUser);
  asynStatus disconnect(void *drvPvt,asynUser *pasynUser);
  asynStatus write(void *drvPvt,
                   asynUser *pasynUser,
                   const char *data,
                   size_t nchars,
                   size_t *nbytesTransfered);
  asynStatus read(void *drvPvt,
                  asynUser *pasynUser,
                  char *data,
                  size_t maxchars,
                  size_t *nbytesTransfered,
                  int *eomReason);
  asynStatus setDataItem(const std::string& key, const std::string& value);

private:
  std::map<std::string, std::string> data_;

};

#endif /* PMACAPP_UNITTESTS_MOCKPMACASYNDRIVER_H_ */
