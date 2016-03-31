/*
 * MockPMACAsynDriver.cpp
 *
 *  Created on: 29 Mar 2016
 *      Author: gnx91527
 */

#include "MockPMACAsynDriver.h"

/* asynCommon methods */
static void report(void *drvPvt,FILE *fp,int details)
{
  mPmacPvt *pmPmacPvt = (mPmacPvt *)drvPvt;
  pmPmacPvt->driver->report(drvPvt, fp, details);
}

static asynStatus connect(void *drvPvt,asynUser *pasynUser)
{
  mPmacPvt *pmPmacPvt = (mPmacPvt *)drvPvt;
  return pmPmacPvt->driver->connect(drvPvt, pasynUser);
}

static asynStatus disconnect(void *drvPvt,asynUser *pasynUser)
{
  mPmacPvt *pmPmacPvt = (mPmacPvt *)drvPvt;
  return pmPmacPvt->driver->disconnect(drvPvt, pasynUser);
}

static asynCommon asyn = { report, connect, disconnect };

static asynStatus cPMACWrite(void *drvPvt,
                             asynUser *pasynUser,
                             const char *data,
                             size_t nchars,
                             size_t *nbytesTransfered)
{
  mPmacPvt *pmPmacPvt = (mPmacPvt *)drvPvt;
  return pmPmacPvt->driver->write(drvPvt, pasynUser, data, nchars, nbytesTransfered);
}

static asynStatus cPMACRead(void *drvPvt,
                            asynUser *pasynUser,
                            char *data,
                            size_t maxchars,
                            size_t *nbytesTransfered,
                            int *eomReason)
{
  mPmacPvt *pmPmacPvt = (mPmacPvt *)drvPvt;
  return pmPmacPvt->driver->read(drvPvt, pasynUser, data, maxchars, nbytesTransfered, eomReason);
}


MockPMACAsynDriver::MockPMACAsynDriver(const char *dn, double delay, int noAutoConnect)
{
  mPmacPvt   *pmPmacPvt;
  char       *portName;
  asynStatus status;
  size_t     nbytes;
  int        attributes;
  asynOctet  *pasynOctet;

  nbytes = sizeof(mPmacPvt) + sizeof(asynOctet) + strlen(dn) + 1;
  pmPmacPvt = (mPmacPvt *)callocMustSucceed(nbytes,sizeof(char),"MockPmacDriverInit");
  pasynOctet = (asynOctet *)(pmPmacPvt + 1);
  portName = (char *)(pasynOctet + 1);
  strcpy(portName,dn);
  pmPmacPvt->portName = portName;
  pmPmacPvt->driver = this;
  pmPmacPvt->delay = delay;
  pmPmacPvt->common.interfaceType = asynCommonType;
  pmPmacPvt->common.pinterface  = (void *)&asyn;
  pmPmacPvt->common.drvPvt = pmPmacPvt;
  attributes = 0;
  if(delay>0.0) attributes|=ASYN_CANBLOCK;
  status = pasynManager->registerPort(portName,attributes,!noAutoConnect,0,0);
  if(status!=asynSuccess) {
      printf("echoDriverInit registerDriver failed\n");
      return;
  }
  status = pasynManager->registerInterface(portName,&pmPmacPvt->common);
  if(status!=asynSuccess){
      printf("echoDriverInit registerInterface failed\n");
      return;
  }

  pasynOctet->write = cPMACWrite;
  pasynOctet->read = cPMACRead;
//  pasynOctet->flush = echoFlush;
//  pasynOctet->setInputEos = setEos;
//  pasynOctet->getInputEos = getEos;
  pmPmacPvt->octet.interfaceType = asynOctetType;
  pmPmacPvt->octet.pinterface  = pasynOctet;
  pmPmacPvt->octet.drvPvt = pmPmacPvt;
  status = pasynOctetBase->initialize(portName,&pmPmacPvt->octet,1,1,0);
  if(status==asynSuccess){
    status = pasynManager->registerInterruptSource(portName,&pmPmacPvt->octet,&pmPmacPvt->pasynPvt);
  }
  if(status!=asynSuccess){
    printf("echoDriverInit registerInterface failed\n");
  }
}

MockPMACAsynDriver::~MockPMACAsynDriver()
{

}

void MockPMACAsynDriver::report(void *drvPvt,FILE *fp,int details)
{
  mPmacPvt *pmPmacPvt = (mPmacPvt *)drvPvt;
  int i,n;

  fprintf(fp,"    echoDriver. "
          "connected:%s delay = %f\n",
          (pmPmacPvt->connected ? "Yes" : "No"),
          pmPmacPvt->delay);
  n = NUM_DEVICES;
  for(i=0;i<n;i++) {
    fprintf(fp,"        device %d connected:%s nchars = %d\n",
            i,
            (pmPmacPvt->device[i].connected ? "Yes" : "No"),
            (int)pmPmacPvt->device[i].buffer.nchars);
  }
}

asynStatus MockPMACAsynDriver::connect(void *drvPvt,asynUser *pasynUser)
{
  mPmacPvt    *pmPmacPvt = (mPmacPvt *)drvPvt;
  int        addr;
  asynStatus status;

  status = pasynManager->getAddr(pasynUser,&addr);
  if(status!=asynSuccess) return status;
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
      "%s echoDriver:connect addr %d\n",pmPmacPvt->portName,addr);
  if(pmPmacPvt->connected) {
    asynPrint(pasynUser,ASYN_TRACE_ERROR,
              "%s echoDriver:connect port already connected\n",
              pmPmacPvt->portName);
    return asynError;
  }
  // simulate connection delay
  if(pmPmacPvt->delay>0.0) epicsThreadSleep(pmPmacPvt->delay*10.);
  pmPmacPvt->connected = 1;
  pmPmacPvt->device[0].connected = 1;
  pasynManager->exceptionConnect(pasynUser);
  return asynSuccess;
}

asynStatus MockPMACAsynDriver::disconnect(void *drvPvt,asynUser *pasynUser)
{
  mPmacPvt    *pmPmacPvt = (mPmacPvt *)drvPvt;
  int        addr;
  asynStatus status;

  status = pasynManager->getAddr(pasynUser,&addr);
  if(status!=asynSuccess) return status;
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s echoDriver:disconnect addr %d\n",pmPmacPvt->portName,addr);
  if(!pmPmacPvt->connected) {
    asynPrint(pasynUser,ASYN_TRACE_ERROR,
              "%s echoDriver:disconnect port not connected\n",
              pmPmacPvt->portName);
    return asynError;
  }
  pmPmacPvt->connected = 0;
  pmPmacPvt->device[0].connected = 0;
  pasynManager->exceptionDisconnect(pasynUser);
  return asynSuccess;
}

asynStatus MockPMACAsynDriver::write(void *drvPvt,
                                     asynUser *pasynUser,
                                     const char *data,
                                     size_t nchars,
                                     size_t *nbytesTransfered)
{
  mPmacPvt     *pmPmacPvt = (mPmacPvt *)drvPvt;
  deviceInfo   *pdeviceInfo;
  deviceBuffer *pdeviceBuffer;
  int          addr;
  asynStatus   status;

  status = pasynManager->getAddr(pasynUser,&addr);
  if(status!=asynSuccess) return status;
  addr = 0;
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s echoDriver:write addr %d\n",pmPmacPvt->portName,addr);
  if(addr<0 || addr>=NUM_DEVICES) {
      epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
          "addr %d is illegal. Must be 0 or 1",addr);
      return asynError;
  }
  pdeviceInfo = &pmPmacPvt->device[addr];
  if(!pdeviceInfo->connected) {
      asynPrint(pasynUser,ASYN_TRACE_ERROR,
          "%s echoDriver:write device %d not connected\n",
          pmPmacPvt->portName,addr);
      epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
          "%s echoDriver:write device %d not connected",
          pmPmacPvt->portName,addr);
      return asynError;
  }
  if(pmPmacPvt->delay>pasynUser->timeout) {
      if(pasynUser->timeout>0.0) epicsThreadSleep(pasynUser->timeout);
      asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s echoDriver write timeout\n",pmPmacPvt->portName);
      epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
                    "%s echoDriver write timeout",pmPmacPvt->portName);
      return asynTimeout;
  }
  pdeviceBuffer = &pdeviceInfo->buffer;
  if(nchars>BUFFERSIZE) nchars = BUFFERSIZE;
  if(nchars>0) memcpy(pdeviceBuffer->buffer,data,nchars);
  asynPrintIO(pasynUser,ASYN_TRACEIO_DRIVER,data,nchars,
              "echoWrite nchars %lu\n",(unsigned long)nchars);
  pdeviceBuffer->nchars = nchars;
  if(pmPmacPvt->delay>0.0) epicsThreadSleep(pmPmacPvt->delay);
  *nbytesTransfered = nchars;
  return status;
}

asynStatus MockPMACAsynDriver::read(void *drvPvt,
                                    asynUser *pasynUser,
                                    char *data,
                                    size_t maxchars,
                                    size_t *nbytesTransfered,
                                    int *eomReason)
{
  mPmacPvt     *pmPmacPvt = (mPmacPvt *)drvPvt;
  deviceInfo   *pdeviceInfo;
  deviceBuffer *pdeviceBuffer;
  char         *pfrom,*pto;
  char         thisChar;
  size_t       nremaining;
  size_t       nout = 0;
  int          addr;
  asynStatus   status;

  if(eomReason) *eomReason=0;
  if(nbytesTransfered) *nbytesTransfered = 0;
  status = pasynManager->getAddr(pasynUser,&addr);
  if(status!=asynSuccess) return status;
  addr = 0;
  asynPrint(pasynUser, ASYN_TRACE_FLOW,
      "%s echoDriver:read addr %d\n",pmPmacPvt->portName,addr);
  if(addr<0 || addr>=NUM_DEVICES) {
      epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
          "addr %d is illegal. Must be 0 or 1",addr);
      return(asynError);
  }
  pdeviceInfo = &pmPmacPvt->device[addr];
  if(!pdeviceInfo->connected) {
      asynPrint(pasynUser,ASYN_TRACE_ERROR,
          "%s echoDriver:read device %d not connected\n",
          pmPmacPvt->portName,addr);
      epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
          "%s echoDriver:read device %d not connected",
          pmPmacPvt->portName,addr);
      return asynError;
  }
  if(pmPmacPvt->delay>pasynUser->timeout) {
      if(pasynUser->timeout>0.0) epicsThreadSleep(pasynUser->timeout);
      asynPrint(pasynUser, ASYN_TRACE_ERROR,
          "%s echoDriver read timeout\n",pmPmacPvt->portName);
      epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
          "%s echoDriver read timeout",pmPmacPvt->portName);
      return asynTimeout;
  }
  if(pmPmacPvt->delay>0.0) epicsThreadSleep(pmPmacPvt->delay);
  pdeviceBuffer = &pdeviceInfo->buffer;
  nremaining = pdeviceBuffer->nchars;
  pdeviceBuffer->nchars = 0;
  pfrom = pdeviceBuffer->buffer;
  pto = data;
  while(nremaining>0 && nout<maxchars) {
      thisChar = *pto++ = *pfrom++; nremaining--; nout++;
      if(pmPmacPvt->eoslen>0) {
          if(thisChar==pmPmacPvt->eos[0]) {
              if(pmPmacPvt->eoslen==1) {
                  if(eomReason) *eomReason |= ASYN_EOM_EOS;
                  break;
              }
              if(nremaining==0) {
                  if(eomReason) *eomReason |= ASYN_EOM_CNT;
                  break;
              }
              if(*pfrom==pmPmacPvt->eos[1]) {
                  *pto++ = *pfrom++; nremaining--; nout++;
                  if(eomReason) {
                      *eomReason |= ASYN_EOM_EOS;
                      if(nremaining==0) *eomReason |= ASYN_EOM_CNT;
                      break;
                  }
              }
          }
     }
  }
  if(nbytesTransfered) *nbytesTransfered = nout;
  if(eomReason) {
      if(*nbytesTransfered>=maxchars) *eomReason |= ASYN_EOM_CNT;
      if(nremaining==0) *eomReason |= ASYN_EOM_END;
  }
  pasynOctetBase->callInterruptUsers(pasynUser,pmPmacPvt->pasynPvt,
      data,nbytesTransfered,eomReason);
  asynPrintIO(pasynUser,ASYN_TRACEIO_DRIVER,data,nout,
      "echoRead nbytesTransfered %lu\n",(unsigned long)*nbytesTransfered);
  return status;
}
