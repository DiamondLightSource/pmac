/*
 * MockPMACAsynDriver.cpp
 *
 *  Created on: 29 Mar 2016
 *      Author: gnx91527
 */

#include "MockPMACAsynDriver.h"
#include <stdio.h>
#include <string>

/* asynCommon methods */
static void report(void *drvPvt,FILE *fp,int details)
{
  mPmacPvt *pmPmacPvt = (mPmacPvt *)drvPvt;
  pmPmacPvt->driver->report(drvPvt, fp, details);
}

static asynStatus aconnect(void *drvPvt,asynUser *pasynUser)
{
  printf("c connect called\n");
  mPmacPvt *pmPmacPvt = (mPmacPvt *)drvPvt;
  return pmPmacPvt->driver->connect(drvPvt, pasynUser);
}

static asynStatus adisconnect(void *drvPvt,asynUser *pasynUser)
{
  mPmacPvt *pmPmacPvt = (mPmacPvt *)drvPvt;
  return pmPmacPvt->driver->disconnect(drvPvt, pasynUser);
}

static asynCommon asyn = { report, aconnect, adisconnect };

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

static asynStatus cEchoFlush(void *drvPvt,asynUser *pasynUser)
{
  printf("c connect called\n");
  return asynSuccess;
}

static asynStatus cSetEos(void *drvPvt,asynUser *pasynUser, const char *eos,int eoslen)
{
  printf("c connect called\n");
  return asynSuccess;
}

static asynStatus cGetEos(void *drvPvt,asynUser *pasynUser, char *eos, int eossize, int *eoslen)
{
  printf("c connect called\n");
  return asynSuccess;
}

static asynOctet  *pasynOctet;
static mPmacPvt   *pmPmacPvt;

MockPMACAsynDriver::MockPMACAsynDriver(const char *dn, double delay, int noAutoConnect)
{
  //mPmacPvt   *pmPmacPvt;
  char       *portName;
  asynStatus status;
  size_t     nbytes;
  int        attributes;

  printf("Creating Mock\n");

  nbytes = sizeof(mPmacPvt) + sizeof(asynOctet) + strlen(dn) + 1;
  pmPmacPvt = (mPmacPvt *)callocMustSucceed(nbytes,sizeof(char),"MockPmacDriverInit");
  pasynOctet = (asynOctet *)(pmPmacPvt + 1);
  portName = (char *)(pasynOctet + 1);
  strcpy(portName,dn);
  printf("portName: %s\n", portName);
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
  } else {
    printf("Registration complete!\n");
  }
  status = pasynManager->registerInterface(portName,&pmPmacPvt->common);
  if(status!=asynSuccess){
      printf("echoDriverInit registerInterface failed\n");
      return;
  }

  pasynOctet->write = cPMACWrite;
  pasynOctet->read = cPMACRead;
  pasynOctet->flush = cEchoFlush;
  pasynOctet->setInputEos = cSetEos;
  pasynOctet->getInputEos = cGetEos;
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

  printf("Connect called\n");

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
printf("Data: %s\n", data);
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
  if(nchars>0){
    // Here we need to split the incoming string into tokens, then set the reply
    std::string keys(data);

    int status = 0;
    int running = 1;
    while (keys.find("\r") != std::string::npos && running == 1){
      std::string val = keys.substr(0, keys.find("\r"));
      keys = keys.substr(keys.find("\r")+1);
      if (keys.find(" ") != std::string::npos){
        std::string key = keys.substr(0, keys.find(" "));
        printf("KEY    %s", key.c_str());
        printf("VALUE  %s", val.c_str());
        //keys = keys.substr(keys.find(" ")+1);
        //if (this->store.hasKey(key)){
        //  this->store.insert(key, val);
        //}
      } else {
        std::string key = keys;
        printf("KEY    %s", key.c_str());
        printf("VALUE  %s", val.c_str());
        //debug(DEBUG_VARIABLE, functionName, "KEY  ", key);
        //debug(DEBUG_VARIABLE, functionName, "VALUE", val);
        //if (this->store.hasKey(key)){
        //  this->store.insert(key, val);
        //}
        running = 0;
      }
    }

    memcpy(pdeviceBuffer->buffer,data,nchars);
  }
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

asynStatus MockPMACAsynDriver::setDataItem(const std::string& key, const std::string& value)
{
  // Set the value in the data store
  data_[key] = value;
  return asynSuccess;
}
