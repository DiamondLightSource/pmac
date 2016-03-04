/*
 * pmacMessageBroker.cpp
 *
 *  Created on: 4 Feb 2016
 *      Author: gnx91527
 */

#include "pmacMessageBroker.h"

const epicsUInt32  pmacMessageBroker::PMAC_MAXBUF_  = 1024;
const epicsFloat64 pmacMessageBroker::PMAC_TIMEOUT_ = 5.0;

pmacMessageBroker::pmacMessageBroker(asynUser *pasynUser) :
  pmacDebugger("pmacMessageBroker"),
  ownerAsynUser_(pasynUser),
  lowLevelPortUser_(0),
  noOfMessages_(0),
  totalBytesWritten_(0),
  totalBytesRead_(0)
{
  epicsTimeGetCurrent(&this->startTime_);
  epicsTimeGetCurrent(&this->currentTime_);
  slowCallbacks_ = new pmacCallbackStore(pmacMessageBroker::PMAC_SLOW_READ);
  mediumCallbacks_ = new pmacCallbackStore(pmacMessageBroker::PMAC_MEDIUM_READ);
  fastCallbacks_ = new pmacCallbackStore(pmacMessageBroker::PMAC_FAST_READ);
}

pmacMessageBroker::~pmacMessageBroker ()
{
}

asynStatus pmacMessageBroker::connect(const char *port, int addr)
{
  static const char *functionName = "connect";
  asynStatus status = asynSuccess;
  debug(DEBUG_FLOW, functionName, "Connecting to low level asynOctetSyncIO port", port);

  //Connect our Asyn user to the low level port that is a parameter to this constructor
  status = lowLevelPortConnect(port, addr, &lowLevelPortUser_, (char*)"\006", (char*)"\r");
  if (status != asynSuccess){
    debug(DEBUG_ERROR, functionName, "Failed to connect to low level asynOctetSyncIO port", port);
  }
  return status;
}

/**
 * Utilty function to print the connected status of the low level asyn port.
 * @return asynStatus
 */
asynStatus pmacMessageBroker::printConnectedStatus()
{
  static const char *functionName = "pmacController::printConnectedStatus";
  asynStatus status = asynSuccess;
  int asynManagerConnected = 0;

  if (lowLevelPortUser_){
    status = pasynManager->isConnected(lowLevelPortUser_, &asynManagerConnected);
    if (status!=asynSuccess) {
      asynPrint(this->ownerAsynUser_, ASYN_TRACE_ERROR, "pmacController: Error calling pasynManager::isConnected.\n");
      return status;
    } else {
      asynPrint(this->ownerAsynUser_, ASYN_TRACE_FLOW, "%s isConnected: %d\n", functionName, asynManagerConnected);
    }
  }
  return status;
}

asynStatus pmacMessageBroker::immediateWriteRead(const char *command, char *response)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "immediateWriteRead";
  this->startTimer(DEBUG_ERROR, functionName);
  status = this->lowLevelWriteRead(command, response);
  this->stopTimer(DEBUG_ERROR, functionName, "PMAC write/read time");
  return status;
}

asynStatus pmacMessageBroker::addReadVariable(int type, const char *variable)
{
  asynStatus status = asynSuccess;

  // Lock the mutex
  mutex_.lock();

  if (type == PMAC_SLOW_READ){
    slowStore_.addItem(variable);
  } else if (type == PMAC_MEDIUM_READ){
    mediumStore_.addItem(variable);
  } else if (type == PMAC_FAST_READ){
    fastStore_.addItem(variable);
  } else {
    status = asynError;
  }

  // Unlock the mutex
  mutex_.unlock();

  return status;
}

asynStatus pmacMessageBroker::updateVariables(int type)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "updateVariables";
  char response[1024];
  std::string cmd;
  int noOfCmds = 0;
  epicsTimeStamp ts1, ts2;

  // Keep a record of start time for the update
  epicsTimeGetCurrent(&ts1);
  // Lock the mutex
  mutex_.lock();

  startTimer(DEBUG_TIMING, functionName);

  if (type == PMAC_FAST_READ){
    if (fastStore_.size() > 0){
      // Send the command string and read the response
      noOfCmds = fastStore_.countCommandStrings();
      debug(DEBUG_VARIABLE, functionName, "Fast Store command string count", noOfCmds);
      for (int index = 0; index < noOfCmds; index++){
        cmd = fastStore_.readCommandString(index);
        this->immediateWriteRead(cmd.c_str(), response);
        debug(DEBUG_VARIABLE, functionName, "PMAC reply string length", (int)strlen(response));
        // Update the store with the response
        fastStore_.updateReply(cmd, response);
      }
      // Perform the necessary callbacks
      fastCallbacks_->callCallbacks(&fastStore_);
    }
  } else if (type == PMAC_MEDIUM_READ){
    if (mediumStore_.size() > 0){
      // Send the command string and read the response
      noOfCmds = mediumStore_.countCommandStrings();
      for (int index = 0; index < noOfCmds; index++){
        cmd = mediumStore_.readCommandString(index);
        this->immediateWriteRead(cmd.c_str(), response);
        // Update the store with the response
        mediumStore_.updateReply(cmd, response);
      }
      // Perform the necessary callbacks
      mediumCallbacks_->callCallbacks(&mediumStore_);
    }
  } else if (type == PMAC_SLOW_READ){
    if (slowStore_.size() > 0){
      // Send the command string and read the response
      noOfCmds = slowStore_.countCommandStrings();
      for (int index = 0; index < noOfCmds; index++){
        cmd = slowStore_.readCommandString(index);
        this->immediateWriteRead(cmd.c_str(), response);
        // Update the store with the response
        slowStore_.updateReply(cmd, response);
      }
      // Perform the necessary callbacks
      slowCallbacks_->callCallbacks(&slowStore_);
    }
  } else {
    status = asynError;
  }
  stopTimer(DEBUG_TIMING, functionName, "Time taken for updates");

  // Unlock the mutex
  mutex_.unlock();

  // Record the end time for the update
  epicsTimeGetCurrent(&ts2);
  // Calculate the time taken to perform the update (and convert to ms)
  updateTime_ = 1000.0 * epicsTimeDiffInSeconds(&ts2, &ts1);

  return asynSuccess;
}

asynStatus pmacMessageBroker::registerForUpdates(pmacCallbackInterface *cbPtr, int type)
{
  asynStatus status = asynSuccess;

  // Lock the mutex
  mutex_.lock();

  if (type == PMAC_FAST_READ){
    fastCallbacks_->registerCallback(cbPtr);
  } else if (type == PMAC_MEDIUM_READ){
    mediumCallbacks_->registerCallback(cbPtr);
  } else if (type == PMAC_SLOW_READ){
    slowCallbacks_->registerCallback(cbPtr);
  } else {
    status = asynError;
  }

  // Unlock the mutex
  mutex_.unlock();

  return status;
}

double pmacMessageBroker::readUpdateTime()
{
  return updateTime_;
}

/**
 * Connect to the underlying low level Asyn port that is used for comms.
 * This uses the asynOctetSyncIO interface, and also sets the input and output terminators.
 * @param port The port to connect to
 * @param addr The address of the port to connect to
 * @param ppasynUser A pointer to the pasynUser structure used by the controller
 * @param inputEos The input EOS character
 * @param outputEos The output EOS character
 * @return asynStatus
 */
asynStatus pmacMessageBroker::lowLevelPortConnect(const char *port, int addr, asynUser **ppasynUser, char *inputEos, char *outputEos)
{
  static const char *functionName = "pmacController::lowLevelPortConnect";
  asynStatus status = asynSuccess;

  asynPrint(this->ownerAsynUser_, ASYN_TRACE_FLOW, "%s\n", functionName);

  status = pasynOctetSyncIO->connect( port, addr, ppasynUser, NULL);
  if (status) {
    asynPrint(this->ownerAsynUser_, ASYN_TRACE_ERROR,
        "pmacController::motorAxisAsynConnect: unable to connect to port %s\n",
        port);
    return status;
  }

  //Do I want to disconnect below? If the IP address comes up, will the driver recover
  //if the poller functions are running? Might have to use asynManager->isConnected to
  //test connection status of low level port (in the pollers). But then autosave
  //restore doesn't work (and we would save wrong positions). So I need to
  //have a seperate function(s) to deal with connecting after IOC init.

  status = pasynOctetSyncIO->setInputEos(*ppasynUser, inputEos, strlen(inputEos) );
  if (status) {
    asynPrint(this->ownerAsynUser_, ASYN_TRACE_ERROR,
        "pmacController: unable to set input EOS on %s: %s\n",
        port, (*ppasynUser)->errorMessage);
    pasynOctetSyncIO->disconnect(*ppasynUser);
    //Set my low level pasynUser pointer to NULL
    *ppasynUser = NULL;
    return status;
  }

  status = pasynOctetSyncIO->setOutputEos(*ppasynUser, outputEos, strlen(outputEos));
  if (status) {
    asynPrint(this->ownerAsynUser_, ASYN_TRACE_ERROR,
        "pmacController: unable to set output EOS on %s: %s\n",
        port, (*ppasynUser)->errorMessage);
    pasynOctetSyncIO->disconnect(*ppasynUser);
    //Set my low level pasynUser pointer to NULL
    *ppasynUser = NULL;
    return status;
  }

  return status;
}

/**
 * Wrapper for asynOctetSyncIO write/read functions.
 * @param command - String command to send.
 * @response response - String response back.
 */
asynStatus pmacMessageBroker::lowLevelWriteRead(const char *command, char *response)
{
  asynStatus status = asynSuccess;
  int eomReason = 0;
  size_t nwrite = 0;
  size_t nread = 0;
  static const char *functionName = "pmacController::lowLevelWriteRead";

  asynPrint(this->ownerAsynUser_, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (!lowLevelPortUser_) {
    return asynError;
  }

  asynPrint(lowLevelPortUser_, ASYN_TRACEIO_DRIVER, "%s: command: %s\n", functionName, command);

  status = pasynOctetSyncIO->writeRead(lowLevelPortUser_ ,
                                       command,
                                       strlen(command),
                                       response,
                                       PMAC_MAXBUF_,
                                       PMAC_TIMEOUT_,
                                       &nwrite,
                                       &nread,
                                       &eomReason);

  if (status) {
    asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, "%s: Error from pasynOctetSyncIO->writeRead. command: %s\n", functionName, command);
  } else {
    // Update statistics
    this->noOfMessages_++;
    this->totalBytesWritten_ += strlen(command);
    this->totalBytesRead_ += strlen(response);
    epicsTimeGetCurrent(&this->currentTime_);
  }

  asynPrint(lowLevelPortUser_, ASYN_TRACEIO_DRIVER, "%s: response: %s\n", functionName, response);

  return status;
}

void pmacMessageBroker::report()
{
  double elapsedTime = 0.0;
  printf("Total number of messages: %d\n", this->noOfMessages_);
  printf("Total number of bytes written: %d\n", this->totalBytesWritten_);
  printf("Total number of bytes read: %d\n", this->totalBytesRead_);
  elapsedTime = epicsTimeDiffInSeconds(&this->currentTime_, &this->startTime_);
  printf("Average message rate: %.2f / s\n", (double)this->noOfMessages_/elapsedTime);
  printf("Average data rate: %.2f\n", (double)this->totalBytesWritten_/elapsedTime);
}
