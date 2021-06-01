/*
 * pmac-valgrind.cpp
 *
 *  Created on: 31 Oct 2016
 *      Author: gnx91527
 */

#include <cmath>
#include <tr1/memory>
#include <asynPortClient.h>
#include <unistd.h>

#include "pmacAsynIPPort.h"
#include "pmacController.h"
#include "pmacAxis.h"
#include "pmacCSController.h"
#include "pmacCSAxis.h"


#define PMAC_PORT "BRICK1"

int main()
{
  // Some variables for setting up trajectory scan
  epicsFloat64 trajPoints[2000];
  epicsFloat64 trajTimes[2000];
  epicsInt32 trajUser[2000];
  epicsInt32 trajVel[2000];
  epicsInt32 running = 0;

  // Some helper shared pointers for read/write parameters
  std::tr1::shared_ptr<asynOctetClient> pOctetClient;
  std::tr1::shared_ptr<asynInt32Client> pIntClient;
  std::tr1::shared_ptr<asynFloat64Client> pFloatClient;
  std::tr1::shared_ptr<asynFloat64ArrayClient> pFloatArrayClient;
  std::tr1::shared_ptr<asynInt32ArrayClient> pIntArrayClient;

  pmacAsynIPConfigure("BRICK1Port", "127.0.0.1:1025");
  // Create an instance of the PMAC driver
  pmacController *pPmac = new pmacController(PMAC_PORT, "BRICK1Port", 0, 8, 0.2, 1.0);
  // Create the motor instances
  new pmacAxis(pPmac, 1);
  new pmacAxis(pPmac, 2);
  new pmacAxis(pPmac, 3);
  new pmacAxis(pPmac, 4);
  new pmacAxis(pPmac, 5);
  new pmacAxis(pPmac, 6);
  new pmacAxis(pPmac, 7);
  new pmacAxis(pPmac, 8);
  // Create an instance of a CS controller
  pmacCSController *pCs = new pmacCSController("CS1", PMAC_PORT, 1, 10);
  // Create the CS motor instances
  new pmacCSAxis(pCs, 1);
  new pmacCSAxis(pCs, 2);
  new pmacCSAxis(pCs, 3);
  new pmacCSAxis(pCs, 4);
  new pmacCSAxis(pCs, 5);
  new pmacCSAxis(pCs, 6);
  new pmacCSAxis(pCs, 7);
  new pmacCSAxis(pCs, 8);
  new pmacCSAxis(pCs, 9);

  // Start the PMAC polling thread
  pPmac->startPMACPolling();

  // Setup a trajectory scan
  for (int index = 0; index < 2000; index++){
    trajPoints[index] = 5.0 * sin(2.0 * 3.142 * double(index) / 2000.0);
    trajTimes[index] = 50000.0;
    trajUser[index] = 0;
    trajVel[index] = 0;
  }
  pFloatArrayClient = std::tr1::shared_ptr<asynFloat64ArrayClient>(new asynFloat64ArrayClient(PMAC_PORT, 0, PMAC_C_ProfilePositionsAString));
  pFloatArrayClient->write(trajPoints, 2000);
  pFloatArrayClient = std::tr1::shared_ptr<asynFloat64ArrayClient>(new asynFloat64ArrayClient(PMAC_PORT, 0, profileTimeArrayString));
  pFloatArrayClient->write(trajTimes, 2000);
  pIntArrayClient = std::tr1::shared_ptr<asynInt32ArrayClient>(new asynInt32ArrayClient(PMAC_PORT, 0, PMAC_C_ProfileUserString));
  pIntArrayClient->write(trajUser, 2000);
  pIntArrayClient = std::tr1::shared_ptr<asynInt32ArrayClient>(new asynInt32ArrayClient(PMAC_PORT, 0, PMAC_C_ProfileVelModeString));
  pIntArrayClient->write(trajVel, 2000);
  pIntClient = std::tr1::shared_ptr<asynInt32Client>(new asynInt32Client(PMAC_PORT, 0, PMAC_C_ProfileUseAxisAString));
  pIntClient->write(1);
  pIntClient = std::tr1::shared_ptr<asynInt32Client>(new asynInt32Client(PMAC_PORT, 0, profileNumPointsString));
  pIntClient->write(2000);
  pIntClient = std::tr1::shared_ptr<asynInt32Client>(new asynInt32Client(PMAC_PORT, 0, PMAC_C_TrajCSPortString));
  pIntClient->write(1);
  pIntClient = std::tr1::shared_ptr<asynInt32Client>(new asynInt32Client(PMAC_PORT, 0, PMAC_C_ProfileNumBuildString));
  pIntClient->write(2000);
  pIntClient = std::tr1::shared_ptr<asynInt32Client>(new asynInt32Client(PMAC_PORT, 0, profileBuildString));
  pIntClient->write(1);
  pIntClient = std::tr1::shared_ptr<asynInt32Client>(new asynInt32Client(PMAC_PORT, 0, profileExecuteString));
  pIntClient->write(1);

  // Wait for the trajectory scan to complete
  pIntClient->read(&running);
  while (running == 1){
    pIntClient->read(&running);
    sleep(1);
  }
  // Clean up clients
  pFloatArrayClient.reset();
  pIntArrayClient.reset();
  pIntClient.reset();
  // Motor classes are auto cleaned up when EPICS exit is called
  return 0;
}

