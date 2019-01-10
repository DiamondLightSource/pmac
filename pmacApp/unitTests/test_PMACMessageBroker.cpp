/*
 * test_PMACMessageBroker.cpp
 *
 *  Created on: 16 Feb 2016
 *      Author: gnx91527
 */

#include <stdio.h>


#include "boost/test/unit_test.hpp"

#include <string.h>
#include <stdint.h>

#include <deque>
#include <tr1/memory>
#include <iostream>
#include <fstream>

#include "pmacTestingUtilities.h"
#include "asynPortDriver.h"
#include "MockPMACAsynDriver.h"
#include "pmacMessageBroker.h"

struct PMACMessageBrokerFixture
{
  MockPMACAsynDriver *pMock;
  pmacMessageBroker *pMB;
  asynUser *pAsynUser;
  std::string mockport;

  PMACMessageBrokerFixture()
  {
    mockport = "MOCK";
    uniqueAsynPortName(mockport);

    pMock = new MockPMACAsynDriver(mockport.c_str(), 0.01, 1);
    pAsynUser = pasynManager->createAsynUser(0, 0);
    pMB = new pmacMessageBroker(pAsynUser);
  }

  ~PMACMessageBrokerFixture()
  {
  }
};

class TestCallback : public pmacCallbackInterface
{
public:
  TestCallback()
  {
    sPtr_ = 0;
  };

  ~TestCallback(){};

  pmacCommandStore *sPtr_;

  void callback(pmacCommandStore *sPtr, int type)
  {
    sPtr_ = sPtr;
  }
};

BOOST_FIXTURE_TEST_SUITE(PMACMessageBrokerTest, PMACMessageBrokerFixture)

BOOST_AUTO_TEST_CASE(test_PMACMessageBroker)
{
  int connected = 0;
  int newConnection = 0;
  int noOfMsgs = 0;
  int totalBytesWritten = 0;
  int totalBytesRead = 0;
  int totalMsgTime = 0;
  int lastMsgBytesWritten = 0;
  int lastMsgBytesRead = 0;
  int lastMsgTime = 0;
  char command[1024];
  char response[1024];

  // First try to connect to a non-existent port
  BOOST_CHECK_EQUAL(pMB->connect("NOPORT", 0), asynError);

  // Check the broker is not connected
  BOOST_CHECK_NO_THROW(pMB->getConnectedStatus(&connected, &newConnection));
  BOOST_CHECK_EQUAL(connected, 0);
  BOOST_CHECK_EQUAL(newConnection, 1);

  // Try to send a write/read
  strcpy(command, "TEST COMMAND");
  BOOST_CHECK_EQUAL(pMB->immediateWriteRead(command, response), asynDisconnected);

  // Connect to the mock drive
  BOOST_CHECK_EQUAL(pMB->connect(mockport.c_str(), 0), asynSuccess);

  // Check the broker is connected
  // getConnectedStatus now sends a blank message to check connection
  // the message is 0 long and the OK response is 2 characters,
  // readStatistics expected results were altered accordingly
  pMock->setResponse("OK");
  BOOST_CHECK_NO_THROW(pMB->getConnectedStatus(&connected, &newConnection));
  BOOST_CHECK_EQUAL(connected, 1);
  BOOST_CHECK_EQUAL(newConnection, 1);
  pMB->clearNewConnection();
  pMB->getConnectedStatus(&connected, &newConnection);
  BOOST_CHECK_EQUAL(newConnection, 0);

  // Prime the mock with a response
  pMock->setResponse("TEST RESPONSE");
  // Now send a write/read and check the response
  strcpy(command, "TEST COMMAND 2");
  BOOST_CHECK_EQUAL(pMB->immediateWriteRead(command, response), asynSuccess);
  BOOST_CHECK_EQUAL(response, "TEST RESPONSE");
  BOOST_CHECK_EQUAL(pMock->checkForWrite("TEST COMMAND 2"), true);
  pMock->clearStore();

  // Add some read variables for each command store type
  BOOST_CHECK_EQUAL(pMB->addReadVariable(pmacMessageBroker::PMAC_SLOW_READ, "VAR1"), asynSuccess);
  BOOST_CHECK_EQUAL(pMB->addReadVariable(pmacMessageBroker::PMAC_SLOW_READ, "VAR2"), asynSuccess);
  BOOST_CHECK_EQUAL(pMB->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, "VAR3"), asynSuccess);
  BOOST_CHECK_EQUAL(pMB->addReadVariable(pmacMessageBroker::PMAC_MEDIUM_READ, "VAR4"), asynSuccess);
  BOOST_CHECK_EQUAL(pMB->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, "VAR5"), asynSuccess);
  BOOST_CHECK_EQUAL(pMB->addReadVariable(pmacMessageBroker::PMAC_FAST_READ, "VAR6"), asynSuccess);
  // Add a variable with an out of range type
  BOOST_CHECK_EQUAL(pMB->addReadVariable(5, "VAR7"), asynError);

  // Create and register a callback object for slow updates
  TestCallback *cbPtr1 = new TestCallback();
  BOOST_CHECK_NO_THROW(pMB->registerForUpdates(cbPtr1, pmacMessageBroker::PMAC_SLOW_READ));
  // Prime the response
  pMock->setResponse("10\r20\r");
  // Call update variables
  BOOST_CHECK_NO_THROW(pMB->updateVariables(pmacMessageBroker::PMAC_SLOW_READ));
  // Check the message sent to the mock driver
  BOOST_CHECK_EQUAL(pMock->checkForWrite("VAR2 VAR1"), true);
  // Now verify that the variables have the correct values
  BOOST_CHECK_EQUAL(cbPtr1->sPtr_->readValue("VAR1"), "20");
  BOOST_CHECK_EQUAL(cbPtr1->sPtr_->readValue("VAR2"), "10");

  // Create and register a callback object for medium updates
  TestCallback *cbPtr2 = new TestCallback();
  BOOST_CHECK_NO_THROW(pMB->registerForUpdates(cbPtr2, pmacMessageBroker::PMAC_MEDIUM_READ));
  // Prime the response
  pMock->setResponse("30\r40\r");
  // Call update variables
  BOOST_CHECK_NO_THROW(pMB->updateVariables(pmacMessageBroker::PMAC_MEDIUM_READ));
  // Check the message sent to the mock driver
  BOOST_CHECK_EQUAL(pMock->checkForWrite("VAR4 VAR3"), true);
  // Now verify that the variables have the correct values
  BOOST_CHECK_EQUAL(cbPtr2->sPtr_->readValue("VAR3"), "40");
  BOOST_CHECK_EQUAL(cbPtr2->sPtr_->readValue("VAR4"), "30");

  // Create and register a callback object for fast updates
  TestCallback *cbPtr3 = new TestCallback();
  BOOST_CHECK_NO_THROW(pMB->registerForUpdates(cbPtr3, pmacMessageBroker::PMAC_FAST_READ));
  // Prime the response
  pMock->setResponse("50\r60\r");
  // Call update variables
  BOOST_CHECK_NO_THROW(pMB->updateVariables(pmacMessageBroker::PMAC_FAST_READ));
  // Check the message sent to the mock driver
  BOOST_CHECK_EQUAL(pMock->checkForWrite("VAR6 VAR5"), true);
  // Now verify that the variables have the correct values
  BOOST_CHECK_EQUAL(cbPtr3->sPtr_->readValue("VAR5"), "60");
  BOOST_CHECK_EQUAL(cbPtr3->sPtr_->readValue("VAR6"), "50");

  // Verify the read update time is greater than zero
  BOOST_CHECK_GT(pMB->readUpdateTime(), 0.0);

  // Read out the statistics
  BOOST_CHECK_EQUAL(pMB->readStatistics(&noOfMsgs,
                                        &totalBytesWritten,
                                        &totalBytesRead,
                                        &totalMsgTime,
                                        &lastMsgBytesWritten,
                                        &lastMsgBytesRead,
                                        &lastMsgTime), asynSuccess);

  // Verify the number of messages sent is 4
  BOOST_CHECK_EQUAL(noOfMsgs, 5);
  // Verify total bytes written is 41
  BOOST_CHECK_EQUAL(totalBytesWritten, 41);
  // Verify total bytes read is 31
  BOOST_CHECK_EQUAL(totalBytesRead, 33);
  // Verify total message time is greater than 0
  BOOST_CHECK_GT(totalMsgTime, 0);
  // Verify last message bytes written is 9
  BOOST_CHECK_EQUAL(lastMsgBytesWritten, 9);
  // Verify last message bytes read is 6
  BOOST_CHECK_EQUAL(lastMsgBytesRead, 6);
  // Verify last message time is greater than 0
  BOOST_CHECK_GT(lastMsgTime, 0);

}

BOOST_AUTO_TEST_SUITE_END()
