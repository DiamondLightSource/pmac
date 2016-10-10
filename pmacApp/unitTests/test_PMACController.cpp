/*
 * test_PMACController.cpp
 *
 *  Created on: 5 Oct 2016
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

#include <asynPortClient.h>

#include "pmacTestingUtilities.h"
#include "MockPMACAsynDriver.h"

// Very naughty define to provide easy access to members for testing
#define private public
#include "pmacController.h"


struct PMACControllerFixture
{
  MockPMACAsynDriver *pMock;
  pmacController *pPmac;
  std::string pmacport;

  PMACControllerFixture()
  {
    std::string mockport("MOCK");
    uniqueAsynPortName(mockport);
    pmacport = "PMAC";
    uniqueAsynPortName(pmacport);

    pMock = new MockPMACAsynDriver(mockport.c_str(), 0.01, 1);
    // Set the response to error for initial messages
    pMock->setResponse("\007ERR003\006");
    pPmac = new pmacController(pmacport.c_str(), mockport.c_str(), 0, 8, 0.2, 1.0);
    // Clear the response
    pMock->setResponse("");
    // Clear out any startup messages
    pMock->clearStore();
  }

  ~PMACControllerFixture()
  {
  }
};

BOOST_FIXTURE_TEST_SUITE(PMACControllerTest, PMACControllerFixture)

BOOST_AUTO_TEST_CASE(test_PMACController)
{
  char input[128];
  char output[128];
  std::tr1::shared_ptr<asynInt32Client> pClient;

  // Test the processDrvInfo method for evaluating expressions
  strcpy(input, "No expression");
  pPmac->processDrvInfo(input, output);
  BOOST_CHECK_EQUAL(output, "No expression");
  strcpy(input, "`Bad expression");
  pPmac->processDrvInfo(input, output);
  BOOST_CHECK_EQUAL(output, "`Bad expression");
  strcpy(input, "P`4000+32`");
  pPmac->processDrvInfo(input, output);
  BOOST_CHECK_EQUAL(output, "P4032");
  strcpy(input, "P`4100-32`");
  pPmac->processDrvInfo(input, output);
  BOOST_CHECK_EQUAL(output, "P4068");
  strcpy(input, "P`4000 + 32`");
  pPmac->processDrvInfo(input, output);
  BOOST_CHECK_EQUAL(output, "P4032");
  strcpy(input, "P`4100 - 32`");
  pPmac->processDrvInfo(input, output);
  BOOST_CHECK_EQUAL(output, "P4068");

  // Test the connection
  pPmac->checkConnection();
  BOOST_CHECK_EQUAL(pPmac->connected_, 1);


  // Test the low level write read method
  pMock->clearStore();
  pMock->setResponse("test response");
  strcpy(input, "test command");
  pPmac->immediateWriteRead(input, output);
  BOOST_CHECK_EQUAL(output, "test response");
  BOOST_CHECK_EQUAL(pMock->checkForWrite("test command"), true);

  // Test the axis write method with write enabled
  // The message should reach the mock port and a response is received
  pClient = std::tr1::shared_ptr<asynInt32Client>(new asynInt32Client(pmacport.c_str(), 0, PMAC_C_AxisReadonlyString));
  pClient->write(0);
  pMock->clearStore();
  pMock->setResponse("test response 1");
  strcpy(input, "test command 1");
  pPmac->axisWriteRead(input, output);
  BOOST_CHECK_EQUAL(output, "test response 1");
  BOOST_CHECK_EQUAL(pMock->checkForWrite("test command 1"), true);

  // Test the axis write method with write disabled
  // The message should not reach the mock port and NO response is received
  pClient = std::tr1::shared_ptr<asynInt32Client>(new asynInt32Client(pmacport.c_str(), 0, PMAC_C_AxisReadonlyString));
  pClient->write(1);
  pMock->clearStore();
  pMock->setResponse("test response 2");
  strcpy(input, "test command 2");
  strcpy(output, "null");
  pPmac->axisWriteRead(input, output);
  BOOST_CHECK_EQUAL(output, "null");
  BOOST_CHECK_EQUAL(pMock->checkForWrite("test command 2"), false);

}

BOOST_AUTO_TEST_SUITE_END()




