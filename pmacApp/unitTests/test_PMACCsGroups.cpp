/*
 * test_PMACCsGroups.cpp
 *
 *  Created on: 4 Apr 2016
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
#include "pmacController.h"
#include "pmacCsGroups.h"
#include "MockPMACAsynDriver.h"


struct PMACCsGroupsFixture
{
  MockPMACAsynDriver *pMock;
  pmacController *pPmac;
  pmacCsGroups *pGroups;

  PMACCsGroupsFixture()
  {
    std::string mockport("MOCK");
    uniqueAsynPortName(mockport);
    std::string pmacport("PMAC");
    uniqueAsynPortName(pmacport);

    pMock = new MockPMACAsynDriver(mockport.c_str(), 0.0, 1);
    // Set the response to error for initial messages
    pMock->setResponse("604020\006");
    pMock->setOnceOnly();
    //pMock->setResponse("\007ERR003\006");
    pPmac = new pmacController(pmacport.c_str(), mockport.c_str(), 0, 8, 0.2, 1.0);
    pGroups = new pmacCsGroups(pPmac);
    // Clear the response
    pMock->setResponse("");
    // Clear out any startup messages
    pMock->clearStore();
  }

  ~PMACCsGroupsFixture()
  {
  }
};

BOOST_FIXTURE_TEST_SUITE(PMACCsGroupsTest, PMACCsGroupsFixture)

BOOST_AUTO_TEST_CASE(test_PMACCsGroups)
{
  // Add a new group
  pGroups->addGroup(1, "Test1", 4);
  // Add some Axes to the group
  pGroups->addAxisToGroup(1, 1, "X", 1);
  pGroups->addAxisToGroup(1, 2, "Y", 1);
  pGroups->addAxisToGroup(1, 3, "Z", 2);
  pGroups->addAxisToGroup(1, 4, "A", 2);

  // Switch to the group
  pGroups->switchToGroup(1);

  // Check the first message is undefine all
  BOOST_CHECK_EQUAL(pMock->checkForWrite("undefine all", 0), true);

  // Now check that the messages sent contain the axis definitions
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&1 #1->X"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&1 #2->Y"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&2 #3->Z"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&2 #4->A"), true);
  // Clear out messages
  pMock->clearStore();

  // Check the coordinate systems are as expected for each of the axes
  BOOST_CHECK_EQUAL(pGroups->getAxisCoordSys(1), 1);
  BOOST_CHECK_EQUAL(pGroups->getAxisCoordSys(2), 1);
  BOOST_CHECK_EQUAL(pGroups->getAxisCoordSys(3), 2);
  BOOST_CHECK_EQUAL(pGroups->getAxisCoordSys(4), 2);

  // Try a non-existant axis, check exception is thrown
  BOOST_CHECK_THROW(pGroups->getAxisCoordSys(5), std::out_of_range);

  // Attempt to switch to a group that does not exist
  // todo why is this failing?
  // BOOST_CHECK_EQUAL(pGroups->switchToGroup(2), asynError);

  // Attempt to add an axis to a group that does not exist
  BOOST_CHECK_EQUAL(pGroups->addAxisToGroup(2, 1, "X", 1), asynError);

  // Create a new group
  pGroups->addGroup(2, "Test2", 8);
  // Add the axes
  pGroups->addAxisToGroup(2, 1, "X", 1);
  pGroups->addAxisToGroup(2, 2, "Y", 1);
  pGroups->addAxisToGroup(2, 3, "Z", 1);
  pGroups->addAxisToGroup(2, 4, "A", 1);
  pGroups->addAxisToGroup(2, 5, "B", 1);
  pGroups->addAxisToGroup(2, 6, "C", 1);
  pGroups->addAxisToGroup(2, 7, "U", 1);
  pGroups->addAxisToGroup(2, 8, "V", 1);

  // Switch to the group
  pGroups->switchToGroup(2);

  // Check the messages sent contain CS aborts and undefine all
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&1A"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&2A"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("undefine all"), true);
  // Now check that the messages sent contain the axis definitions
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&1 #1->X"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&1 #2->Y"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&1 #3->Z"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&1 #4->A"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&1 #5->B"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&1 #6->C"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&1 #7->U"), true);
  BOOST_CHECK_EQUAL(pMock->checkForWrite("&1 #8->V"), true);
  // Clear out messages
  pMock->clearStore();


}

BOOST_AUTO_TEST_SUITE_END()


