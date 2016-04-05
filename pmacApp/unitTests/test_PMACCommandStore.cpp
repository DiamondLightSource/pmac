/*
 * test_PMACCommandStore.cpp
 *
 *  Created on: 15 Feb 2016
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

#include "StringHashtable.h"
#include "pmacCommandStore.h"

struct PMACCommandStoreFixture
{
  pmacCommandStore store;

  PMACCommandStoreFixture()
  {
  }

  ~PMACCommandStoreFixture()
  {
  }
};

BOOST_FIXTURE_TEST_SUITE(PMACCommandStoreTest, PMACCommandStoreFixture)

BOOST_AUTO_TEST_CASE(test_PMACCommandStore)
{
  // Add some items
  store.addItem("i1");
  store.addItem("i2");
  store.addItem("i3");
  store.addItem("i4");
  store.addItem("i5");

  // Check the size
  BOOST_CHECK_EQUAL(store.size(), 5);

  // Check for the items
  BOOST_CHECK_EQUAL(store.checkForItem("i1"), true);
  BOOST_CHECK_EQUAL(store.checkForItem("i2"), true);
  BOOST_CHECK_EQUAL(store.checkForItem("i3"), true);
  BOOST_CHECK_EQUAL(store.checkForItem("i4"), true);
  BOOST_CHECK_EQUAL(store.checkForItem("i5"), true);

  // Check for an item that has not been added
  BOOST_CHECK_EQUAL(store.checkForItem("i6"), false);

  // Check the size
  BOOST_CHECK_EQUAL(store.size(), 5);

  // Now add the item and check again
  store.addItem("i6");
  BOOST_CHECK_EQUAL(store.checkForItem("i6"), true);

  // Check the size
  BOOST_CHECK_EQUAL(store.size(), 6);

  // Check the command string contains all of the requested variables
  std::string cmdString = store.readCommandString(0);
  BOOST_CHECK_NE(cmdString.find("i1"), std::string::npos);
  BOOST_CHECK_NE(cmdString.find("i2"), std::string::npos);
  BOOST_CHECK_NE(cmdString.find("i3"), std::string::npos);
  BOOST_CHECK_NE(cmdString.find("i4"), std::string::npos);
  BOOST_CHECK_NE(cmdString.find("i5"), std::string::npos);
  BOOST_CHECK_NE(cmdString.find("i6"), std::string::npos);

  // Check for a non inserted value
  BOOST_CHECK_EQUAL(cmdString.find("i7"), std::string::npos);

  // Insert the value and check again
  store.addItem("i7");
  cmdString = store.readCommandString(0);
  BOOST_CHECK_NE(cmdString.find("i7"), std::string::npos);

  // Check the size
  BOOST_CHECK_EQUAL(store.size(), 7);

  // Now setup a reply string
  std::string reply = "10\r20\r30\r40\r50\r60\r70\r\6";
  int status = store.updateReply(cmdString, reply);

  // Check the status is good
  BOOST_CHECK_EQUAL(status, 0);

  // Check each value
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(0,2)), "10");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(3,2)), "20");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(6,2)), "30");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(9,2)), "40");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(12,2)), "50");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(15,2)), "60");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(18,2)), "70");

  // Now setup a reply string with too many values
  reply = "1\r2\r3\r4\r5\r6\r7\r8\r\6";
  status = store.updateReply(cmdString, reply);

  // Check the status is good
  BOOST_CHECK_EQUAL(status, 0);

  // Check each value
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(0,2)), "1");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(3,2)), "2");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(6,2)), "3");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(9,2)), "4");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(12,2)), "5");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(15,2)), "6");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(18,2)), "7");

  // Now setup a reply string with not enough values
  reply = "100\r200\r300\r\6";
  status = store.updateReply(cmdString, reply);

  // Check the status is good
  BOOST_CHECK_EQUAL(status, 0);

  // Check each value
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(0,2)), "100");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(3,2)), "200");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(6,2)), "300");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(9,2)), "4");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(12,2)), "5");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(15,2)), "6");
  BOOST_CHECK_EQUAL(store.readValue(cmdString.substr(18,2)), "7");

}

BOOST_AUTO_TEST_SUITE_END()
