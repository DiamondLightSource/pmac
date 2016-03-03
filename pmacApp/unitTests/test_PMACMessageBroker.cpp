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

#include "pmacAsynIPPort.h"
#include "StringHashtable.h"
#include "pmacCommandStore.h"

struct PMACMessageBrokerFixture
{

  PMACMessageBrokerFixture()
  {
  }

  ~PMACMessageBrokerFixture()
  {
  }
};

BOOST_FIXTURE_TEST_SUITE(PMACMessageBrokerTest, PMACMessageBrokerFixture)

BOOST_AUTO_TEST_CASE(test_PMACMessageBroker)
{
  pmacAsynIPConfigure("PMAC", "localhost:1025");


}

BOOST_AUTO_TEST_SUITE_END()
