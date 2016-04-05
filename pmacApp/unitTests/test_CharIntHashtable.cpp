/*
 * test_CharIntHashtable.cpp
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

#include "CharIntHashtable.h"

struct CharIntHashtableFixture
{
  CharIntHashtable table;

  CharIntHashtableFixture()
  {
  }

  ~CharIntHashtableFixture()
  {
  }
};

BOOST_FIXTURE_TEST_SUITE(PMACCharIntHastableTest, CharIntHashtableFixture)

BOOST_AUTO_TEST_CASE(test_CharIntHashtable)
{
  int val = 0;

  // Perform a lookup on an empty table, check it throws an exception
  BOOST_REQUIRE_THROW(val = table.lookup('A'), std::out_of_range);

  // Check the size is zero
  BOOST_CHECK_EQUAL(table.count(), 0);

  // Add some values
  table.insert('A', 9);
  table.insert('B', 8);
  table.insert('C', 7);
  table.insert('D', 6);
  table.insert('E', 5);
  table.insert('F', 4);
  table.insert('G', 3);
  table.insert('H', 2);
  table.insert('I', 1);

  // Check the size is 9
  BOOST_CHECK_EQUAL(table.count(), 9);

  // Now lookup the entries
  BOOST_CHECK_NO_THROW(val = table.lookup('E'));
  BOOST_CHECK_EQUAL(val, 5);
  BOOST_CHECK_NO_THROW(val = table.lookup('F'));
  BOOST_CHECK_EQUAL(val, 4);
  BOOST_CHECK_NO_THROW(val = table.lookup('G'));
  BOOST_CHECK_EQUAL(val, 3);
  BOOST_CHECK_NO_THROW(val = table.lookup('H'));
  BOOST_CHECK_EQUAL(val, 2);
  BOOST_CHECK_NO_THROW(val = table.lookup('I'));
  BOOST_CHECK_EQUAL(val, 1);
  BOOST_CHECK_NO_THROW(val = table.lookup('A'));
  BOOST_CHECK_EQUAL(val, 9);
  BOOST_CHECK_NO_THROW(val = table.lookup('B'));
  BOOST_CHECK_EQUAL(val, 8);
  BOOST_CHECK_NO_THROW(val = table.lookup('C'));
  BOOST_CHECK_EQUAL(val, 7);
  BOOST_CHECK_NO_THROW(val = table.lookup('D'));
  BOOST_CHECK_EQUAL(val, 6);

  // Now remove the B entry
  val = table.remove('B');
  BOOST_CHECK_EQUAL(val, 8);

  // Check the size is 8
  BOOST_CHECK_EQUAL(table.count(), 8);

  // Perform a lookup of B, check it throws an exception
  BOOST_REQUIRE_THROW(val = table.lookup('B'), std::out_of_range);

}

BOOST_AUTO_TEST_SUITE_END()

