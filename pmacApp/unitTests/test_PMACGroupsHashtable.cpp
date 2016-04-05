/*
 * test_PMACGroupsHashtable.cpp
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

#include "pmacGroupsHashtable.h"

struct pmacGroupsHashtableFixture
{
  pmacGroupsHashtable table;

  pmacGroupsHashtableFixture()
  {
  }

  ~pmacGroupsHashtableFixture()
  {
  }
};

BOOST_FIXTURE_TEST_SUITE(PMACCharIntHastableTest, pmacGroupsHashtableFixture)

BOOST_AUTO_TEST_CASE(test_pmacGroupsHashtable)
{
  void *ptr = 0;
  void *ptr1 = 0;
  void *ptr2 = 0;
  void *ptr3 = 0;
  void *ptr4 = 0;
  void *ptr5 = 0;
  void *ptr6 = 0;
  void *ptr7 = 0;
  void *ptr8 = 0;
  void *ptr9 = 0;
  void *ptr10 = 0;

  // Perform a lookup on an empty table with a zero ID
  BOOST_CHECK_NO_THROW(ptr = table.lookup(0));

  // Verify the ptr is NULL
  BOOST_CHECK(!ptr);

  // Check the size is zero
  BOOST_CHECK_EQUAL(table.count(), 0);

  // Add a couple of voids
  ptr1 = malloc(10);
  ptr2 = malloc(10);
  table.insert(5, ptr1);
  table.insert(8, ptr2);

  // Check the size is 2
  BOOST_CHECK_EQUAL(table.count(), 2);

  // Now lookup the second entry
  ptr = table.lookup(8);
  BOOST_CHECK_EQUAL(ptr, ptr2);

  // Now lookup the first entry
  ptr = table.lookup(5);
  BOOST_CHECK_EQUAL(ptr, ptr1);

  // Now remove the second entry
  ptr = table.remove(8);
  BOOST_CHECK_EQUAL(ptr, ptr2);

  // Check the size is 1
  BOOST_CHECK_EQUAL(table.count(), 1);

  // Attempt to lookup the second entry
  ptr = table.lookup(8);
  BOOST_CHECK(!ptr);

  // Add some additional items
  ptr3 = malloc(10);
  ptr4 = malloc(10);
  ptr5 = malloc(10);
  ptr6 = malloc(10);
  ptr7 = malloc(10);
  ptr8 = malloc(10);
  ptr9 = malloc(10);
  ptr10 = malloc(10);
  table.insert(10, ptr3);
  table.insert(11, ptr4);
  table.insert(12, ptr5);
  table.insert(13, ptr6);
  table.insert(14, ptr7);
  table.insert(15, ptr8);
  table.insert(16, ptr9);
  table.insert(17, ptr10);

  // Check the size is 9
  BOOST_CHECK_EQUAL(table.count(), 9);

  // Now lookup entry
  ptr = table.lookup(13);
  BOOST_CHECK_EQUAL(ptr, ptr6);

  // Now lookup entry
  ptr = table.lookup(16);
  BOOST_CHECK_EQUAL(ptr, ptr9);

  // Now lookup entry
  ptr = table.lookup(10);
  BOOST_CHECK_EQUAL(ptr, ptr3);

  // Now lookup entry
  ptr = table.lookup(17);
  BOOST_CHECK_EQUAL(ptr, ptr10);


}

BOOST_AUTO_TEST_SUITE_END()

