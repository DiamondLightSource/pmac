/*
 * test_IntegerHashtable.cpp
 *
 *  Created on: 10 Oct 2016
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
#include <unistd.h>
#include <ios>

#include "pmacTestingUtilities.h"
#include "IntegerHashtable.h"


struct HashtableTestFixture
{
};

BOOST_FIXTURE_TEST_SUITE(IntegetHashtableTest, HashtableTestFixture)

BOOST_AUTO_TEST_CASE(test_IntHashtable)
{
  IntegerHashtable t1;
  int value;

  // Check size
  BOOST_CHECK_EQUAL(t1.count(), 0);

  // Try to read a non value
  BOOST_CHECK_THROW(t1.lookup("NOVAL"), std::out_of_range);

  // Check size again
  BOOST_CHECK_EQUAL(t1.count(), 0);

  // Now add some values
  BOOST_CHECK_NO_THROW(t1.insert("I1", 1));
  BOOST_CHECK_NO_THROW(t1.insert("I2", 2));
  BOOST_CHECK_NO_THROW(t1.insert("I3", 3));
  BOOST_CHECK_NO_THROW(t1.insert("I4", 4));
  BOOST_CHECK_NO_THROW(t1.insert("I5", 5));
  BOOST_CHECK_NO_THROW(t1.insert("I6", 6));
  BOOST_CHECK_NO_THROW(t1.insert("I7", 7));
  BOOST_CHECK_NO_THROW(t1.insert("I8", 8));
  BOOST_CHECK_NO_THROW(t1.insert("I9", 9));
  BOOST_CHECK_NO_THROW(t1.insert("I0", 10));
  BOOST_CHECK_NO_THROW(t1.insert("A1", 11));
  BOOST_CHECK_NO_THROW(t1.insert("A2", 12));
  BOOST_CHECK_NO_THROW(t1.insert("A3", 13));
  BOOST_CHECK_NO_THROW(t1.insert("A4", 14));
  BOOST_CHECK_NO_THROW(t1.insert("A5", 15));
  BOOST_CHECK_NO_THROW(t1.insert("A6", 16));
  BOOST_CHECK_NO_THROW(t1.insert("A7", 17));
  BOOST_CHECK_NO_THROW(t1.insert("A8", 18));
  BOOST_CHECK_NO_THROW(t1.insert("A9", 19));
  BOOST_CHECK_NO_THROW(t1.insert("A0", 20));

  // Check size is equal to 20
  BOOST_CHECK_EQUAL(t1.count(), 20);

  // Check a few values are correct
  value = t1.lookup("I4");
  BOOST_CHECK_EQUAL(value, 4);
  value = t1.lookup("I8");
  BOOST_CHECK_EQUAL(value, 8);
  value = t1.lookup("I0");
  BOOST_CHECK_EQUAL(value, 10);
  value = t1.lookup("A1");
  BOOST_CHECK_EQUAL(value, 11);
  value = t1.lookup("A7");
  BOOST_CHECK_EQUAL(value, 17);

  // Overwrite two values
  t1.insert("I5", 101);
  t1.insert("A4", 102);

  // Check size is still equal to 20
  BOOST_CHECK_EQUAL(t1.count(), 20);

  // Check a few values are correct
  value = t1.lookup("I4");
  BOOST_CHECK_EQUAL(value, 4);
  value = t1.lookup("I8");
  BOOST_CHECK_EQUAL(value, 8);
  value = t1.lookup("I0");
  BOOST_CHECK_EQUAL(value, 10);
  value = t1.lookup("A1");
  BOOST_CHECK_EQUAL(value, 11);
  value = t1.lookup("A7");
  BOOST_CHECK_EQUAL(value, 17);
  value = t1.lookup("I5");
  BOOST_CHECK_EQUAL(value, 101);
  value = t1.lookup("A4");
  BOOST_CHECK_EQUAL(value, 102);

  // Delete some entries
  value = t1.remove("I4");
  BOOST_CHECK_EQUAL(value, 4);
  value = t1.remove("I5");
  BOOST_CHECK_EQUAL(value, 101);

  // Check size is now equal to 18
  BOOST_CHECK_EQUAL(t1.count(), 18);

  // Check values of deleted entries, check they are empty
  BOOST_CHECK_THROW(t1.lookup("I4"), std::out_of_range);
  BOOST_CHECK_THROW(t1.lookup("I5"), std::out_of_range);

}

BOOST_AUTO_TEST_CASE(test_IntHashtableMemory)
{
  IntegerHashtable t1;
  double vm = 0.0;
  double res = 0.0;
  double vm_base = 0.0;
  double res_base = 0.0;

  // Record the base memory usage
  process_mem_usage(vm, res);
  BOOST_MESSAGE("VM: " << vm << "; RSS: " << res);
  process_mem_usage(vm_base, res_base);
  BOOST_MESSAGE("VM: " << vm_base << "; RSS: " << res_base);

  // Insert a value 10 times
  for (int index = 0; index < 10; index++){
    t1.insert("KEY1", 1);
  }
  // Record the base memory usage
  process_mem_usage(vm, res);
  BOOST_MESSAGE("VM: " << vm << "; RSS: " << res);
  // Verify memory usage change less than 1%
  BOOST_CHECK_CLOSE(vm/vm_base, 1.0, 1.0);
  BOOST_CHECK_CLOSE(res/res_base, 1.0, 1.0);

  // Insert the same value 10 million times
  for (int index = 0; index < 10000000; index++){
    t1.insert("KEY1", 1);
  }

  // Check memory usage
  process_mem_usage(vm, res);
  BOOST_MESSAGE("VM: " << vm << "; RSS: " << res);
  // Verify memory usage change less than 1%
  BOOST_CHECK_CLOSE(vm/vm_base, 1.0, 1.0);
  BOOST_CHECK_CLOSE(res/res_base, 1.0, 1.0);

  // Insert the same value 10 million times
  for (int index = 0; index < 10000000; index++){
    t1.insert("KEY1", 1);
  }

  // Check memory usage
  process_mem_usage(vm, res);
  BOOST_MESSAGE("VM: " << vm << "; RSS: " << res);
  // Verify memory usage change less than 1%
  BOOST_CHECK_CLOSE(vm/vm_base, 1.0, 1.0);
  BOOST_CHECK_CLOSE(res/res_base, 1.0, 1.0);

  // Insert the same value 10 million times
  for (int index = 0; index < 10000000; index++){
    t1.insert("KEY1", 1);
  }

  // Check memory usage
  process_mem_usage(vm, res);
  BOOST_MESSAGE("VM: " << vm << "; RSS: " << res);
  // Verify memory usage change less than 1%
  BOOST_CHECK_CLOSE(vm/vm_base, 1.0, 1.0);
  BOOST_CHECK_CLOSE(res/res_base, 1.0, 1.0);
}

BOOST_AUTO_TEST_SUITE_END()



