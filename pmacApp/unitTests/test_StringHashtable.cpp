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
#include "StringHashtable.h"

struct HashtableTestFixture
{
};

BOOST_FIXTURE_TEST_SUITE(StringHashtableTest, HashtableTestFixture)

BOOST_AUTO_TEST_CASE(test_Hashtable)
{
  StringHashtable t1;
  std::string value;

  // Check size
  BOOST_CHECK_EQUAL(t1.count(), 0);

  // Try to read a non value
  value = t1.lookup("NOVAL");
  BOOST_CHECK_EQUAL(value, "");

  // Check size again
  BOOST_CHECK_EQUAL(t1.count(), 0);

  // Now add some values
  BOOST_CHECK_NO_THROW(t1.insert("I1", "VAL01"));
  BOOST_CHECK_NO_THROW(t1.insert("I2", "VAL02"));
  BOOST_CHECK_NO_THROW(t1.insert("I3", "VAL03"));
  BOOST_CHECK_NO_THROW(t1.insert("I4", "VAL04"));
  BOOST_CHECK_NO_THROW(t1.insert("I5", "VAL05"));
  BOOST_CHECK_NO_THROW(t1.insert("I6", "VAL06"));
  BOOST_CHECK_NO_THROW(t1.insert("I7", "VAL07"));
  BOOST_CHECK_NO_THROW(t1.insert("I8", "VAL08"));
  BOOST_CHECK_NO_THROW(t1.insert("I9", "VAL09"));
  BOOST_CHECK_NO_THROW(t1.insert("I0", "VAL10"));
  BOOST_CHECK_NO_THROW(t1.insert("A1", "VAL11"));
  BOOST_CHECK_NO_THROW(t1.insert("A2", "VAL12"));
  BOOST_CHECK_NO_THROW(t1.insert("A3", "VAL13"));
  BOOST_CHECK_NO_THROW(t1.insert("A4", "VAL14"));
  BOOST_CHECK_NO_THROW(t1.insert("A5", "VAL15"));
  BOOST_CHECK_NO_THROW(t1.insert("A6", "VAL16"));
  BOOST_CHECK_NO_THROW(t1.insert("A7", "VAL17"));
  BOOST_CHECK_NO_THROW(t1.insert("A8", "VAL18"));
  BOOST_CHECK_NO_THROW(t1.insert("A9", "VAL19"));
  BOOST_CHECK_NO_THROW(t1.insert("A0", "VAL20"));

  // Check size is equal to 20
  BOOST_CHECK_EQUAL(t1.count(), 20);

  // Check a few values are correct
  value = t1.lookup("I4");
  BOOST_CHECK_EQUAL(value, "VAL04");
  value = t1.lookup("I8");
  BOOST_CHECK_EQUAL(value, "VAL08");
  value = t1.lookup("I0");
  BOOST_CHECK_EQUAL(value, "VAL10");
  value = t1.lookup("A1");
  BOOST_CHECK_EQUAL(value, "VAL11");
  value = t1.lookup("A7");
  BOOST_CHECK_EQUAL(value, "VAL17");

  // Overwrite two values
  t1.insert("I5", "NEWVAL1");
  t1.insert("A4", "NEWVAL2");

  // Check size is still equal to 20
  BOOST_CHECK_EQUAL(t1.count(), 20);

  // Check a few values are correct
  value = t1.lookup("I4");
  BOOST_CHECK_EQUAL(value, "VAL04");
  value = t1.lookup("I8");
  BOOST_CHECK_EQUAL(value, "VAL08");
  value = t1.lookup("I0");
  BOOST_CHECK_EQUAL(value, "VAL10");
  value = t1.lookup("A1");
  BOOST_CHECK_EQUAL(value, "VAL11");
  value = t1.lookup("A7");
  BOOST_CHECK_EQUAL(value, "VAL17");
  value = t1.lookup("I5");
  BOOST_CHECK_EQUAL(value, "NEWVAL1");
  value = t1.lookup("A4");
  BOOST_CHECK_EQUAL(value, "NEWVAL2");

  // Delete some entries
  value = t1.remove("I4");
  BOOST_CHECK_EQUAL(value, "VAL04");
  value = t1.remove("I5");
  BOOST_CHECK_EQUAL(value, "NEWVAL1");

  // Check size is now equal to 18
  BOOST_CHECK_EQUAL(t1.count(), 18);

  // Check values of deleted entries, check they are empty
  value = t1.lookup("I4");
  BOOST_CHECK_EQUAL(value, "");
  value = t1.lookup("I5");
  BOOST_CHECK_EQUAL(value, "");

  // Verify large inserts do not trash memory
  // Unit test for problem discovered with powerPMAC reading position
  // of CS axes (9th Nov 2016).
  BOOST_CHECK_NO_THROW(t1.insert("&1Q89", "-0.123456789876543212345"));
  BOOST_CHECK_NO_THROW(t1.insert("&1Q89", "-0.123456789876543212345"));
  BOOST_CHECK_NO_THROW(t1.insert("&1Q87654321", "-0.1234567892352647214876543212345"));
  BOOST_CHECK_NO_THROW(t1.insert("&1Q87654321", "-0.123243242242235252235797"));
  BOOST_CHECK_NO_THROW(t1.insert("&1Q89", "-0.12345678987"));
  BOOST_CHECK_NO_THROW(t1.insert("&1Q89", "-0.123456789876543212345"));
}

BOOST_AUTO_TEST_CASE(test_HashtableMemory)
{
  StringHashtable t1;
  std::string value;
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
    t1.insert("KEY1", "VALUE1");
  }
  // Record the base memory usage
  process_mem_usage(vm, res);
  BOOST_MESSAGE("VM: " << vm << "; RSS: " << res);
  // Verify memory usage change less than 1%
  BOOST_CHECK_CLOSE(vm/vm_base, 1.0, 1.0);
  BOOST_CHECK_CLOSE(res/res_base, 1.0, 1.0);

  // Insert the same value 10 million times
  for (int index = 0; index < 10000000; index++){
    t1.insert("KEY1", "VALUE1");
  }

  // Check memory usage
  process_mem_usage(vm, res);
  BOOST_MESSAGE("VM: " << vm << "; RSS: " << res);
  // Verify memory usage change less than 1%
  BOOST_CHECK_CLOSE(vm/vm_base, 1.0, 1.0);
  BOOST_CHECK_CLOSE(res/res_base, 1.0, 1.0);

  // Insert the same value 10 million times
  for (int index = 0; index < 10000000; index++){
    t1.insert("KEY1", "VALUE1");
  }

  // Check memory usage
  process_mem_usage(vm, res);
  BOOST_MESSAGE("VM: " << vm << "; RSS: " << res);
  // Verify memory usage change less than 1%
  BOOST_CHECK_CLOSE(vm/vm_base, 1.0, 1.0);
  BOOST_CHECK_CLOSE(res/res_base, 1.0, 1.0);

  // Insert the same value 10 million times
  for (int index = 0; index < 10000000; index++){
    t1.insert("KEY1", "VALUE1");
  }

  // Check memory usage
  process_mem_usage(vm, res);
  BOOST_MESSAGE("VM: " << vm << "; RSS: " << res);
  // Verify memory usage change less than 1%
  BOOST_CHECK_CLOSE(vm/vm_base, 1.0, 1.0);
  BOOST_CHECK_CLOSE(res/res_base, 1.0, 1.0);
}

BOOST_AUTO_TEST_SUITE_END()

