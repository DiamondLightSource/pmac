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

#include "StringHashtable.h"

//////////////////////////////////////////////////////////////////////////////
//
// process_mem_usage(double &, double &) - takes two doubles by reference,
// attempts to read the system-dependent data for a process' virtual memory
// size and resident set size, and return the results in KB.
//
// On failure, returns 0.0, 0.0

void process_mem_usage(double& vm_usage, double& resident_set)
{
   using std::ios_base;
   using std::ifstream;
   using std::string;

   vm_usage     = 0.0;
   resident_set = 0.0;

   // 'file' stat seems to give the most reliable results
   //
   ifstream stat_stream("/proc/self/stat",ios_base::in);

   // dummy vars for leading entries in stat that we don't care about
   //
   string pid, comm, state, ppid, pgrp, session, tty_nr;
   string tpgid, flags, minflt, cminflt, majflt, cmajflt;
   string utime, stime, cutime, cstime, priority, nice;
   string O, itrealvalue, starttime;

   // the two fields we want
   //
   unsigned long vsize;
   long rss;

   stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
               >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
               >> utime >> stime >> cutime >> cstime >> priority >> nice
               >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

   stat_stream.close();

   long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
   vm_usage     = vsize / 1024.0;
   resident_set = rss * page_size_kb;
}

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

