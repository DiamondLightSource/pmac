/** pmac-test.cpp
 * 
 *  This file just defines the basic level of the boost unittest
 *  system for pmac classes. By doing this here, the actual unittests
 *  can span multiple source files.
 *
 *  Author: Alan Greer, Diamond Light Source.
 *          11. Feb 2016
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "PMAC Unit Tests"
#include <boost/test/unit_test.hpp>


