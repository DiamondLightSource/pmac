TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
# The unit test executable with the actual unit tests depends on the boost 
# unit test framework so we can only build it if boost has been configured
ifdef BOOST
  PROD_IOC_Linux += pmac-test
  pmac-test_SRCS += pmac-test.cpp
  pmac-test_SRCS += test_StringHashtable.cpp
  pmac-test_SRCS += test_PMACCommandStore.cpp

  # Add pmac tests for new classes like this:
  #pmac-test_SRCS += test_<test name>.cpp
  
  pmac-test_LIBS += boost_unit_test_framework pmacAsynMotorPort
  boost_unit_test_framework_DIR=$(BOOST_LIB)
endif

include $(TOP)/configure/RULES

