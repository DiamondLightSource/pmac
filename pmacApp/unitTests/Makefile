TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
# The unit test executable with the actual unit tests depends on the boost 
# unit test framework so we can only build it if boost has been configured
ifeq ($(WITH_BOOST), YES)
  PROD_IOC_Linux += pmac-test
  pmac-test_SRCS += pmac-test.cpp
  pmac-test_SRCS += pmacTestUtilities.cpp  
  pmac-test_SRCS += MockPMACAsynDriver.cpp
  pmac-test_SRCS += test_StringHashtable.cpp
  pmac-test_SRCS += test_IntegerHashtable.cpp
  pmac-test_SRCS += test_PMACCommandStore.cpp
  pmac-test_SRCS += test_PMACGroupsHashtable.cpp
  pmac-test_SRCS += test_CharIntHashtable.cpp
  pmac-test_SRCS += test_PMACMessageBroker.cpp
  pmac-test_SRCS += test_PMACCsGroups.cpp
  pmac-test_SRCS += test_PMACTrajectory.cpp
  #pmac-test_SRCS += test_PMACController.cpp

  # Add pmac tests for new classes like this:
  #pmac-test_SRCS += test_<test name>.cpp

  pmac-test_LIBS += pmacAsynMotorPort
  pmac-test_LIBS += asyn
  pmac-test_LIBS += $(EPICS_BASE_IOC_LIBS)

  ifdef BOOST_LIB
    boost_unit_test_framework_DIR=$(BOOST_LIB)
    pmac-test_LIBS += boost_unit_test_framework
  else
    pmac-test_SYS_LIBS += boost_unit_test_framework
  endif
endif

USR_INCLUDES += $(BOOST_INCLUDE)

# Executable for testing with Valgrind
PROD_IOC_Linux += pmac-valgrind
pmac-valgrind_SRCS += pmac-valgrind.cpp
pmac-valgrind_LIBS += pmacAsynIPPort
pmac-valgrind_LIBS += pmacAsynMotorPort
pmac-valgrind_LIBS += asyn
mac-valgrind_LIBS += $(EPICS_BASE_IOC_LIBS)


include $(TOP)/configure/RULES

