TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

DBD += pmacAsynMotorPort.dbd
DBD += pmacAsynMotorPortVxWorks.dbd

DBD += pmac.dbd
pmac_DBD += base.dbd
pmac_DBD += asyn.dbd
pmac_DBD += calcSupport.dbd
pmac_DBD += busySupport.dbd
pmac_DBD += motorSupport.dbd

LIBRARY_IOC += pmacAsynMotorPort

INC += pmacController.h
INC += pmacAxis.h
INC += pmacCSController.h
INC += pmacCSAxis.h
INC += pmacCSMonitor.h
INC += pmacCsGroups.h
INC += Hashtable.h
INC += StringHashtable.h
INC += IntegerHashtable.h
INC += pmacGroupsHashtable.h
INC += CharIntHashtable.h
INC += pmacCommandStore.h
INC += pmacMessageBroker.h
INC += pmacTrajectory.h
INC += pmacHardwareInterface.h
INC += pmacHardwareTurbo.h
INC += pmacHardwarePower.h
INC += pmacCallbackStore.h
INC += pmacCallbackInterface.h
INC += pmacDebugger.h

LIB_LIBS += motor
LIB_LIBS += asyn
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

pmacAsynMotorPort_SRCS = pmacDebugger.cpp
pmacAsynMotorPort_SRCS += pmacCSMonitor.cpp
pmacAsynMotorPort_SRCS += pmacController.cpp
pmacAsynMotorPort_SRCS += pmacAxis.cpp
pmacAsynMotorPort_SRCS += pmacCSController.cpp
pmacAsynMotorPort_SRCS += pmacCSAxis.cpp
pmacAsynMotorPort_SRCS += pmacCsGroups.cpp
pmacAsynMotorPort_SRCS += pmacGroupsHashtable.cpp
pmacAsynMotorPort_SRCS += CharIntHashtable.cpp
pmacAsynMotorPort_SRCS += StringHashtable.cpp
pmacAsynMotorPort_SRCS += IntegerHashtable.cpp
pmacAsynMotorPort_SRCS += Hashtable.cpp
pmacAsynMotorPort_SRCS += pmacCommandStore.cpp
pmacAsynMotorPort_SRCS += pmacMessageBroker.cpp
pmacAsynMotorPort_SRCS += pmacTrajectory.cpp
pmacAsynMotorPort_SRCS += pmacHardwareInterface.cpp
pmacAsynMotorPort_SRCS += pmacHardwareTurbo.cpp
pmacAsynMotorPort_SRCS += pmacHardwarePower.cpp
pmacAsynMotorPort_SRCS += pmacCallbackStore.cpp
pmacAsynMotorPort_SRCS += pmacCallbackInterface.cpp

# do debug build
# CXXFLAGS= -g -O0 -fPIC

# Enable wonderful C++ 11 features
USR_CXXFLAGS_Linux += -std=c++11

# specify all source files to be compiled and added to the library
#pmac_SRCS += xxx

pmacAsynMotorPort_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

