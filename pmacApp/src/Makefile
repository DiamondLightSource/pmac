TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

DBD += pmacAsynMotorPort.dbd
DBD += pmacAsynMotorPortVxWorks.dbd


LIBRARY_IOC += pmacAsynMotorPort

INC += pmacController.h
INC += pmacAxis.h
INC += pmacCSController.h
INC += pmacCSAxis.h
INC += pmacCSMonitor.h
INC += Hashtable.h
INC += StringHashtable.h
INC += IntegerHashtable.h
INC += pmacCommandStore.h
INC += pmacMessageBroker.h
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
pmacAsynMotorPort_SRCS += pmacCallbackStore.cpp
pmacAsynMotorPort_SRCS += pmacCallbackInterface.cpp

# specify all source files to be compiled and added to the library
#pmac_SRCS += xxx

pmacAsynMotorPort_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

