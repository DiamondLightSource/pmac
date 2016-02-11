TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += pmacAsynMotorPort

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install pmac.dbd into <top>/dbd
#DBD += pmac.dbd
INC += Hashtable.h
INC += StringHashtable.h

LIB_LIBS += motor
LIB_LIBS += asyn
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

#pmacAsynMotorPort_SRCS = pmacController.cpp
#pmacAsynMotorPort_SRCS += pmacAxis.cpp
#pmacAsynMotorPort_SRCS += pmacCsGroups.cpp
#pmacAsynMotorPort_SRCS += pmacGroupsHashtable.cpp
#pmacAsynMotorPort_SRCS += CharIntHashtable.cpp
pmacAsynMotorPort_SRCS += StringHashtable.cpp
pmacAsynMotorPort_SRCS += Hashtable.cpp
#pmacAsynMotorPort_SRCS += pmacCommandStore.cpp
#pmacAsynMotorPort_SRCS += pmacMessageBroker.cpp
#pmacAsynMotorPort_SRCS += pmacCallbackStore.cpp
#pmacAsynMotorPort_SRCS += pmacCallbackInterface.cpp

# specify all source files to be compiled and added to the library
#pmac_SRCS += xxx

pmacAsynMotorPort_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

